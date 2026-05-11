#include "wireless_transport_manager.h"
#include "wireless_scenario_builder.h"
#include "cross_layer_metrics_collector.h"

#include "ns3/log.h"
#include "ns3/on-off-helper.h"
#include "ns3/onoff-application.h"
#include "ns3/packet-sink.h"

#include <chrono>
#include <iomanip>
#include <algorithm>
#include <sstream>
#include <set>

NS_LOG_COMPONENT_DEFINE("WirelessTransportManager");

namespace net_cfg = robot_fl_config::network;


static std::map<std::string, uint64_t> g_rxBytes;
static std::map<std::string, uint64_t> g_targetBytes;
static std::map<std::string, uint16_t> g_uploadPorts;
static std::map<std::string, Ptr<OnOffApplication>> g_flowSources;
static std::map<std::string, Ptr<PacketSink>> g_flowSinks;

namespace
{

std::string
BuildTransportKey(const std::string& ueIp, uint16_t port)
{
    std::ostringstream oss;
    oss << ueIp << ":" << port;
    return oss.str();
}

std::string
BuildTransportKey(const Ipv4Address& ueIp, uint16_t port)
{
    std::ostringstream oss;
    oss << ueIp;
    return BuildTransportKey(oss.str(), port);
}

const char*
GetTransportProtocolName(AppTransportProtocol protocol)
{
    switch (protocol)
    {
    case AppTransportProtocol::UDP:
        return "UDP";
    case AppTransportProtocol::TCP:
        return "TCP";
    }
    return "UNKNOWN";
}

const char*
GetTransportSocketFactoryName(AppTransportProtocol protocol)
{
    switch (protocol)
    {
    case AppTransportProtocol::UDP:
        return "ns3::UdpSocketFactory";
    case AppTransportProtocol::TCP:
        return "ns3::TcpSocketFactory";
    }
    return "ns3::UdpSocketFactory";
}

void
CloseAndDisposeOnOff(const std::string& transportKey, Ptr<OnOffApplication> source)
{
    if (!source)
    {
        return;
    }


    Ptr<Socket> socket = source->GetSocket();
    if (socket)
    {
        socket->Close();
    }


    source->Dispose();
    NS_LOG_INFO("CloseAndDisposeOnOff: stopped source for " << transportKey << ".");
}

void
CloseAndDisposePacketSink(const std::string& transportKey, Ptr<PacketSink> sink)
{
    if (!sink)
    {
        return;
    }


    Ptr<Socket> listeningSocket = sink->GetListeningSocket();
    if (listeningSocket)
    {
        listeningSocket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
        listeningSocket->Close();
    }

    for (const Ptr<Socket>& acceptedSocket : sink->GetAcceptedSockets())
    {
        if (!acceptedSocket)
        {
            continue;
        }
        acceptedSocket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
        acceptedSocket->Close();
    }

    sink->Dispose();
    NS_LOG_INFO("CloseAndDisposePacketSink: stopped sink for " << transportKey << ".");
}

void
StopTransportFlowNowInternal(const std::string& transportKey, const std::string& reason)
{


    g_stoppedTransportFlows.insert(transportKey);

    auto sourceIt = g_flowSources.find(transportKey);
    if (sourceIt != g_flowSources.end())
    {
        CloseAndDisposeOnOff(transportKey, sourceIt->second);
        g_flowSources.erase(sourceIt);
    }

    auto sinkIt = g_flowSinks.find(transportKey);
    if (sinkIt != g_flowSinks.end())
    {
        CloseAndDisposePacketSink(transportKey, sinkIt->second);
        g_flowSinks.erase(sinkIt);
    }

    NS_LOG_INFO("StopTransportFlowNowInternal: transportKey=" << transportKey
                << ", reason=" << reason << ".");
}

}


std::map<std::string, double> endOfStreamTimes;
std::map<std::string, bool> selectedUeFinshed;
std::map<std::string, std::string> transportInfo;


std::pair<uint16_t, uint16_t>
WirelessTransportManager::getUeRntiCellid(Ptr<ns3::NetDevice> ueNetDevice)
{
    NS_LOG_INFO("getUeRntiCellid: Attempting to get RNTI and CellID for NetDevice: "
                << (ueNetDevice ? ueNetDevice->GetInstanceTypeId().GetName() : "null"));

    if (!ueNetDevice)
    {
        NS_LOG_INFO("getUeRntiCellid: Input ueNetDevice is null. Returning {0, 0}.");
        return {0, 0};
    }

    auto lteDevice = ueNetDevice->GetObject<LteUeNetDevice>();
    if (!lteDevice)
    {
        NS_LOG_INFO("getUeRntiCellid: NetDevice is not an LteUeNetDevice or GetObject failed. Device Type: "
                    << ueNetDevice->GetInstanceTypeId().GetName() << ". Returning {0, 0}.");
        return {0, 0};
    }

    NS_LOG_INFO("getUeRntiCellid: LteUeNetDevice found: " << lteDevice);

    if (!lteDevice->GetRrc())
    {
        NS_LOG_INFO("getUeRntiCellid: LteUeNetDevice " << lteDevice
                                                       << " has no RRC instance. Returning {0, 0}.");
        return {0, 0};
    }

    LteUeRrc::State rrcState = lteDevice->GetRrc()->GetState();
    NS_LOG_INFO("getUeRntiCellid: LteUeRrc State: " << rrcState);


    if (rrcState != LteUeRrc::CONNECTED_NORMALLY &&
        rrcState != LteUeRrc::CONNECTED_HANDOVER)
    {
        NS_LOG_INFO("getUeRntiCellid: UE RRC not in a connected state (Current state: "
                    << rrcState << "). Returning {0, 0}.");
        return {0, 0};
    }

    auto rnti = lteDevice->GetRrc()->GetRnti();
    auto cellid = lteDevice->GetRrc()->GetCellId();
    NS_LOG_INFO("getUeRntiCellid: Successfully retrieved RNTI: " << rnti
                                                                 << ", CellID: " << cellid
                                                                 << " for UE connected to eNB.");
    return std::make_pair(rnti, cellid);
}


std::vector<NodesIps>
WirelessTransportManager::nodeToIps()
{
    std::vector<NodesIps> nodes_ips_list;

    ueIpSet.clear();
    ueIpToIndex.clear();
    ueIndexToIp.clear();

    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        Ptr<Node> ueNode = ueNodes.Get(i);

        NS_ASSERT(ueNode);
        if (!ueNode)
        {
            continue;
        }

        Ptr<Ipv4> ipv4 = ueNode->GetObject<Ipv4>();

        NS_ASSERT(ipv4);
        if (ipv4)
        {
            NS_ASSERT(ipv4->GetNInterfaces() > 1);
            if (ipv4->GetNInterfaces() > 1)
            {

                Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1, 0);
                Ipv4Address ipAddr = iaddr.GetLocal();

                std::ostringstream ipInfo;
                ipInfo << ipAddr;
                std::string ipStr = ipInfo.str();

                ueIpSet.insert(ipStr);
                ueIpToIndex[ipStr] = i;
                ueIndexToIp[i] = ipStr;

                nodes_ips_list.push_back(NodesIps(ueNode->GetId(), i, ipAddr));
            }
        }
    }
    return nodes_ips_list;
}


void
WirelessTransportManager::sinkRxCallback(Ptr<const Packet> packet,
                             const Address& from,
                             const Address& to,
                             const SeqTsSizeHeader& header)
{
    (void)header;

    if (!packet)
    {
        return;
    }

    uint32_t packetSize = packet->GetSize();
    if (packetSize == 0)
    {
        return;
    }


    InetSocketAddress address = InetSocketAddress::ConvertFrom(from);
    Ipv4Address senderIp = address.GetIpv4();
    if (!InetSocketAddress::IsMatchingType(to))
    {
        NS_LOG_WARN("sinkRxCallback: local address is not IPv4, skip packet accounting.");
        return;
    }
    const uint16_t dstPort = InetSocketAddress::ConvertFrom(to).GetPort();
    const std::string transportKey = BuildTransportKey(senderIp, dstPort);

    if (!g_activeTransportFlows.count(transportKey) || g_stoppedTransportFlows.count(transportKey))
    {
        NS_LOG_DEBUG("sinkRxCallback: ignore inactive/stopped transportKey=" << transportKey);
        return;
    }


    g_rxBytes[transportKey] += packetSize;


    auto itTarget = g_targetBytes.find(transportKey);
    if (itTarget != g_targetBytes.end())
    {
        uint64_t targetBytes = itTarget->second;
        uint64_t currentBytes = g_rxBytes[transportKey];

        if (currentBytes >= targetBytes &&
            endOfStreamTimes.find(transportKey) == endOfStreamTimes.end())
        {
            double receiveTime = Simulator::Now().GetSeconds();
            endOfStreamTimes[transportKey] = receiveTime;
            selectedUeFinshed[transportKey] = true;


            g_stoppedTransportFlows.insert(transportKey);
            Simulator::ScheduleNow(&StopTransportFlowNowInternal,
                                   transportKey,
                                   std::string("target-bytes-reached"));
            NS_LOG_INFO(std::fixed << std::setprecision(6)
                                   << receiveTime
                                   << "s: UE " << senderIp
                                   << " transportKey=" << transportKey
                                   << " reached target bytes " << targetBytes
                                   << " (current " << currentBytes << "). Mark as finished.");
        }
    }
}


void
WirelessTransportManager::sendStream(Ptr<Node> sendingNode,
                         Ptr<Node> receivingNode,
                         int size)
{
    if (!sendingNode || !receivingNode)
    {
        NS_LOG_ERROR("sendStream: One or both nodes are null. Sending Node: "
                     << sendingNode << ", Receiving Node: " << receivingNode
                     << ". Aborting stream setup.");
        return;
    }

    if (size <= 0)
    {
        NS_LOG_INFO("sendStream: Requested size is non-positive (" << size
                                                                  << "). Nothing to send.");
        return;
    }

    uint64_t totalBytes = static_cast<uint64_t>(size);
    const SeqTsSizeHeader seqHeader;
    const uint32_t seqHeaderBytes = seqHeader.GetSerializedSize();

    if (net_cfg::kAppPacketSizeBytes <= seqHeaderBytes)
    {
        NS_LOG_ERROR("sendStream: PacketSize " << net_cfg::kAppPacketSizeBytes
                     << " is too small for SeqTsSizeHeader " << seqHeaderBytes
                     << ". Aborting.");
        return;
    }

    const uint32_t payloadBytesPerPacket = net_cfg::kAppPacketSizeBytes - seqHeaderBytes;
    const uint64_t packetCount = (totalBytes + payloadBytesPerPacket - 1) / payloadBytesPerPacket;


    static uint16_t port_counter = net_cfg::kFirstUploadPort;
    uint16_t current_port = port_counter++;


    Ptr<Ipv4> ipv4Receiver = receivingNode->GetObject<Ipv4>();
    if (!ipv4Receiver || ipv4Receiver->GetNInterfaces() <= 1)
    {
        NS_LOG_ERROR("sendStream: No valid IPv4 interface 1 on receiving node "
                     << receivingNode->GetId() << ". Aborting.");
        return;
    }
    Ipv4Address ipAddrReceiver = ipv4Receiver->GetAddress(1, 0).GetLocal();


    Ptr<Ipv4> ipv4Sender = sendingNode->GetObject<Ipv4>();
    if (!ipv4Sender || ipv4Sender->GetNInterfaces() <= 1)
    {
        NS_LOG_ERROR("sendStream: No valid IPv4 interface 1 on sending node "
                     << sendingNode->GetId() << ". Aborting.");
        return;
    }
    Ipv4Address senderIp = ipv4Sender->GetAddress(1, 0).GetLocal();

    std::ostringstream senderIpInfo;
    senderIpInfo << senderIp;
    const std::string senderIpStr = senderIpInfo.str();
    const std::string transportKey = BuildTransportKey(senderIpStr, current_port);
    auto ueIdIt = ueIpToIndex.find(senderIpInfo.str());
    bool hasMappedUeId = (ueIdIt != ueIpToIndex.end());
    if (!hasMappedUeId)
    {
        NS_LOG_WARN("sendStream: sender IP " << senderIp
                    << " not found in ueIpToIndex; tx trace stats may be incomplete.");
    }


    g_rxBytes[transportKey] = 0;
    g_targetBytes[transportKey] = totalBytes;
    g_uploadPorts[transportKey] = current_port;
    endOfStreamTimes.erase(transportKey);

    NS_LOG_INFO("sendStream: Node " << sendingNode->GetId() << " (IP " << senderIp
                                    << ") sending payload target " << totalBytes
                                    << " bytes to Node " << receivingNode->GetId()
	                                    << " (IP " << ipAddrReceiver
	                                    << "), port " << current_port
	                                    << ", transport="
	                                    << GetTransportProtocolName(net_cfg::kAppTransportProtocol)
	                                    << ", transportKey=" << transportKey
                                    << ", payloadPerPacket=" << payloadBytesPerPacket
                                    << ", estimatedPacketsToReachTarget=" << packetCount << ".");

    auto previousTransportIt = transportInfo.find(senderIpStr);
    if (previousTransportIt != transportInfo.end())
    {

        StopTransportFlowNowInternal(previousTransportIt->second, "replace-with-new-flow");
        g_flowTraceStats.erase(previousTransportIt->second);
        g_activeTransportFlows.erase(previousTransportIt->second);
        g_stoppedTransportFlows.erase(previousTransportIt->second);
        selectedUeFinshed.erase(previousTransportIt->second);
        endOfStreamTimes.erase(previousTransportIt->second);
        g_rxBytes.erase(previousTransportIt->second);
        g_targetBytes.erase(previousTransportIt->second);
        g_uploadPorts.erase(previousTransportIt->second);
    }

    selectedUeFinshed.erase(transportKey);
    transportInfo[senderIpStr] = transportKey;
    selectedUeFinshed[transportKey] = false;
    g_activeTransportFlows.insert(transportKey);
    g_stoppedTransportFlows.erase(transportKey);

    FlowTraceStats flowStats;
    if (hasMappedUeId)
    {
        flowStats.ueId = ueIdIt->second;
    }
    g_flowTraceStats[transportKey] = flowStats;


    Time sinkStartDelay = net_cfg::kSinkStartDelay;
    Time sourceStartDelay = net_cfg::kSourceStartDelay;


    PacketSinkHelper packetSinkHelper(GetTransportSocketFactoryName(net_cfg::kAppTransportProtocol),
	                                      InetSocketAddress(Ipv4Address::GetAny(),
	                                                        current_port));
    packetSinkHelper.SetAttribute("EnableSeqTsSizeHeader",
                                  BooleanValue(net_cfg::kEnableSeqTsSizeHeader));
    ApplicationContainer sinkApps = packetSinkHelper.Install(receivingNode);
    sinkApps.Start(sinkStartDelay);

    if (sinkApps.GetN() == 0)
    {
        NS_LOG_ERROR("sendStream: Failed to install PacketSink on Node "
                     << receivingNode->GetId() << ". Aborting.");
        return;
    }

    Ptr<PacketSink> sink = DynamicCast<PacketSink>(sinkApps.Get(0));
    if (!sink)
    {
        NS_LOG_ERROR("sendStream: Failed to cast Application to PacketSink on Node "
                     << receivingNode->GetId() << ". Aborting.");
        return;
    }


    sink->TraceConnectWithoutContext("RxWithSeqTsSize",
                                     MakeCallback(&CrossLayerMetricsCollector::traceAppRx));
    sink->TraceConnectWithoutContext("RxWithSeqTsSize",
                                     MakeCallback(&WirelessTransportManager::sinkRxCallback));
    g_flowSinks[transportKey] = sink;


    OnOffHelper onOffHelper(GetTransportSocketFactoryName(net_cfg::kAppTransportProtocol),
	                            InetSocketAddress(ipAddrReceiver, current_port));
    onOffHelper.SetAttribute("DataRate",
                             DataRateValue(DataRate(net_cfg::kAppUploadDataRate)));
    onOffHelper.SetAttribute("PacketSize",
                             UintegerValue(net_cfg::kAppPacketSizeBytes));
    onOffHelper.SetAttribute("EnableSeqTsSizeHeader",
                             BooleanValue(net_cfg::kEnableSeqTsSizeHeader));
    onOffHelper.SetAttribute("OnTime",
                             StringValue(net_cfg::kOnTimeRandomVariable));
    onOffHelper.SetAttribute("OffTime",
                             StringValue(net_cfg::kOffTimeRandomVariable));

    ApplicationContainer sourceApps = onOffHelper.Install(sendingNode);
    sourceApps.Start(sourceStartDelay);
    if (sourceApps.GetN() == 0)
    {
        NS_LOG_ERROR("sendStream: Failed to install OnOffApplication on Node "
                     << sendingNode->GetId() << ".");
        return;
    }

    Ptr<OnOffApplication> source = DynamicCast<OnOffApplication>(sourceApps.Get(0));
    if (!source)
    {
        NS_LOG_ERROR("sendStream: Failed to cast source application to OnOffApplication.");
        return;
    }
    if (hasMappedUeId)
    {
        source->TraceConnectWithoutContext("TxWithSeqTsSize",
                                           MakeBoundCallback(&CrossLayerMetricsCollector::traceAppTx,
                                                             transportKey));
    }
    g_flowSources[transportKey] = source;
}

std::vector<UeUploadRoundStat>
WirelessTransportManager::collectRoundUploadStats(
    const std::vector<ClientModels> &selected_clients_for_round)
{
    std::vector<UeUploadRoundStat> roundStats;
    std::vector<NodesIps> all_nodes_ips = nodeToIps();
    std::map<uint32_t, uint64_t> selectedTargetBytesByNodeId;

    for (const auto &client_model : selected_clients_for_round)
    {
        if (!client_model.node)
        {
            continue;
        }

        selectedTargetBytesByNodeId[client_model.node->GetId()] =
            static_cast<uint64_t>(std::max(client_model.nodeModelSize, 0));
    }

    roundStats.reserve(all_nodes_ips.size());

    for (const auto &node_ip : all_nodes_ips)
    {
        std::ostringstream ipInfo;
        ipInfo << node_ip.ip;
        std::string ipStr = ipInfo.str();

        auto selectedTargetIt =
            selectedTargetBytesByNodeId.find(node_ip.nodeId);
        bool selected = selectedTargetIt != selectedTargetBytesByNodeId.end();
        uint64_t targetBytes =
            selected ? selectedTargetIt->second : static_cast<uint64_t>(0);

        std::string transportKey;
        auto transportIt = transportInfo.find(ipStr);
        if (transportIt != transportInfo.end())
        {
            transportKey = transportIt->second;
        }

        auto internalTargetIt = g_targetBytes.find(transportKey);
        if (selected && internalTargetIt != g_targetBytes.end())
        {
            targetBytes = internalTargetIt->second;
        }

        uint64_t uploadedBytes = 0;
        auto flowIt = g_flowTraceStats.find(transportKey);
        if (flowIt != g_flowTraceStats.end())
        {
            uploadedBytes = flowIt->second.rxBytes;
        }

        auto portIt = g_uploadPorts.find(transportKey);
        int uePort = (portIt != g_uploadPorts.end())
                         ? static_cast<int>(portIt->second)
                         : -1;

        auto finishIt = endOfStreamTimes.find(transportKey);
        bool finished = finishIt != endOfStreamTimes.end();
        double finishTime = finished ? finishIt->second : -1.0;

        double completionRatio = 0.0;
        if (selected && targetBytes > 0)
        {
            completionRatio = std::min(
                1.0, static_cast<double>(uploadedBytes) / targetBytes);
        }

        roundStats.push_back({node_ip.nodeId,
                              ipStr,
                              uePort,
                              selected,
                              finished,
                              targetBytes,
                              uploadedBytes,
                              completionRatio,
                              finishTime});
    }

    return roundStats;
}


bool
WirelessTransportManager::checkFinishedTransmission(const std::vector<NodesIps> &all_nodes_ips,
                                        const std::vector<ClientModels> &selected_clients_for_round)
{
    if (selected_clients_for_round.empty())
    {
        NS_LOG_INFO("checkFinishedTransmission: No clients selected for this round. "
                    "Transmission considered finished.");
        return true;
    }

    size_t finished_count = 0;

    for (const auto &client_model : selected_clients_for_round)
    {
        if (!client_model.node)
        {
            continue;
        }
        uint32_t clientNodeId = client_model.node->GetId();


        Ipv4Address client_ip;
        bool ip_found = false;
        for (const auto &nip : all_nodes_ips)
        {
            if (nip.nodeId == clientNodeId)
            {
                client_ip = nip.ip;
                ip_found = true;
                break;
            }
        }

        if (ip_found)
        {
            std::ostringstream ipInfo;
            ipInfo << client_ip;
            auto transportIt = transportInfo.find(ipInfo.str());
            if (transportIt != transportInfo.end() &&
                endOfStreamTimes.count(transportIt->second))
            {
                finished_count++;
                uint64_t uploadedBytes = 0;
                auto flowIt = g_flowTraceStats.find(transportIt->second);
                if (flowIt != g_flowTraceStats.end())
                {
                    uploadedBytes = flowIt->second.rxBytes;
                }
                NS_LOG_INFO(client_ip << " key=" << transportIt->second
                            << " uploaded=" << uploadedBytes);
            }
        }
    }

    bool all_finished = (finished_count == selected_clients_for_round.size());
    NS_LOG_INFO("checkFinishedTransmission: " << finished_count << " out of "
                                              << selected_clients_for_round.size()
                                              << " selected clients have completed. "
                                              << "All finished: " << (all_finished ? "YES" : "NO"));
    return all_finished;
}

void
WirelessTransportManager::stopRoundApplicationsNow()
{
    std::set<std::string> transportKeys;
    for (const auto& item : g_flowSources)
    {
        transportKeys.insert(item.first);
    }
    for (const auto& item : g_flowSinks)
    {
        transportKeys.insert(item.first);
    }

    NS_LOG_INFO(Simulator::Now().GetSeconds()
                << "s: stopRoundApplicationsNow: stopping "
                << transportKeys.size() << " transport flows.");

    for (const std::string& transportKey : transportKeys)
    {
        StopTransportFlowNowInternal(transportKey, "round-end-or-timeout");
    }
}


void
WirelessTransportManager::roundCleanup()
{
    NS_LOG_INFO(Simulator::Now().GetSeconds()
                << "s: Starting round cleanup: clearing per-round state tables.");


    endOfStreamTimes.clear();
    g_rxBytes.clear();
    g_targetBytes.clear();
    g_uploadPorts.clear();
    g_flowSources.clear();
    g_flowSinks.clear();
    g_ueNetBaseline.clear();
    CrossLayerMetricsCollector::resetRealtimeRoundState();

    transportInfo.clear();
    selectedUeFinshed.clear();

    NS_LOG_INFO(Simulator::Now().GetSeconds()
                << "s: ns-3 round cleanup finished.");
}
