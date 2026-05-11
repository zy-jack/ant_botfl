#include "cross_layer_metrics_collector.h"
#include "robot_fl_state_types.h"
#include "wireless_scenario_builder.h"
#include "wireless_transport_manager.h"
#include "ns3/log.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

NS_LOG_COMPONENT_DEFINE("CrossLayerMetricsCollector");

namespace fl_cfg = robot_fl_config::fl;


std::map<uint16_t, std::map<uint16_t, double>> sinrUe;
std::map<uint16_t, std::map<uint16_t, double>> rsrpUe;

std::map<uint32_t, UeNetBaseline> g_ueNetBaseline;
std::map<std::string, FlowTraceStats> g_flowTraceStats;
std::set<std::string> g_activeTransportFlows;
std::set<std::string> g_stoppedTransportFlows;

extern std::vector<ClientModels> clientsInfoGlobal;
extern int roundNumber;

namespace
{

bool
TryBuildTransportKey(const Address& from, const Address& to, std::string& transportKey)
{
  if (!InetSocketAddress::IsMatchingType(from) || !InetSocketAddress::IsMatchingType(to))
  {
    return false;
  }

  std::ostringstream ipInfo;
  ipInfo << InetSocketAddress::ConvertFrom(from).GetIpv4();
  transportKey = ipInfo.str() + ":" + std::to_string(InetSocketAddress::ConvertFrom(to).GetPort());
  return true;
}

bool
TryGetCurrentTransportKeyForUe(uint32_t ueId, std::string& transportKey)
{
  auto itIp = ueIndexToIp.find(ueId);
  if (itIp == ueIndexToIp.end())
  {
    return false;
  }

  auto itTransport = transportInfo.find(itIp->second);
  if (itTransport == transportInfo.end())
  {
    return false;
  }

  if (!g_activeTransportFlows.count(itTransport->second))
  {
    return false;
  }

  transportKey = itTransport->second;
  return true;
}

const FlowTraceStats*
GetActiveFlowTraceStatsForUe(uint32_t ueId)
{
  std::string transportKey;
  if (!TryGetCurrentTransportKeyForUe(ueId, transportKey))
  {
    return nullptr;
  }

  auto itFlow = g_flowTraceStats.find(transportKey);
  if (itFlow == g_flowTraceStats.end())
  {
    return nullptr;
  }

  return &itFlow->second;
}

bool
IsUeSelected(uint32_t ueId)
{
  std::string transportKey;
  return TryGetCurrentTransportKeyForUe(ueId, transportKey);
}

bool
IsUeFinished(uint32_t ueId)
{
  std::string transportKey;
  if (!TryGetCurrentTransportKeyForUe(ueId, transportKey))
  {
    return false;
  }

  auto itFinished = selectedUeFinshed.find(transportKey);
  return itFinished != selectedUeFinshed.end() && itFinished->second;
}

}


std::pair<double, double> CrossLayerMetricsCollector::getRsrpSinr(uint32_t nodeIdx) {
  Ptr<NetDevice> ueDevice = ueDevs.Get(nodeIdx);
  if (!ueDevice) {
    NS_LOG_INFO("getRsrpSinr: UE device at index " << nodeIdx << " is null.");
    return {0.0, 0.0};
  }
  auto lteUeNetDevice = ueDevice->GetObject<LteUeNetDevice>();
  if (!lteUeNetDevice) {
    NS_LOG_INFO("getRsrpSinr: NetDevice at index "
                 << nodeIdx << " is not an LteUeNetDevice.");
    return {0.0, 0.0};
  }
  auto rrc = lteUeNetDevice->GetRrc();


  if (!rrc || (rrc->GetState() != LteUeRrc::CONNECTED_NORMALLY &&
               rrc->GetState() != LteUeRrc::CONNECTED_HANDOVER)) {
    NS_LOG_INFO("getRsrpSinr: UE Node "
                 << ueNodes.Get(nodeIdx)->GetId()
                 << " RRC not in connected state. State: "
                 << (rrc ? rrc->GetState() : LteUeRrc::IDLE_START));
    return {0.0, 0.0};
  }

  auto rnti = rrc->GetRnti();
  auto cellId = rrc->GetCellId();


  double rsrp = 0.0;
  double sinr = 0.0;
  if (rsrpUe.count(cellId) && rsrpUe[cellId].count(rnti)) {
    rsrp = rsrpUe[cellId][rnti];
  }
  if (sinrUe.count(cellId) && sinrUe[cellId].count(rnti)) {
    sinr = sinrUe[cellId][rnti];
  }


  return {rsrp, sinr};
}


void CrossLayerMetricsCollector::updateAllClientsGlobalInfo(int trainingTime, int modelSizeBytes) {
  NS_LOG_INFO("Updating global client information for all UEs.");
  clientsInfoGlobal.clear();

	  for (uint32_t i = 0; i < ueNodes.GetN(); ++i) {
	    auto [rsrp, sinr] = getRsrpSinr(i);
	    double placeholderAccuracy = fl_cfg::kPlaceholderAccuracy;
    clientsInfoGlobal.emplace_back(ueNodes.Get(i), trainingTime,
                                   modelSizeBytes, rsrp, sinr,
                                   placeholderAccuracy);
    NS_LOG_DEBUG("  UE Node " << ueNodes.Get(i)->GetId() << ": RSRP=" << rsrp
                              << " dBm, SINR=" << sinr << " dB.");
  }
  NS_LOG_INFO("Global client information updated for "
              << clientsInfoGlobal.size() << " UEs.");
}


void CrossLayerMetricsCollector::reportUeSinrRsrp(uint16_t cellId,
                      uint16_t rnti,
                      double rsrp,
                      double sinr,
                      uint8_t componentCarrierId) {
    sinrUe[cellId][rnti] = SinrToDb(sinr);
    rsrpUe[cellId][rnti] = RsrpToDbm(rsrp);
    NS_LOG_DEBUG("ReportUeSinrRsrp: Stored SINR=" << sinr << " and RSRP=" << rsrp << " for CellID=" << cellId << ", RNTI=" << rnti);
}

double CrossLayerMetricsCollector::Safe10Log10(double x){
  return (x > 0.0 && std::isfinite(x)) ? 10.0 * std::log10(x) : -300.0;
}

double CrossLayerMetricsCollector::RsrpToDbm(double rsrp){
  return Safe10Log10(rsrp) + 30.0;
}

double CrossLayerMetricsCollector::SinrToDb(double sinr){
  return Safe10Log10(sinr);
}


void CrossLayerMetricsCollector::reportUeSinrRsrp(std::string context,
                      uint16_t cellId,
                      uint16_t rnti,
                      double rsrp,
                      double sinr,
                      uint8_t componentCarrierId) {
    NS_LOG_DEBUG("ReportUeSinrRsrp (context version) - Context: '" << context
                                                                   << "', CellID: " << cellId << ", RNTI: " << rnti
                                                                   << ", RSRP: " << std::fixed << std::setprecision(2) << rsrp << " dBm"
                                                                   << ", SINR: " << std::fixed << std::setprecision(2) << sinr << " dB"
                                                                   << ", CC ID: " << (unsigned int)componentCarrierId);

    NS_LOG_DEBUG("ReportUeSinrRsrp: Calling non-context version of ReportUeSinrRsrp.");
    reportUeSinrRsrp(cellId, rnti, rsrp, sinr, componentCarrierId);
    NS_LOG_DEBUG("ReportUeSinrRsrp: Returned from non-context version of ReportUeSinrRsrp.");
}


void CrossLayerMetricsCollector::setupRsrpSinrTracing() {
  for (uint32_t i = 0; i < ueNodes.GetN(); i++) {
    Ptr<LteUePhy> uePhy = ueDevs.Get(i)->GetObject<LteUeNetDevice>()->GetPhy();

    uePhy->TraceConnectWithoutContext(
        "ReportCurrentCellRsrpSinr",
        MakeCallback<void, uint16_t, uint16_t, double, double, uint8_t>(
            &ReportUeSinrRsrp));
  }
  NS_LOG_INFO("RSRP/SINR trace sources connected for UEs.");
}

void
CrossLayerMetricsCollector::resetRealtimeRoundState()
{
  g_flowTraceStats.clear();
  g_activeTransportFlows.clear();
  g_stoppedTransportFlows.clear();
}

void
CrossLayerMetricsCollector::traceAppTx(std::string transportKey,
                             Ptr<const Packet> packet,
                             const Address& from,
                             const Address& to,
                             const SeqTsSizeHeader& header)
{
  (void)from;
  (void)to;
  (void)header;

  if (!packet)
  {
    return;
  }


  if (!g_activeTransportFlows.count(transportKey) || g_stoppedTransportFlows.count(transportKey))
  {
    NS_LOG_DEBUG("traceAppTx: ignore inactive/stopped transportKey=" << transportKey);
    return;
  }

  FlowTraceStats& stats = g_flowTraceStats[transportKey];
  stats.txPackets += 1;
  stats.txBytes += packet->GetSize();
}

void
CrossLayerMetricsCollector::traceAppRx(Ptr<const Packet> packet,
                             const Address& from,
                             const Address& to,
                             const SeqTsSizeHeader& header)
{
  if (!packet)
  {
    return;
  }

  std::string transportKey;
  if (!TryBuildTransportKey(from, to, transportKey))
  {
    NS_LOG_DEBUG("traceAppRx: could not build transport key from addresses.");
    return;
  }


  if (!g_activeTransportFlows.count(transportKey) || g_stoppedTransportFlows.count(transportKey))
  {
    NS_LOG_DEBUG("traceAppRx: ignore inactive/stopped transportKey=" << transportKey);
    return;
  }

  FlowTraceStats& stats = g_flowTraceStats[transportKey];
  stats.rxPackets += 1;
  stats.rxBytes += packet->GetSize();

  const double delaySec = std::max(0.0, (Simulator::Now() - header.GetTs()).GetSeconds());
  stats.delaySum += delaySec;
  if (stats.hasLastDelay)
  {
    stats.jitterSum += std::abs(delaySec - stats.lastDelay);
  }
  stats.lastDelay = delaySec;
  stats.hasLastDelay = true;
}


void
CrossLayerMetricsCollector::networkInfo()
{
    static Time lastTime = Seconds(0);
    static uint64_t lastTotalRxBytes = 0;
    static uint64_t lastTotalTxBytes = 0;

	    Simulator::Schedule(fl_cfg::kNetworkInfoInterval, &CrossLayerMetricsCollector::networkInfo);

    uint64_t currentTotalRxBytes = 0;
    uint64_t currentTotalTxBytes = 0;
    std::map<uint32_t, UeAggStats> perUeStats;

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        const FlowTraceStats* flowStats = GetActiveFlowTraceStatsForUe(i);
        const bool selected = IsUeSelected(i);
        const bool isFinished = IsUeFinished(i);

        Ptr<MobilityModel> mob = ueNodes.Get(i)->GetObject<MobilityModel>();
        Vector pos = mob ? mob->GetPosition() : Vector();
        Vector vel = mob ? mob->GetVelocity() : Vector();
        double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
        auto [rsrp, sinr] = getRsrpSinr(i);
        client_info[i].x_pos = pos.x;
        client_info[i].y_pos = pos.y;
        client_info[i].velocity = speed;
        client_info[i].rsrp = rsrp;
        client_info[i].sinr = sinr;

        UeAggStats agg;
        if (flowStats)
        {
            agg.rxBytes = flowStats->rxBytes;
            agg.txBytes = flowStats->txBytes;
            agg.rxPackets = flowStats->rxPackets;
            agg.txPackets = flowStats->txPackets;
            agg.delaySum = flowStats->delaySum;
            agg.jitterSum = flowStats->jitterSum;
        }
        agg.selected = selected;
        agg.isFinished = isFinished;
        perUeStats[i] = agg;

        currentTotalRxBytes += agg.rxBytes;
        currentTotalTxBytes += agg.txBytes;

    }

    Time currentTime = Simulator::Now();
    double timeDiff = (currentTime - lastTime).GetSeconds();
    double instantRxThroughputMbps = 0.0;
    double instantTxThroughputMbps = 0.0;

    bool counterReset = (currentTotalRxBytes < lastTotalRxBytes) || (currentTotalTxBytes < lastTotalTxBytes);
    if (counterReset)
    {
        NS_LOG_WARN("networkInfo: detected counter reset, refreshing throughput baseline.");
        lastTotalRxBytes = currentTotalRxBytes;
        lastTotalTxBytes = currentTotalTxBytes;
        lastTime = currentTime;
    }
    else if (timeDiff > 0)
    {
        instantRxThroughputMbps =
            static_cast<double>(currentTotalRxBytes - lastTotalRxBytes) * 8.0 / timeDiff / 1e6;
        instantTxThroughputMbps =
            static_cast<double>(currentTotalTxBytes - lastTotalTxBytes) * 8.0 / timeDiff / 1e6;
    }

    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4)
            << currentTime.GetSeconds()
            << "s: Global TxThroughput=" << instantTxThroughputMbps
            << " Mbps, Global RxThroughput=" << instantRxThroughputMbps
            << " Mbps, TotalTxBytes=" << currentTotalTxBytes
            << ", TotalRxBytes=" << currentTotalRxBytes;
        NS_LOG_INFO(oss.str());
    }

    if (timeDiff > 0)
    {
        for (auto& kv : perUeStats)
        {
            uint32_t ueId = kv.first;
            const UeAggStats& agg = kv.second;
            UeNetBaseline& base = g_ueNetBaseline[ueId];

            double rxRateMbps = 0.0;
            double txRateMbps = 0.0;
            double delayNow = 0.0;
            double jitterNow = 0.0;

            if (base.initialized)
            {
                const uint64_t dRxBytes = agg.rxBytes - base.lastRxBytes;
                const uint64_t dTxBytes = agg.txBytes - base.lastTxBytes;
                const uint32_t dRxPackets = agg.rxPackets - base.lastRxPackets;
                const double dDelaySum = agg.delaySum - base.lastDelaySum;
                const double dJitterSum = agg.jitterSum - base.lastJitterSum;

                rxRateMbps = static_cast<double>(dRxBytes) * 8.0 / timeDiff / 1e6;
                txRateMbps = static_cast<double>(dTxBytes) * 8.0 / timeDiff / 1e6;

                if (dRxPackets > 0)
                {
                    delayNow = dDelaySum / static_cast<double>(dRxPackets);
                }
                if (dRxPackets > 1)
                {
                    jitterNow = dJitterSum / static_cast<double>(dRxPackets - 1);
                }
            }
            else
            {
                base.initialized = true;
            }

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(4)
                << currentTime.GetSeconds()
                << "s: UE[" << ueId
                << "] round=" << roundNumber
                << ", selected=" << (agg.selected ? "true" : "false")
                << ", isFinished=" << (agg.isFinished ? "true" : "false")
                << ", txRate=" << txRateMbps
                << " Mbps, rxRate=" << rxRateMbps
                << " Mbps, delayNow=" << delayNow
                << " s, jitterNow=" << jitterNow
                << " s, txPackets=" << agg.txPackets
                << ", rxPackets=" << agg.rxPackets
                << ", dLostPkts=0, lostTotal=0"
                << ", dRlcLostPkts=0, rlcLostTotal=0"
                << ", txBytes=" << agg.txBytes
                << ", rxBytes=" << agg.rxBytes
                << ", RSRP=" << client_info[ueId].rsrp
                << " dBm, SINR=" << client_info[ueId].sinr
                << " dB, pos=(" << client_info[ueId].x_pos
                << "," << client_info[ueId].y_pos
                << "), velocity=" << client_info[ueId].velocity
                << " m/s";
            NS_LOG_INFO(oss.str());

            base.lastRxBytes = agg.rxBytes;
            base.lastTxBytes = agg.txBytes;
            base.lastRxPackets = agg.rxPackets;
            base.lastDelaySum = agg.delaySum;
            base.lastJitterSum = agg.jitterSum;
            base.isFinished = agg.isFinished;
        }
    }

    lastTotalRxBytes = currentTotalRxBytes;
    lastTotalTxBytes = currentTotalTxBytes;
    lastTime = currentTime;
}


void ReportUeSinrRsrp(uint16_t cellId, uint16_t rnti, double rsrp, double sinr, uint8_t componentCarrierId) {
    CrossLayerMetricsCollector::reportUeSinrRsrp(cellId, rnti, rsrp, sinr, componentCarrierId);
}


void ReportUeSinrRsrp(std::string context, uint16_t cellId, uint16_t rnti, double rsrp, double sinr, uint8_t componentCarrierId) {
    CrossLayerMetricsCollector::reportUeSinrRsrp(context, cellId, rnti, rsrp, sinr, componentCarrierId);
}
