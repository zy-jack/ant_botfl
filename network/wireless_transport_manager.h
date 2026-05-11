#pragma once

#include "robot_fl_state_types.h"


#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/seq-ts-size-header.h"

#include <map>
#include <string>
#include <vector>

using namespace ns3;


extern std::map<std::string, double> endOfStreamTimes;
extern std::map<std::string, bool> selectedUeFinshed;
extern std::map<std::string, std::string> transportInfo;

struct UeUploadRoundStat {
    uint32_t ueId;
    std::string ueIp;
    int uePort;
    bool selected;
    bool finished;
    uint64_t targetBytes;
    uint64_t uploadedBytes;
    double completionRatio;
    double finishTime;
};


class WirelessTransportManager {
public:

    static std::pair<uint16_t, uint16_t> getUeRntiCellid(Ptr<ns3::NetDevice> ueNetDevice);


    static std::vector<NodesIps> nodeToIps();


    static void sendStream(Ptr<Node> sendingNode, Ptr<Node> receivingNode, int size);


    static void sinkRxCallback(Ptr<const Packet> packet,
                               const Address& from,
                               const Address& to,
                               const SeqTsSizeHeader& header);


    static void stopRoundApplicationsNow();


    static std::vector<UeUploadRoundStat> collectRoundUploadStats(
        const std::vector<ClientModels>& selected_clients_for_round);


    static bool checkFinishedTransmission(const std::vector<NodesIps>& all_nodes_ips,
                                          const std::vector<ClientModels>& selected_clients_for_round);


    static void roundCleanup();
};


