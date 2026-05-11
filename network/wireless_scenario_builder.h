#pragma once

#include "robot_fl_simulation_config.h"

#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-helper.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/point-to-point-helper.h"
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>


using namespace ns3;

using AppTransportProtocol = robot_fl_config::AppTransportProtocol;


struct ClientInfo {
    int serving_enb;
    double x_pos;
    double y_pos;
    double velocity;
    double rsrp;
    double sinr;
};

struct EnbInfo {
    double x_pos;
    double y_pos;
};


extern std::map<int, ClientInfo> client_info;
extern std::map<int, EnbInfo> enb_info;

extern std::set<std::string> ueIpSet;
extern std::map<std::string, uint32_t> ueIpToIndex;
extern std::map<uint32_t, std::string> ueIndexToIp;


extern Ptr<LteHelper> globalWirelessHelper;
extern NodeContainer ueNodes;
extern NodeContainer enbNodes;
extern NodeContainer remoteHostContainer;
extern NetDeviceContainer enbDevs;
extern NetDeviceContainer ueDevs;
extern Ipv4Address remoteHostAddr;

class WirelessScenarioBuilder {
public:
    static void configureDefaults();
    static void setupCoreNetwork(Ptr<LteHelper> &wirelessHelper, Ptr<PointToPointEpcHelper> &epcHelper);

    static void setupNodes(int numberOfEnbs, int numberOfUes);
    static void setupMobility(bool useStaticClients, int scenarioX, int scenarioY, Ptr<PointToPointEpcHelper> epcHelper);

    static void setupDevicesAndIp(Ptr<LteHelper> wirelessHelper, Ptr<PointToPointEpcHelper> epcHelper);

    static void setupRouting(Ptr<PointToPointEpcHelper> epcHelper);
    static void setupAnimation();
private:
    static void setupPgwRemoteHostConnection(Ptr<PointToPointEpcHelper> epcHelper);

};
