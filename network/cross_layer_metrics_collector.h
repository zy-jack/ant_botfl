#pragma once

#include "robot_fl_state_types.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/seq-ts-size-header.h"
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace ns3;

extern std::map<uint16_t, std::map<uint16_t, double>> sinrUe;
extern std::map<uint16_t, std::map<uint16_t, double>> rsrpUe;


struct UeNetBaseline
{
    uint64_t lastRxBytes   = 0;
    uint64_t lastTxBytes   = 0;
    uint32_t lastRxPackets = 0;
    double   lastDelaySum  = 0.0;
    double   lastJitterSum = 0.0;
    bool     initialized   = false;
    bool isFinished = false;
};


struct UeAggStats
{
    uint64_t rxBytes   = 0;
    uint64_t txBytes   = 0;
    uint32_t rxPackets = 0;
    uint32_t txPackets = 0;
    double   delaySum  = 0.0;
    double   jitterSum = 0.0;
    bool isFinished = false;
    bool selected = false;
};


struct FlowTraceStats
{
    uint32_t ueId      = 0;
    uint64_t rxBytes    = 0;
    uint64_t txBytes    = 0;
    uint32_t rxPackets  = 0;
    uint32_t txPackets  = 0;
    double   delaySum   = 0.0;
    double   jitterSum  = 0.0;
    double   lastDelay  = 0.0;
    bool     hasLastDelay = false;
};


extern std::map<uint32_t, UeNetBaseline> g_ueNetBaseline;
extern std::map<std::string, FlowTraceStats> g_flowTraceStats;
extern std::set<std::string> g_activeTransportFlows;
extern std::set<std::string> g_stoppedTransportFlows;


class CrossLayerMetricsCollector {
public:
    static std::pair<double, double> getRsrpSinr(uint32_t nodeIdx);
    static void updateAllClientsGlobalInfo(int trainingTime, int modelSizeBytes);
    static void networkInfo();
    static void setupRsrpSinrTracing();
    static double Safe10Log10(double x);
    static double RsrpToDbm(double rsrp);
    static double SinrToDb(double sinr);
    static void traceAppTx(std::string transportKey,
                           Ptr<const Packet> packet,
                           const Address& from,
                           const Address& to,
                           const SeqTsSizeHeader& header);
    static void traceAppRx(Ptr<const Packet> packet,
                           const Address& from,
                           const Address& to,
                           const SeqTsSizeHeader& header);
    static void resetRealtimeRoundState();

    static void reportUeSinrRsrp(uint16_t cellId, uint16_t rnti, double rsrp, double sinr, uint8_t componentCarrierId);
    static void reportUeSinrRsrp(std::string context, uint16_t cellId, uint16_t rnti, double rsrp, double sinr, uint8_t componentCarrierId);

private:
};

void ReportUeSinrRsrp(uint16_t cellId, uint16_t rnti, double rsrp, double sinr, uint8_t componentCarrierId);
void ReportUeSinrRsrp(std::string context, uint16_t cellId, uint16_t rnti, double rsrp, double sinr, uint8_t componentCarrierId);
