#ifndef ROBOT_MOBILITY_FADING_MODEL_H
#define ROBOT_MOBILITY_FADING_MODEL_H

#include "ns3/propagation-loss-model.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

#include <map>
#include <string>
#include <vector>

namespace ns3
{

class MobilityModel;

class RobotMobilityFadingModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId(void);
    RobotMobilityFadingModel();
    ~RobotMobilityFadingModel() override;

  private:
    struct TimeSeries
    {
        std::vector<double> timeSec;
        std::vector<double> gainDb;
        bool loaded = false;
        bool exists = false;
    };

    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;

    int64_t DoAssignStreams(int64_t stream) override;

    double CalcLogDistanceLossDb(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const;
    bool TryGetNodeId(Ptr<MobilityModel> mob, uint32_t& nodeId) const;
    double LookupFadingGainDbForLink(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const;
    double LookupFadingGainDbForNode(uint32_t nodeId, double nowSec) const;
    std::string BuildCsvPath(uint32_t nodeId) const;
    void EnsureSeriesLoaded(uint32_t nodeId) const;
    double InterpolateGainDb(const TimeSeries& ts, double nowSec) const;

  private:
    double m_referenceLossDb;
    double m_referenceDistanceM;
    double m_exponent;

    std::string m_traceRootDir;
    std::string m_fadingGroup;

    mutable std::map<uint32_t, TimeSeries> m_seriesCache;
};

}

#endif
