#include "robot_mobility_fading_model.h"
#include "robot_fl_simulation_config.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/string.h"

#include <cmath>
#include <fstream>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("RobotMobilityFadingModel");
NS_OBJECT_ENSURE_REGISTERED(RobotMobilityFadingModel);

TypeId
RobotMobilityFadingModel::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::RobotMobilityFadingModel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("Propagation")
            .AddConstructor<RobotMobilityFadingModel>()
	            .AddAttribute("ReferenceLoss",
	                          "Pathloss at reference distance in dB.",
	                          DoubleValue(::robot_fl_config::fading::kReferenceLossDb),
	                          MakeDoubleAccessor(&RobotMobilityFadingModel::m_referenceLossDb),
	                          MakeDoubleChecker<double>())
	            .AddAttribute("ReferenceDistance",
	                          "Reference distance in meters.",
	                          DoubleValue(::robot_fl_config::fading::kReferenceDistanceM),
	                          MakeDoubleAccessor(&RobotMobilityFadingModel::m_referenceDistanceM),
	                          MakeDoubleChecker<double>())
	            .AddAttribute("Exponent",
	                          "Log-distance pathloss exponent.",
	                          DoubleValue(::robot_fl_config::fading::kExponent),
	                          MakeDoubleAccessor(&RobotMobilityFadingModel::m_exponent),
	                          MakeDoubleChecker<double>())
	            .AddAttribute("TraceRootDir",
	                          "Root directory of per-robot fading CSV bank.",
	                          StringValue(::robot_fl_config::fading::kTraceRootDir),
	                          MakeStringAccessor(&RobotMobilityFadingModel::m_traceRootDir),
	                          MakeStringChecker())
	            .AddAttribute("FadingGroup",
	                          "Selected fading group: no_fading/rayleigh/k0/k4/k12.",
	                          StringValue(::robot_fl_config::fading::kFadingGroup),
	                          MakeStringAccessor(&RobotMobilityFadingModel::m_fadingGroup),
	                          MakeStringChecker());
    return tid;
}

RobotMobilityFadingModel::RobotMobilityFadingModel()
    : m_referenceLossDb(::robot_fl_config::fading::kReferenceLossDb),
      m_referenceDistanceM(::robot_fl_config::fading::kReferenceDistanceM),
      m_exponent(::robot_fl_config::fading::kExponent),
      m_traceRootDir(::robot_fl_config::fading::kTraceRootDir),
      m_fadingGroup(::robot_fl_config::fading::kFadingGroup)
{
}

RobotMobilityFadingModel::~RobotMobilityFadingModel() = default;

double
RobotMobilityFadingModel::DoCalcRxPower(double txPowerDbm,
                                                 Ptr<MobilityModel> a,
                                                 Ptr<MobilityModel> b) const
{
    const double lossDb = CalcLogDistanceLossDb(a, b);
    const double fadingGainDb = LookupFadingGainDbForLink(a, b);


    return txPowerDbm - lossDb + fadingGainDb;
}

int64_t
RobotMobilityFadingModel::DoAssignStreams(int64_t stream)
{
    return 0;
}

double
RobotMobilityFadingModel::CalcLogDistanceLossDb(Ptr<MobilityModel> a,
                                                         Ptr<MobilityModel> b) const
{
    const double d = a->GetDistanceFrom(b);
    if (d <= m_referenceDistanceM)
    {
        return m_referenceLossDb;
    }
    return m_referenceLossDb + 10.0 * m_exponent * std::log10(d / m_referenceDistanceM);
}

bool
RobotMobilityFadingModel::TryGetNodeId(Ptr<MobilityModel> mob, uint32_t& nodeId) const
{
    if (!mob)
    {
        return false;
    }

    Ptr<Node> node = mob->GetObject<Node>();
    if (!node)
    {
        return false;
    }

    nodeId = node->GetId();
    return true;
}

double
RobotMobilityFadingModel::LookupFadingGainDbForLink(Ptr<MobilityModel> a,
                                                             Ptr<MobilityModel> b) const
{
    if (m_fadingGroup == "no_fading")
    {
        return 0.0;
    }

    const double nowSec = Simulator::Now().GetSeconds();

    uint32_t nodeA = 0;
    uint32_t nodeB = 0;

    const bool hasA = TryGetNodeId(a, nodeA);
    const bool hasB = TryGetNodeId(b, nodeB);

    if (hasA)
    {
        const double gA = LookupFadingGainDbForNode(nodeA, nowSec);
        if (!std::isnan(gA))
        {
            return gA;
        }
    }

    if (hasB)
    {
        const double gB = LookupFadingGainDbForNode(nodeB, nowSec);
        if (!std::isnan(gB))
        {
            return gB;
        }
    }

    return 0.0;
}

double
RobotMobilityFadingModel::LookupFadingGainDbForNode(uint32_t nodeId, double nowSec) const
{
    EnsureSeriesLoaded(nodeId);

    auto it = m_seriesCache.find(nodeId);
    if (it == m_seriesCache.end() || !it->second.exists)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return InterpolateGainDb(it->second, nowSec);
}

std::string
RobotMobilityFadingModel::BuildCsvPath(uint32_t nodeId) const
{
    std::ostringstream oss;
    oss << m_traceRootDir << "/" << m_fadingGroup << "/node" << nodeId << ".csv";
    return oss.str();
}

void
RobotMobilityFadingModel::EnsureSeriesLoaded(uint32_t nodeId) const
{
    auto it = m_seriesCache.find(nodeId);
    if (it != m_seriesCache.end() && it->second.loaded)
    {
        return;
    }

    TimeSeries ts;
    ts.loaded = true;
    ts.exists = false;

    const std::string filePath = BuildCsvPath(nodeId);
    std::ifstream fin(filePath.c_str());
    if (!fin.is_open())
    {
        m_seriesCache[nodeId] = ts;
        return;
    }

    std::string line;
    bool firstLine = true;
    while (std::getline(fin, line))
    {
        if (line.empty())
        {
            continue;
        }

        if (firstLine)
        {
            firstLine = false;
            if (line.find("time") != std::string::npos)
            {
                continue;
            }
        }

        std::stringstream ss(line);
        std::string token1, token2;
        if (!std::getline(ss, token1, ','))
        {
            continue;
        }
        if (!std::getline(ss, token2, ','))
        {
            continue;
        }

        try
        {
            const double t = std::stod(token1);
            const double g = std::stod(token2);
            ts.timeSec.push_back(t);
            ts.gainDb.push_back(g);
        }
        catch (...)
        {
            continue;
        }
    }

    ts.exists = !ts.timeSec.empty();
    m_seriesCache[nodeId] = ts;

    if (ts.exists)
    {
        NS_LOG_INFO("Loaded fading CSV for nodeId=" << nodeId
                                                    << " group=" << m_fadingGroup
                                                    << " samples=" << ts.timeSec.size());
    }
}

double
RobotMobilityFadingModel::InterpolateGainDb(const TimeSeries& ts, double nowSec) const
{
    if (ts.timeSec.empty())
    {
        return 0.0;
    }

    if (nowSec <= ts.timeSec.front())
    {
        return ts.gainDb.front();
    }
    if (nowSec >= ts.timeSec.back())
    {
        return ts.gainDb.back();
    }

    auto upper = std::upper_bound(ts.timeSec.begin(), ts.timeSec.end(), nowSec);
    const size_t idx1 = std::distance(ts.timeSec.begin(), upper);
    const size_t idx0 = idx1 - 1;

    const double t0 = ts.timeSec[idx0];
    const double t1 = ts.timeSec[idx1];
    const double g0 = ts.gainDb[idx0];
    const double g1 = ts.gainDb[idx1];

    if (t1 <= t0)
    {
        return g0;
    }

    const double alpha = (nowSec - t0) / (t1 - t0);
    return g0 + alpha * (g1 - g0);
}

}
