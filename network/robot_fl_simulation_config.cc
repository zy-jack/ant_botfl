#include "robot_fl_simulation_config.h"

namespace robot_fl_config
{

const char*
GetExperimentName(ExperimentPreset preset)
{
    switch (preset)
    {
    case ExperimentPreset::FREE_SPACE:
        return "FREE_SPACE";
    case ExperimentPreset::LOG_DISTANCE_25:
        return "LOG_DISTANCE_25";
    case ExperimentPreset::LOG_DISTANCE_30:
        return "LOG_DISTANCE_30";
    case ExperimentPreset::LOG_DISTANCE_35:
        return "LOG_DISTANCE_35";
    default:
        return "UNKNOWN";
    }
}

bool
IsFreeSpacePreset(ExperimentPreset preset)
{
    return preset == ExperimentPreset::FREE_SPACE;
}

double
GetLogDistanceExponent(ExperimentPreset preset)
{
    switch (preset)
    {
    case ExperimentPreset::LOG_DISTANCE_25:
        return 2.5;
    case ExperimentPreset::LOG_DISTANCE_30:
        return 3.0;
    case ExperimentPreset::LOG_DISTANCE_35:
        return 3.5;
    case ExperimentPreset::FREE_SPACE:
    default:
        return 2.0;
    }
}

namespace network
{

const double kSimStopTime = 120.0;
const int kNumberOfUes = 10;
const int kNumberOfEnbs = 1;
const int kScenarioX = 400;
const int kScenarioY = 400;
bool useStaticClients = false;

const ExperimentPreset kExperimentPreset = ExperimentPreset::LOG_DISTANCE_30;

const std::string kSpectrumChannelType = "ns3::MultiModelSpectrumChannel";
const std::string kSchedulerType = "ns3::RrFfMacScheduler";
const std::string kFfrAlgorithmType = "ns3::LteFrNoOpAlgorithm";
const std::string kFriisPathlossModel = "ns3::FriisPropagationLossModel";
const std::string kLogDistancePathlossModel = "ns3::LogDistancePropagationLossModel";
const double kFriisMinLossDb = 0.0;
const double kCarrierFrequencyHz = 1747.5e6;
const double kLogDistanceReferenceLossDb = 37.30;
const double kLogDistanceReferenceDistanceM = 1.0;

const uint16_t kDlEarfcn = 1575;
const uint16_t kUlEarfcn = 19575;
const uint8_t kBandwidthRb = 100;
const double kEnbTxPowerDbm = 46.0;
const double kUeTxPowerDbm = 23.0;
const double kEnbNoiseFigureDb = 5.0;
const double kUeNoiseFigureDb = 7.0;

const double kRobotSpeedMin = 3.0;
const double kRobotSpeedMax = 3.0;
const ns3::Time kRandomWalkTime = ns3::Seconds(5.0);
const double kPatrolMinX = 0.0;
const double kPatrolMaxX = kScenarioX;
const double kPatrolMinY = 0.0;
const double kPatrolMaxY = kScenarioY;

const std::vector<ns3::Vector> kEnbPositions = {
    ns3::Vector(0.0, 0.0, 25.0),
};

const bool kUeInitRandom = false;
const std::vector<ns3::Vector> kUeManualPositions = {
    ns3::Vector(90.0, 100.0, 1.5),
    ns3::Vector(115.0, 110.0, 1.5),
    ns3::Vector(135.0, 120.0, 1.5),
    ns3::Vector(150.0, 130.0, 1.5),
    ns3::Vector(165.0, 140.0, 1.5),
    ns3::Vector(180.0, 145.0, 1.5),
    ns3::Vector(190.0, 155.0, 1.5),
    ns3::Vector(205.0, 160.0, 1.5),
    ns3::Vector(215.0, 165.0, 1.5),
    ns3::Vector(225.0, 175.0, 1.5),
};

const uint32_t kRadioRlcUmMaxTxBufferSize = 512u * 1024u;
const bool kUseIdealRrc = true;
const bool kEnableUplinkPowerControl = false;
const bool kHarqEnabled = true;
const uint8_t kRsrpFilterCoefficient = 2;
const uint8_t kRsrqFilterCoefficient = 2;
const uint8_t kDefaultTransmissionMode = 2;

const std::string kPgwRemoteHostDataRate = "10Gb/s";
const uint32_t kPgwRemoteHostMtu = 1500;
const ns3::Time kPgwRemoteHostDelay = ns3::MicroSeconds(1);
const std::string kInternetIpBase = "1.0.0.0";
const std::string kInternetIpMask = "255.0.0.0";
const std::string kUeNetworkRoute = "7.0.0.0";
const std::string kUeNetworkMask = "255.0.0.0";

const AppTransportProtocol kAppTransportProtocol = AppTransportProtocol::UDP;
const std::string kAppUploadDataRate = "6Mbps";
const uint32_t kAppPacketSizeBytes = 1400u;
const uint32_t kTcpSendBufferBytes = 256u * 1024u;
const uint16_t kFirstUploadPort = 5000;
const ns3::Time kSinkStartDelay = ns3::MilliSeconds(200);
const ns3::Time kSourceStartDelay = kSinkStartDelay + ns3::MilliSeconds(10);
const bool kEnableSeqTsSizeHeader = true;
const std::string kOnTimeRandomVariable = "ns3::ConstantRandomVariable[Constant=1]";
const std::string kOffTimeRandomVariable = "ns3::ConstantRandomVariable[Constant=0]";

}

namespace fl
{

const std::string kAlgorithm = "fedavg";
const int kDefaultTrainingTime = 80;
const uint32_t kModelSizeBytes = 10u * 1024u * 1024u;
const int kApiNumClients = 10;
const int kApiClientsPerRound = 10;
const ns3::Time kUploadDeadline = ns3::Seconds(25.0);
const ns3::Time kInterRoundPause = ns3::Seconds(5.0);
const ns3::Time kManagerInterval = ns3::Seconds(1.0);
const ns3::Time kInitialManagerDelay = ns3::Seconds(2.0);
const ns3::Time kNetworkInfoInterval = ns3::Seconds(1.0);
const double kPlaceholderAccuracy = 0.1;

const std::string kRosSetupPath = "/home/yan/Project/Code/FL/FedAvg/rosfl_code/install/setup.bash";
const std::string kRosService = "/ros_server/execute_action rosfl_interfaces/srv/CmdAction";
const std::string kModeControl = "CONTROLL";
const std::string kModePing = "PING";

const bool kEnableLinkQualityFilter = false;
const double kMinSelectionSinrDb = 0.001;
const double kMinSelectionRsrpDbm = -120.0;

}

namespace visualization
{

const bool kEnableNetAnimExport = true;

const NetAnimStyleConfig kNetAnim = {
    "network-setup-netanim.xml",
    true,
    "/home/yan/tools/NS3/ns3_LTE/ns3_lte/images/devices/senseV2.png",
    0.0,
    0.0,
    0.5,
    0.5,
    0.30,
    "/home/yan/tools/NS3/ns3_LTE/ns3_lte/images/devices/antennatower_vl.png",
    "/home/yan/tools/NS3/ns3_LTE/ns3_lte/images/devices/robot.png",
    "/home/yan/tools/NS3/ns3_LTE/ns3_lte/images/devices/server_vl.png",
    "/home/yan/tools/NS3/ns3_LTE/ns3_lte/images/devices/lan-bus_vl.png",
    true,
    false,
    "Edge Server",
    "robot",
    "robot",
    "RAN Node",
    "core",
    {"Data GW", "Core GW", "Control"},
    ns3::Vector(0.0, 700.0, 0.0),
    ns3::Vector(-160.0, 520.0, 0.0),
    80.0,
    50000,
    ns3::MilliSeconds(100),
    false,
    50.0,
    50.0,
    100.0,
    100.0,
    40.0,
    40.0,
    40.0,
    40.0,
    {180, 0, 180},
    {255, 0, 0},
    {0, 0, 255},
    {255, 80, 80},
    {255, 165, 0},
    {0, 170, 0},
    {255, 140, 0},
};

}

namespace fading
{

const double kReferenceLossDb = 29.34;
const double kReferenceDistanceM = 1.0;
const double kExponent = 2.6;
const char* const kTraceRootDir = "./matlab/perue_fading_bank";
const char* const kFadingGroup = "no_fading";

}

}
