#pragma once

#include "ns3/core-module.h"

#include <cstdint>
#include <string>
#include <vector>

namespace robot_fl_config
{

enum class AppTransportProtocol
{
    UDP,
    TCP
};

enum class ExperimentPreset
{
    FREE_SPACE,
    LOG_DISTANCE_25,
    LOG_DISTANCE_30,
    LOG_DISTANCE_35
};

const char* GetExperimentName(ExperimentPreset preset);
bool IsFreeSpacePreset(ExperimentPreset preset);
double GetLogDistanceExponent(ExperimentPreset preset);

namespace network
{

extern const double kSimStopTime;
extern const int kNumberOfUes;
extern const int kNumberOfEnbs;
extern const int kScenarioX;
extern const int kScenarioY;
extern bool useStaticClients;

extern const ExperimentPreset kExperimentPreset;

extern const std::string kSpectrumChannelType;
extern const std::string kSchedulerType;
extern const std::string kFfrAlgorithmType;
extern const std::string kFriisPathlossModel;
extern const std::string kLogDistancePathlossModel;
extern const double kFriisMinLossDb;
extern const double kCarrierFrequencyHz;
extern const double kLogDistanceReferenceLossDb;
extern const double kLogDistanceReferenceDistanceM;

extern const uint16_t kDlEarfcn;
extern const uint16_t kUlEarfcn;
extern const uint8_t kBandwidthRb;
extern const double kEnbTxPowerDbm;
extern const double kUeTxPowerDbm;
extern const double kEnbNoiseFigureDb;
extern const double kUeNoiseFigureDb;

extern const double kRobotSpeedMin;
extern const double kRobotSpeedMax;
extern const ns3::Time kRandomWalkTime;
extern const double kPatrolMinX;
extern const double kPatrolMaxX;
extern const double kPatrolMinY;
extern const double kPatrolMaxY;

extern const std::vector<ns3::Vector> kEnbPositions;
extern const bool kUeInitRandom;
extern const std::vector<ns3::Vector> kUeManualPositions;

extern const uint32_t kRadioRlcUmMaxTxBufferSize;
extern const bool kUseIdealRrc;
extern const bool kEnableUplinkPowerControl;
extern const bool kHarqEnabled;
extern const uint8_t kRsrpFilterCoefficient;
extern const uint8_t kRsrqFilterCoefficient;
extern const uint8_t kDefaultTransmissionMode;

extern const std::string kPgwRemoteHostDataRate;
extern const uint32_t kPgwRemoteHostMtu;
extern const ns3::Time kPgwRemoteHostDelay;
extern const std::string kInternetIpBase;
extern const std::string kInternetIpMask;
extern const std::string kUeNetworkRoute;
extern const std::string kUeNetworkMask;

extern const AppTransportProtocol kAppTransportProtocol;
extern const std::string kAppUploadDataRate;
extern const uint32_t kAppPacketSizeBytes;
extern const uint32_t kTcpSendBufferBytes;
extern const uint16_t kFirstUploadPort;
extern const ns3::Time kSinkStartDelay;
extern const ns3::Time kSourceStartDelay;
extern const bool kEnableSeqTsSizeHeader;
extern const std::string kOnTimeRandomVariable;
extern const std::string kOffTimeRandomVariable;

}

namespace fl
{

extern const std::string kAlgorithm;
extern const int kDefaultTrainingTime;
extern const uint32_t kModelSizeBytes;
extern const int kApiNumClients;
extern const int kApiClientsPerRound;
extern const ns3::Time kUploadDeadline;
extern const ns3::Time kInterRoundPause;
extern const ns3::Time kManagerInterval;
extern const ns3::Time kInitialManagerDelay;
extern const ns3::Time kNetworkInfoInterval;
extern const double kPlaceholderAccuracy;

extern const std::string kRosSetupPath;
extern const std::string kRosService;
extern const std::string kModeControl;
extern const std::string kModePing;

extern const bool kEnableLinkQualityFilter;
extern const double kMinSelectionSinrDb;
extern const double kMinSelectionRsrpDbm;

}

namespace visualization
{

struct RgbColor
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct NetAnimStyleConfig
{
    std::string xmlFile;
    bool enableBackgroundImage;
    std::string backgroundImage;
    double backgroundX;
    double backgroundY;
    double backgroundScaleX;
    double backgroundScaleY;
    double backgroundOpacity;

    std::string enbIcon;
    std::string ueIcon;
    std::string serverIcon;
    std::string coreNodeIcon;

    bool highlightFirstUe;
    bool showAllUeLabels;
    std::string remoteHostLabel;
    std::string firstUeLabel;
    std::string otherUeLabelPrefix;
    std::string enbLabelPrefix;
    std::string coreNodeLabelPrefix;
    std::vector<std::string> coreNodeLabels;

    ns3::Vector remoteHostPosition;
    ns3::Vector autoPlacedCoreStart;
    double autoPlacedCoreStepY;

    uint32_t maxPacketsPerTraceFile;
    ns3::Time mobilityPollInterval;
    bool enablePacketAnimation;

    double remoteHostSizeX;
    double remoteHostSizeY;
    double enbSizeX;
    double enbSizeY;
    double ueSizeX;
    double ueSizeY;
    double coreNodeSizeX;
    double coreNodeSizeY;

    RgbColor remoteHostColor;
    RgbColor firstEnbColor;
    RgbColor secondEnbColor;
    RgbColor otherEnbColor;
    RgbColor firstUeColor;
    RgbColor otherUeColor;
    RgbColor coreNodeColor;
};

extern const bool kEnableNetAnimExport;
extern const NetAnimStyleConfig kNetAnim;

}

namespace fading
{

extern const double kReferenceLossDb;
extern const double kReferenceDistanceM;
extern const double kExponent;
extern const char* const kTraceRootDir;
extern const char* const kFadingGroup;

}

}
