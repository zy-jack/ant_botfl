#include "ros_fl_round_coordinator.h"
#include "cross_layer_metrics_collector.h"
#include "wireless_scenario_builder.h"
#include "wireless_transport_manager.h"
#include "ns3/command-line.h"
#include "ns3/log.h"
#include <cstring>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("RobotFlSimulation");

int main(int argc, char *argv[]) {
  namespace net_cfg = robot_fl_config::network;
  namespace fl_cfg = robot_fl_config::fl;


  LogComponentEnable("RobotFlSimulation", LOG_LEVEL_INFO);
  LogComponentEnable("RosFlRoundCoordinator", LOG_LEVEL_INFO);
  LogComponentEnable("WirelessScenarioBuilder", LOG_LEVEL_INFO);
  LogComponentEnable("CrossLayerMetricsCollector", LOG_LEVEL_INFO);
  LogComponentEnable("WirelessTransportManager", LOG_LEVEL_INFO);
  LogComponentEnable("RobotFlStateTypes", LOG_LEVEL_INFO);


  NS_LOG_INFO("Starting FL-NS3 Simulation");
  WirelessScenarioBuilder::configureDefaults();


  std::string algorithm = fl_cfg::kAlgorithm;
  CommandLine cmd;
  cmd.AddValue("algorithm", "FL algorithm (ns-3 perspective, less relevant now)", algorithm);
  cmd.Parse(argc, argv);


  if (!RosFlRoundCoordinator::initializeROSFL()) {
    NS_LOG_ERROR("Failed to initialize FL API. Exiting.");
    return 1;
  }


  Ptr<LteHelper> wirelessHelper;
  Ptr<PointToPointEpcHelper> epcHelper;
  WirelessScenarioBuilder::setupNodes(net_cfg::kNumberOfEnbs, net_cfg::kNumberOfUes);
  WirelessScenarioBuilder::setupCoreNetwork(wirelessHelper, epcHelper);
  WirelessScenarioBuilder::setupMobility(net_cfg::useStaticClients,
                              net_cfg::kScenarioX,
                              net_cfg::kScenarioY,
                              epcHelper);
  WirelessScenarioBuilder::setupDevicesAndIp(wirelessHelper, epcHelper);
  WirelessScenarioBuilder::setupRouting(epcHelper);
  CrossLayerMetricsCollector::setupRsrpSinrTracing();

  Simulator::Schedule(fl_cfg::kInitialManagerDelay, &RosFlRoundCoordinator::manager);
  Simulator::Schedule(fl_cfg::kNetworkInfoInterval, &CrossLayerMetricsCollector::networkInfo);
  NS_LOG_INFO("Manager and networkInfo functions scheduled.");

  WirelessScenarioBuilder::setupAnimation();
  Simulator::Stop(Seconds(net_cfg::kSimStopTime));
  NS_LOG_INFO("Starting ns-3 Simulation. Simulation will stop at " << net_cfg::kSimStopTime << "s.");
  Simulator::Run();
  NS_LOG_INFO("ns-3 Simulation Finished.");
  Simulator::Destroy();
  NS_LOG_INFO("Simulator Destroyed.");
  return 0;
}
