#include "ros_fl_round_coordinator.h"
#include "cross_layer_metrics_collector.h"
#include "wireless_transport_manager.h"
#include "wireless_scenario_builder.h"
#include "ns3/log.h"
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <unistd.h>
#include <cerrno>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <vector>
#include <cctype>
#include <regex>

NS_LOG_COMPONENT_DEFINE("RosFlRoundCoordinator");

namespace fl_cfg = robot_fl_config::fl;

static bool roundFinished = true;
int roundNumber = 0;
static bool fl_api_initialized = false;
static Time nextRoundEligibleTime = Seconds(0.0);


std::vector<NodesIps> nodesIPs;
std::vector<ClientModels> clientsInfoGlobal;
std::vector<ClientModels> selectedClientsForCurrentRound;

std::string RosFlRoundCoordinator::getEnvVar(const std::string &key, const std::string &default_val) {
    const char* val = std::getenv(key.c_str());
    if (val == nullptr) {
        NS_LOG_INFO("Environment variable '" << key << "' not found. Using default value: '" << default_val << "'.");
        return default_val;
    }
    NS_LOG_INFO("Found environment variable '" << key << "'. Value: '" << val << "'.");
    return std::string(val);
}

std::string RosFlRoundCoordinator::toUploadCompletionJsonPayload(const std::map<int, double>& data) {
    std::ostringstream payload;
    payload << "{";

    size_t i = 0;
    for (const auto& item : data) {
        if (i) {
            payload << ", ";
        }

        double ratio = std::clamp(item.second, 0.0, 1.0);
        payload << "\"" << item.first << "\": "
                << std::fixed << std::setprecision(6) << ratio;
        ++i;
    }

    payload << "}";
    return payload.str();
}

std::string RosFlRoundCoordinator::yamlSingleQuote(const std::string& value) {
    std::string out;
    out.reserve(value.size() + 2);
    out.push_back('\'');
    for (char c : value) {
        if (c == '\'') {
            out += "''";
        } else {
            out.push_back(c);
        }
    }
    out.push_back('\'');
    return out;
}

std::string RosFlRoundCoordinator::bashSingleQuote(const std::string& cmd) {
    std::string out;
    out.reserve(cmd.size() + 2);
    out.push_back('\'');
    for (char c : cmd) {
        if (c == '\'') out += "'\\''";
        else out.push_back(c);
    }
    out.push_back('\'');
    return out;
}

std::string RosFlRoundCoordinator::runCommand(const std::string &cmd, int &exit_status) {

    std::fflush(stdout);

    std::array<char, 4096> buffer{};
    std::string output;

    FILE *pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        throw std::runtime_error(std::string("popen failed: ") + std::strerror(errno));
    }

    try {
        while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
            output += buffer.data();
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }

    int status = pclose(pipe);
    exit_status = status;
    return output;
}

int RosFlRoundCoordinator::parseResponseExitCode(const std::string& out, bool &success_value) {

    std::regex re(R"(success\s*[:=]\s*(true|false))", std::regex::icase);
    std::smatch m;
    bool found_any = false;
    bool val = false;


    auto begin = out.cbegin();
    auto end   = out.cend();
    while (std::regex_search(begin, end, m, re)) {
        found_any = true;
        std::string token = m[1].str();
        for (auto &ch : token) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        val = (token == "true");
        begin = m.suffix().first;
    }

    if (!found_any) return 3;
    success_value = val;
    return val ? 0 : 2;
}

bool RosFlRoundCoordinator::callROSFL(const std::string &ros_mode,
                              const std::string &int16_param_payload,
                              const std::string &json_param_payload) {

  std::ostringstream command;
  command << "bash -lc ";

  std::ostringstream ros_call;

  if (access(fl_cfg::kRosSetupPath.c_str(), R_OK) != 0) {
    NS_LOG_ERROR("[WARN] Workspace setup.bash does not exist or is not readable: " << fl_cfg::kRosSetupPath << "\n");
  }

  ros_call << "source " << bashSingleQuote(fl_cfg::kRosSetupPath) << " && "
           << "ros2 service call " << fl_cfg::kRosService << " ";

  std::ostringstream request;
  request << "{mode: " << yamlSingleQuote(ros_mode)
          << ", param: " << int16_param_payload
          << ", json_param: " << yamlSingleQuote(json_param_payload)
          << "}";
  ros_call << bashSingleQuote(request.str()) << " 2>&1";

  command << bashSingleQuote(ros_call.str());

  std::fflush(stdout);

  int pipe_status = 0;
  std::string out = runCommand(command.str(), pipe_status);
  std::fflush(stdout);

  if (pipe_status == -1) {
    NS_LOG_ERROR("[ERROR] pclose() failed: " << std::strerror(errno) << "\n");
    std::fflush(stdout);
    return false;
  }
  if (!WIFEXITED(pipe_status)) {
      if (WIFSIGNALED(pipe_status)) {
          int sig = WTERMSIG(pipe_status);
          NS_LOG_ERROR("[ERROR] pipe killed by signal " << sig << "\n");
          std::fflush(stdout);
          return 128 + sig;
      }
      NS_LOG_ERROR("[ERROR] Unknown pipe status: " << pipe_status << "\n");
      std::fflush(stdout);
      return false;
  }
  int code = WEXITSTATUS(pipe_status);
  if (code != 0) {
      NS_LOG_ERROR("[ERROR] ros2 CLI exited with code " << code << "\n");
      return false;
  }


  bool success_val = false;
  int rc = parseResponseExitCode(out, success_val);
  if (rc == 0) {
    NS_LOG_DEBUG("[RESULT] success=True\n");
  } else if (rc == 2) {
    NS_LOG_ERROR("[RESULT] success=False\n");
  } else {
    NS_LOG_ERROR("[RESULT] success field NOT FOUND\n");
  }
  std::fflush(stdout);
  return success_val;
}

bool RosFlRoundCoordinator::initializeROSFL() {

    NS_LOG_INFO("Check ROSFL State...");
    bool result = callROSFL(fl_cfg::kModePing, "[0]", "");
    fl_api_initialized = result;
    if (result) {
      NS_LOG_INFO("ROSFL Running...");
    }else{
      NS_LOG_WARN("ROSFL Excites Error...");
    }
    std::fflush(stdout);
    return result;
}

void RosFlRoundCoordinator::selectNs3ManagedClients(int n_to_select) {
  NS_LOG_INFO("Selecting " << n_to_select << " clients for FL round "
	                           << roundNumber << " based on ns-3 criteria.");
  selectedClientsForCurrentRound.clear();
  CrossLayerMetricsCollector::updateAllClientsGlobalInfo(fl_cfg::kDefaultTrainingTime,
                                               static_cast<int>(fl_cfg::kModelSizeBytes));

  std::vector<ClientModels> candidates = clientsInfoGlobal;
  std::sort(candidates.begin(), candidates.end(),
            [](const ClientModels &a, const ClientModels &b) {
              return a.sinr > b.sinr;
            });

  int actual_selected_count = 0;
  for (int i = 0; i < n_to_select && (long unsigned int)i < candidates.size(); ++i) {
    const bool passesLinkFilter =
        !fl_cfg::kEnableLinkQualityFilter ||
        candidates[i].sinr > fl_cfg::kMinSelectionSinrDb ||
        candidates[i].rsrp > fl_cfg::kMinSelectionRsrpDbm;
    if (passesLinkFilter) {
      ClientModels selected_client = candidates[i];
      selected_client.selected = true;
      selectedClientsForCurrentRound.push_back(selected_client);
      actual_selected_count++;
      NS_LOG_DEBUG("  Selected client " << selected_client.node->GetId()
                                        << " (SINR: " << selected_client.sinr
                                        << " dB, RSRP: " << selected_client.rsrp
                                        << " dBm)");
    } else {
      NS_LOG_DEBUG("  Skipping client "
                   << candidates[i].node->GetId() << " due to low SINR ("
                   << candidates[i].sinr << " dB) or RSRP ("
                   << candidates[i].rsrp << " dBm).");
    }
  }
  NS_LOG_INFO("ns-3 selected " << actual_selected_count << " clients (out of "
                               << n_to_select << " requested) for FL round "
                               << roundNumber);
  if (actual_selected_count == 0 && n_to_select > 0) {
    NS_LOG_WARN("No eligible clients were selected by ns-3 for this round, "
                "possibly due to poor network conditions for all UEs.");
  }
}

bool RosFlRoundCoordinator::triggerAndProcessFLRoundInApi() {
  NS_LOG_INFO("=================== Preparing FL Round "
              << roundNumber << " in Python API at "
              << Simulator::Now().GetSeconds() << "s ===================");
  std::fflush(stdout);

  if (selectedClientsForCurrentRound.empty() && fl_cfg::kApiClientsPerRound > 0) {
    NS_LOG_INFO("No clients were selected by ns-3 for this round. Skipping ns-3 comms.");
    std::fflush(stdout);
    if (fl_cfg::kApiClientsPerRound > 0)
      return false;
  }

  NS_LOG_INFO("Selected UE id list is no longer sent to ROS2 at round start. "
              "NS3 upload completion dictionary will be sent after this round's "
              "communication phase finishes.");
  return true;
}

void RosFlRoundCoordinator::sendModelsToServer() {
  NS_LOG_INFO("ns-3: Simulating model uploads for "
              << selectedClientsForCurrentRound.size() << " selected clients.");
  if (selectedClientsForCurrentRound.empty()) {
    NS_LOG_INFO("  No clients selected for model upload in ns-3 this round. Skipping send.");
    return;
  }

  for (const auto &client_model_info : selectedClientsForCurrentRound) {


    Simulator::Schedule(MilliSeconds(client_model_info.nodeTrainingTime),
                        &WirelessTransportManager::sendStream, client_model_info.node,
                        remoteHostContainer.Get(0),
                        client_model_info.nodeModelSize);
  }
}

bool RosFlRoundCoordinator::isRoundTimedOut(Time roundStartTimeNs3Comms) {


  bool timedOut = Simulator::Now() - roundStartTimeNs3Comms > fl_cfg::kUploadDeadline;
  if (timedOut) {
    NS_LOG_WARN("isRoundTimedOut: ns-3 Comms phase for round "
                << roundNumber << " has timed out at "
                << Simulator::Now().GetSeconds() << "s.");
  } else {
    NS_LOG_DEBUG("isRoundTimedOut: ns-3 Comms phase for round "
	                 << roundNumber << " is not yet timed out. Current duration: "
	                 << (Simulator::Now() - roundStartTimeNs3Comms).GetSeconds()
	                 << "s, deadline=" << fl_cfg::kUploadDeadline.GetSeconds() << "s.");
  }
  return timedOut;
}

void RosFlRoundCoordinator::logRoundTimeout() {
  NS_LOG_WARN("ns-3 Comms Round timed out for round "
              << roundNumber
              << ". Successful transfers: " << endOfStreamTimes.size() << "/"
              << selectedClientsForCurrentRound.size() << " clients.");
}

void RosFlRoundCoordinator::finalizeNs3CommsPhase() {
  NS_LOG_INFO("ns-3 Comms phase for round "
              << roundNumber << " finished at " << Simulator::Now().GetSeconds()
              << ". Successful transfers: " << endOfStreamTimes.size() << "/"
              << selectedClientsForCurrentRound.size());


  WirelessTransportManager::stopRoundApplicationsNow();

  std::map<int, double> uploadCompletionByUe;
  std::vector<UeUploadRoundStat> roundUploadStats =
      WirelessTransportManager::collectRoundUploadStats(selectedClientsForCurrentRound);
  for (const auto& stat : roundUploadStats) {
    if (stat.selected) {
      uploadCompletionByUe[static_cast<int>(stat.ueId)] = stat.completionRatio;
    }
  }

  std::string uploadCompletionPayload =
      toUploadCompletionJsonPayload(uploadCompletionByUe);
  NS_LOG_INFO("Sending ns-3 upload completion dictionary to ROS2 as JSON: "
              << uploadCompletionPayload);
  bool rosResult = callROSFL(fl_cfg::kModeControl, "[0]", uploadCompletionPayload);
  if (rosResult) {
    NS_LOG_INFO("ROS2 accepted ns-3 upload completion dictionary for round "
                << roundNumber << ".");
  } else {
    NS_LOG_WARN("ROS2 rejected or failed to process ns-3 upload completion "
                "dictionary for round " << roundNumber << ".");
  }

  WirelessTransportManager::roundCleanup();

  nextRoundEligibleTime = Simulator::Now() + fl_cfg::kInterRoundPause;
  NS_LOG_INFO("ns-3 Comms phase cleanup complete for round " << roundNumber
              << ". Next round can start after "
              << nextRoundEligibleTime.GetSeconds() << "s.");
}

void RosFlRoundCoordinator::startNewFLRound(Time &roundStartTimeNs3CommsParam) {
  roundNumber++;
  NS_LOG_INFO("StartNewFLRound: Beginning for FL Round "
              << roundNumber << " at " << Simulator::Now().GetSeconds()
              << "s.");

  selectNs3ManagedClients(fl_cfg::kApiClientsPerRound);
  NS_LOG_INFO("StartNewFLRound: ns-3 selected "
              << selectedClientsForCurrentRound.size()
              << " clients for this round.");

  if (selectedClientsForCurrentRound.empty() && fl_cfg::kApiClientsPerRound > 0) {
    NS_LOG_INFO("StartNewFLRound: No clients were selected by ns-3. Skipping API call and ns-3 comms for this round.");
    roundFinished = true;
    return;
  }

  NS_LOG_INFO("StartNewFLRound: Preparing ns-3 communication phase for round "
              << roundNumber);
  bool api_success = triggerAndProcessFLRoundInApi();

  if (api_success) {
    NS_LOG_INFO("StartNewFLRound: ns-3 round preparation successful for round "
                << roundNumber);
    if (!selectedClientsForCurrentRound.empty()) {
      NS_LOG_INFO("StartNewFLRound: Scheduling ns-3 model uploads for "
                  << selectedClientsForCurrentRound.size()
                  << " clients with updated times/sizes.");
      sendModelsToServer();
      roundStartTimeNs3CommsParam = Simulator::Now();
      roundFinished = false;
      NS_LOG_INFO("StartNewFLRound: ns-3 comms phase started for round "
                  << roundNumber << " at "
                  << roundStartTimeNs3CommsParam.GetSeconds()
                  << "s. roundFinished set to false.");
    } else {
      NS_LOG_INFO("StartNewFLRound: ns-3 round preparation successful, but no clients "
                  "selected by ns-3 for simulated upload. Marking round as "
                  "(comms-wise) finished.");
      roundFinished = true;
    }
  } else {
    NS_LOG_ERROR("StartNewFLRound: ns-3 round preparation FAILED for round "
                 << roundNumber << ". Skipping ns-3 comms phase.");
    roundFinished = true;
  }
}


void RosFlRoundCoordinator::manager() {
  static Time roundStartTimeNs3Comms = Simulator::Now();
  NS_LOG_INFO("Manager called at " << Simulator::Now().GetSeconds()
                                   << "s. RoundNumber: " << roundNumber
                                   << ", roundFinished (ns-3 comms): "
                                   << (roundFinished ? "true" : "false"));

	  if (!fl_api_initialized) {
	    NS_LOG_INFO("Manager: FL API not yet initialized by main. Manager waiting for "
	                << fl_cfg::kManagerInterval.GetSeconds() << " seconds.");
	    Simulator::Schedule(fl_cfg::kManagerInterval, &RosFlRoundCoordinator::manager);
	    return;
	  }

  if (!roundFinished) {
    NS_LOG_INFO("Manager: ns-3 communication phase for round " << roundNumber << " is ongoing.");
    if (isRoundTimedOut(roundStartTimeNs3Comms)) {
      NS_LOG_WARN("Manager: Round " << roundNumber << " ns-3 comms timed out.");
      logRoundTimeout();
      roundFinished = true;
    } else {
      nodesIPs = WirelessTransportManager::nodeToIps();
      bool all_selected_clients_finished_ns3_comms =
          WirelessTransportManager::checkFinishedTransmission(nodesIPs, selectedClientsForCurrentRound);

      if (all_selected_clients_finished_ns3_comms) {
        if (!selectedClientsForCurrentRound.empty() || !endOfStreamTimes.empty()) {
          NS_LOG_INFO("Manager: All selected clients ("
                      << endOfStreamTimes.size() << "/"
                      << selectedClientsForCurrentRound.size()
                      << ") completed ns-3 transmissions for round "
                      << roundNumber);
        } else if (selectedClientsForCurrentRound.empty()) {
          NS_LOG_INFO("Manager: No clients were selected for ns-3 comms in round "
              << roundNumber << ", considering comms phase complete.");
        }
        roundFinished = true;
      } else {
        NS_LOG_INFO("Manager: Waiting for "
            << selectedClientsForCurrentRound.size() - endOfStreamTimes.size()
            << " more clients to finish ns-3 comms for round " << roundNumber);
      }
    }

    if (roundFinished) {
      NS_LOG_INFO("Manager: Finalizing ns-3 comms phase for round " << roundNumber);
      finalizeNs3CommsPhase();
    }
  }

  if (roundFinished) {
    NS_LOG_INFO("Manager: ns-3 communication phase for round "
                << roundNumber << " is finished or was skipped.");


    if (Simulator::Now() >= nextRoundEligibleTime) {
      startNewFLRound(roundStartTimeNs3Comms);
    } else {
      NS_LOG_INFO("Manager: waiting for inter-round pause. Remaining "
                  << (nextRoundEligibleTime - Simulator::Now()).GetSeconds()
                  << "s before starting the next round.");
    }
  }

	  if (!Simulator::IsFinished()) {
	    NS_LOG_INFO("Manager: Scheduling next call in "
	                << fl_cfg::kManagerInterval.GetSeconds() << " seconds.");
	    Simulator::Schedule(fl_cfg::kManagerInterval, &RosFlRoundCoordinator::manager);
  } else {
    NS_LOG_INFO("Manager: Simulation is finished, not scheduling next call.");
  }
}
