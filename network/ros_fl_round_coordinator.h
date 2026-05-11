#pragma once

#include "robot_fl_state_types.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include <map>
#include <string>
#include <vector>

using namespace ns3;


extern std::vector<ClientModels> clientsInfoGlobal;
extern std::vector<ClientModels> selectedClientsForCurrentRound;
extern std::vector<NodesIps> nodesIPs;


class RosFlRoundCoordinator {
public:
    static bool initializeROSFL();
    static void selectNs3ManagedClients(int n_to_select);
    static bool triggerAndProcessFLRoundInApi();
    static void sendModelsToServer();
    static bool isRoundTimedOut(Time roundStartTimeNs3Comms);
    static void logRoundTimeout();
    static void finalizeNs3CommsPhase();
    static void startNewFLRound(Time &roundStartTimeNs3CommsParam);
    static void manager();

private:
    static std::string getEnvVar(const std::string &key, const std::string &default_val);
    static bool callROSFL(const std::string &ros_mode,
                          const std::string &int16_param_payload,
                          const std::string &json_param_payload);
    static std::string toUploadCompletionJsonPayload(const std::map<int, double>& data);
    static std::string yamlSingleQuote(const std::string& value);
    static std::string bashSingleQuote(const std::string& cmd);
    static std::string runCommand(const std::string &cmd, int &exit_status);
    static std::string exec(const char* cmd);
    static int parseResponseExitCode(const std::string& out, bool &success_value);
};

extern int roundNumber;
