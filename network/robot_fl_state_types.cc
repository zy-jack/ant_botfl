#include "robot_fl_state_types.h"
#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE("RobotFlStateTypes");


ClientModels::ClientModels(Ptr<Node> n, int t, int b, bool s, double r, double sin, double acc)
    : node(n),
      nodeTrainingTime(t),
      nodeModelSize(b),
      selected(s),
      rsrp(r),
      sinr(sin),
      accuracy(acc) {


}


ClientModels::ClientModels(Ptr<Node> n, int t, int b, double r, double sin, double acc)
    : node(n),
      nodeTrainingTime(t),
      nodeModelSize(b),
      selected(false),
      rsrp(r),
      sinr(sin),
      accuracy(acc) {


}


std::ostream& operator<<(std::ostream& os, const ClientModels& model) {
    os << "Clients_Models { id: " << (model.node ? model.node->GetId() : 0)
       << ", training_time: " << model.nodeTrainingTime << ", node_to_bytes: " << model.nodeModelSize
       << ", selected: " << (model.selected ? "true" : "false")
       << ", RSRP: " << model.rsrp << " dBm, SINR: " << model.sinr << " dB"
       << ", Accuracy: " << model.accuracy << " }";
    return os;
}


NodesIps::NodesIps(int n, int i, Ipv4Address ia)
    : nodeId(n),
      index(i),
      ip(ia) {


}
