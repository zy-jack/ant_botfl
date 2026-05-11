
#pragma once

#include "ns3/network-module.h"

namespace wireless_visualization
{

void
ConfigureNetAnimExport(bool enabled,
                       const ns3::NodeContainer& ueNodes,
                       const ns3::NodeContainer& enbNodes,
                       const ns3::NodeContainer& remoteHostContainer,
                       double simStopTimeSec);

}
