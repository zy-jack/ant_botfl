
#include "wireless_scenario_visualizer.h"
#include "robot_fl_simulation_config.h"

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/node-list.h"

#include <memory>
#include <sstream>
#include <string>
#include <vector>

NS_LOG_COMPONENT_DEFINE("WirelessScenarioVisualizer");

namespace wireless_visualization
{
namespace
{

const auto& kCfg = robot_fl_config::visualization::kNetAnim;
std::unique_ptr<ns3::AnimationInterface> gAnim;
bool gConfigured = false;
std::vector<uint32_t> gAutoPlacedCoreNodeIds;

void
EnsureConstantPosition(ns3::Ptr<ns3::Node> node, const ns3::Vector& pos)
{
    using namespace ns3;

    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    if (!mobility)
    {
        MobilityHelper helper;
        helper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        helper.Install(node);
        mobility = node->GetObject<MobilityModel>();
    }

    mobility->SetPosition(pos);
}

void
AutoPlaceNodesWithoutMobility()
{
    using namespace ns3;

    gAutoPlacedCoreNodeIds.clear();

    double x = kCfg.autoPlacedCoreStart.x;
    double y = kCfg.autoPlacedCoreStart.y;
    const double z = kCfg.autoPlacedCoreStart.z;

    for (auto it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        if (!node->GetObject<MobilityModel>())
        {
            EnsureConstantPosition(node, Vector(x, y, z));
            gAutoPlacedCoreNodeIds.push_back(node->GetId());
            y += kCfg.autoPlacedCoreStepY;
        }
    }
}

void
SetDescriptions(const ns3::NodeContainer& ueNodes,
                const ns3::NodeContainer& enbNodes,
                const ns3::NodeContainer& remoteHostContainer)
{
    using namespace ns3;

    if (remoteHostContainer.GetN() > 0)
    {
        gAnim->UpdateNodeDescription(remoteHostContainer.Get(0), kCfg.remoteHostLabel);
    }

    for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
    {
        std::ostringstream oss;
        oss << kCfg.enbLabelPrefix << (i + 1);
        gAnim->UpdateNodeDescription(enbNodes.Get(i), oss.str());
    }

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        if (i == 0)
        {
            gAnim->UpdateNodeDescription(ueNodes.Get(i), kCfg.firstUeLabel);
            continue;
        }

        if (kCfg.showAllUeLabels)
        {
            std::ostringstream oss;
            oss << kCfg.otherUeLabelPrefix << i;
            gAnim->UpdateNodeDescription(ueNodes.Get(i), oss.str());
        }
        else
        {
            gAnim->UpdateNodeDescription(ueNodes.Get(i), "");
        }
    }

    for (uint32_t i = 0; i < gAutoPlacedCoreNodeIds.size(); ++i)
    {
        if (i < kCfg.coreNodeLabels.size())
        {
            gAnim->UpdateNodeDescription(gAutoPlacedCoreNodeIds[i], kCfg.coreNodeLabels[i]);
        }
        else
        {
            std::ostringstream oss;
            oss << kCfg.coreNodeLabelPrefix << (i + 1);
            gAnim->UpdateNodeDescription(gAutoPlacedCoreNodeIds[i], oss.str());
        }
    }
}

void
SetIcons(const ns3::NodeContainer& ueNodes,
         const ns3::NodeContainer& enbNodes,
         const ns3::NodeContainer& remoteHostContainer)
{
    uint32_t enbIconId = 0;
    uint32_t ueIconId = 0;
    uint32_t serverIconId = 0;
    uint32_t coreNodeIconId = 0;
    const bool hasEnbIcon = !kCfg.enbIcon.empty();
    const bool hasUeIcon = !kCfg.ueIcon.empty();
    const bool hasServerIcon = !kCfg.serverIcon.empty();
    const bool hasCoreNodeIcon = !kCfg.coreNodeIcon.empty();

    if (hasEnbIcon)
    {
        enbIconId = gAnim->AddResource(kCfg.enbIcon);
    }
    if (hasUeIcon)
    {
        ueIconId = gAnim->AddResource(kCfg.ueIcon);
    }
    if (hasServerIcon)
    {
        serverIconId = gAnim->AddResource(kCfg.serverIcon);
    }
    if (hasCoreNodeIcon)
    {
        coreNodeIconId = (hasServerIcon && kCfg.coreNodeIcon == kCfg.serverIcon)
                             ? serverIconId
                             : gAnim->AddResource(kCfg.coreNodeIcon);
    }

    if (hasEnbIcon)
    {
        for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
        {
            gAnim->UpdateNodeImage(enbNodes.Get(i)->GetId(), enbIconId);
        }
    }

    if (hasUeIcon)
    {
        for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
        {
            gAnim->UpdateNodeImage(ueNodes.Get(i)->GetId(), ueIconId);
        }
    }

    if (hasServerIcon && remoteHostContainer.GetN() > 0)
    {
        gAnim->UpdateNodeImage(remoteHostContainer.Get(0)->GetId(), serverIconId);
    }

    if (hasCoreNodeIcon)
    {
        for (uint32_t nodeId : gAutoPlacedCoreNodeIds)
        {
            gAnim->UpdateNodeImage(nodeId, coreNodeIconId);
        }
    }
}

void
SetColors(const ns3::NodeContainer& ueNodes,
          const ns3::NodeContainer& enbNodes,
          const ns3::NodeContainer& remoteHostContainer)
{
    if (remoteHostContainer.GetN() > 0)
    {
        gAnim->UpdateNodeColor(remoteHostContainer.Get(0),
                               kCfg.remoteHostColor.r,
                               kCfg.remoteHostColor.g,
                               kCfg.remoteHostColor.b);
    }

    for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
    {
        if (i == 0)
        {
            gAnim->UpdateNodeColor(enbNodes.Get(i),
                                   kCfg.firstEnbColor.r,
                                   kCfg.firstEnbColor.g,
                                   kCfg.firstEnbColor.b);
        }
        else if (i == 1)
        {
            gAnim->UpdateNodeColor(enbNodes.Get(i),
                                   kCfg.secondEnbColor.r,
                                   kCfg.secondEnbColor.g,
                                   kCfg.secondEnbColor.b);
        }
        else
        {
            gAnim->UpdateNodeColor(enbNodes.Get(i),
                                   kCfg.otherEnbColor.r,
                                   kCfg.otherEnbColor.g,
                                   kCfg.otherEnbColor.b);
        }
    }

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        if (kCfg.highlightFirstUe && i == 0)
        {
            gAnim->UpdateNodeColor(ueNodes.Get(i),
                                   kCfg.firstUeColor.r,
                                   kCfg.firstUeColor.g,
                                   kCfg.firstUeColor.b);
        }
        else
        {
            gAnim->UpdateNodeColor(ueNodes.Get(i),
                                   kCfg.otherUeColor.r,
                                   kCfg.otherUeColor.g,
                                   kCfg.otherUeColor.b);
        }
    }

    for (uint32_t nodeId : gAutoPlacedCoreNodeIds)
    {
        gAnim->UpdateNodeColor(nodeId,
                               kCfg.coreNodeColor.r,
                               kCfg.coreNodeColor.g,
                               kCfg.coreNodeColor.b);
    }
}

void
SetSizes(const ns3::NodeContainer& ueNodes,
         const ns3::NodeContainer& enbNodes,
         const ns3::NodeContainer& remoteHostContainer)
{
    if (remoteHostContainer.GetN() > 0)
    {
        gAnim->UpdateNodeSize(remoteHostContainer.Get(0)->GetId(),
                              kCfg.remoteHostSizeX,
                              kCfg.remoteHostSizeY);
    }

    for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
    {
        gAnim->UpdateNodeSize(enbNodes.Get(i)->GetId(), kCfg.enbSizeX, kCfg.enbSizeY);
    }

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        gAnim->UpdateNodeSize(ueNodes.Get(i)->GetId(), kCfg.ueSizeX, kCfg.ueSizeY);
    }

    for (uint32_t nodeId : gAutoPlacedCoreNodeIds)
    {
        gAnim->UpdateNodeSize(nodeId, kCfg.coreNodeSizeX, kCfg.coreNodeSizeY);
    }
}

}

void
ConfigureNetAnimExport(bool enabled,
                       const ns3::NodeContainer& ueNodes,
                       const ns3::NodeContainer& enbNodes,
                       const ns3::NodeContainer& remoteHostContainer,
                       double simStopTimeSec)
{
    using namespace ns3;

    if (!enabled)
    {
        NS_LOG_INFO("NetAnim export disabled.");
        return;
    }

    if (gConfigured)
    {
        NS_LOG_INFO("NetAnim export already configured.");
        return;
    }
    gConfigured = true;

    if (remoteHostContainer.GetN() > 0)
    {
        EnsureConstantPosition(remoteHostContainer.Get(0), kCfg.remoteHostPosition);
    }

    AutoPlaceNodesWithoutMobility();

    gAnim = std::make_unique<AnimationInterface>(kCfg.xmlFile);
    gAnim->SetMobilityPollInterval(kCfg.mobilityPollInterval);
    gAnim->SetStartTime(Seconds(0.0));
    gAnim->SetStopTime(Seconds(simStopTimeSec));
    if (kCfg.enablePacketAnimation)
    {
        gAnim->SetMaxPktsPerTraceFile(kCfg.maxPacketsPerTraceFile);
        gAnim->EnablePacketMetadata(false);
    }
    else
    {
        gAnim->SkipPacketTracing();
    }

    if (kCfg.enableBackgroundImage && !kCfg.backgroundImage.empty())
    {
        gAnim->SetBackgroundImage(kCfg.backgroundImage,
                                  kCfg.backgroundX,
                                  kCfg.backgroundY,
                                  kCfg.backgroundScaleX,
                                  kCfg.backgroundScaleY,
                                  kCfg.backgroundOpacity);
    }

    SetDescriptions(ueNodes, enbNodes, remoteHostContainer);
    SetIcons(ueNodes, enbNodes, remoteHostContainer);
    SetColors(ueNodes, enbNodes, remoteHostContainer);
    SetSizes(ueNodes, enbNodes, remoteHostContainer);

    NS_LOG_INFO("NetAnim export enabled, xml=" << kCfg.xmlFile
                                               << ", packetAnimation="
                                               << (kCfg.enablePacketAnimation ? "true" : "false"));
}

}
