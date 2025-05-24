/*
* Copyright (c) 2017 University of Padova
*
* SPDX-License-Identifier: GPL-2.0-only
*
* Author: Davide Magrin <magrinda@dei.unipd.it>
*/

#include "ns3/basic-energy-source-helper.h"
#include "ns3/building-allocator.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/buildings-helper.h"
#include "ns3/callback.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/double.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/file-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/log.h"
#include "ns3/lora-device-address.h"
#include "ns3/lora-frame-header.h"
#include "ns3/lora-helper.h"
#include "ns3/lora-net-device.h"
#include "ns3/lora-phy.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/lorawan-mac-header.h"
#include "ns3/mobility-helper.h"
#include "ns3/network-server-helper.h"
#include "ns3/node-container.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/pointer.h"
#include "ns3/position-allocator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <ctime>
#include <iomanip>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("AlohaThroughput");

// Network settings
int nDevices = 300;                 //!< Number of end device nodes to create
int nGateways = 1;                  //!< Number of gateway nodes to create
double radiusMeters = 1000;         //!< Radius (m) of the deployment
double simulationTimeSeconds = 180; //!< Scenario duration (s) in simulated time

// double simulationTimeSeconds = 500; //!< Scenario duration (s) in simulated time

// Channel model

bool realisticChannelModel = true; //!< Whether to use a more realistic channel model with
                                  //!< buildings and correlated shadowing

/** Record received pkts by Data Rate (DR) [index 0 -> DR5, index 5 -> DR0]. */
auto packetsSent = std::vector<int>(6, 0);
/** Record received pkts by Data Rate (DR) [index 0 -> DR5, index 5 -> DR0]. */
auto packetsReceived = std::vector<int>(6, 0);
/**  CAD */
bool csmaEnabled = true;

/**
* Record the beginning of a transmission by an end device.
*
* \param packet A pointer to the packet sent.
* \param senderNodeId Node id of the sender end device.
*/
void
OnTransmissionCallback(Ptr<const Packet> packet, uint32_t senderNodeId)
{
   LoraTag tag;
   packet->PeekPacketTag(tag);
   //    NS_LOG_FUNCTION("packetsSent at" <<
   //                    tag.GetSpreadingFactor() - 7 <<
   //                    ": " << packetsSent.at(tag.GetSpreadingFactor() - 7) <<
   //                    ". Packet: " << packet << senderNodeId);

   packetsSent.at(tag.GetSpreadingFactor() - 7)++;
}

/**
* Record the correct reception of a packet by a gateway.
*
* \param packet A pointer to the packet received.
* \param receiverNodeId Node id of the receiver gateway.
*/
void
OnPacketReceptionCallback(Ptr<const Packet> packet, uint32_t receiverNodeId)
{
   //    NS_LOG_FUNCTION(packet << receiverNodeId);
   LoraTag tag;
   packet->PeekPacketTag(tag);
   packetsReceived.at(tag.GetSpreadingFactor() - 7)++;
}

// Output control
bool printBuildingInfo = true; //!< Whether to print building information

std::vector<Vector>
GenerateClusterCentersWithAllocator(uint32_t numClusters, double totalAreaRadius)
{
   std::vector<Vector> clusterCenters;

   // Set up UniformDiscPositionAllocator centered at (0,0) with radius = totalAreaRadius
   Ptr<UniformDiscPositionAllocator> posAllocator = CreateObject<UniformDiscPositionAllocator>();
   posAllocator->SetX(0.0);
   posAllocator->SetY(0.0);
   posAllocator->SetRho(totalAreaRadius);

   // Retrieve positions
   for (uint32_t i = 0; i < numClusters; ++i)
   {
       clusterCenters.push_back(posAllocator->GetNext());
   }

   return clusterCenters;
}

int
main(int argc, char* argv[])
{
   std::string interferenceMatrix = "goursaud";
   CommandLine cmd(__FILE__);
   cmd.AddValue("nDevices", "Number of end devices to include in the simulation", nDevices);
   cmd.AddValue("simulationTime", "Simulation Time (s)", simulationTimeSeconds);
   cmd.AddValue("interferenceMatrix",
                "Interference matrix to use [aloha, goursaud]",
                interferenceMatrix);
   cmd.AddValue("radius", "Radius (m) of the deployment", radiusMeters);
   cmd.AddValue("CsmaEnabled", "ns3::EndDeviceLorawanMac::CsmaEnabled");
   cmd.AddValue("filePostfix", "", csmaEnabled);

   cmd.Parse(argc, argv);

   int appPeriodSeconds = 60;
   //    int appPeriodSeconds = simulationTimeSeconds / 3;

   // Set up logging
   LogComponentEnable("AlohaThroughput", LOG_LEVEL_ALL);
   //
   //        LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
   //        LogComponentEnable("LoraChannel", LOG_LEVEL_ALL);
   //        LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_WARN);
   //        LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);

   //        LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_INFO);
   //        LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_INFO);
   //        LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);

   //        LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
   //        LogComponentEnable("SimpleGatewayLoraPhy", LOG_LEVEL_INFO);
   //        LogComponentEnable("LoraRadioEnergyModel", LOG_LEVEL_ALL);

   // Make all devices use SF7 (i.e., DR5)
   //     Config::SetDefault ("ns3::EndDeviceLorawanMac::DataRate", UintegerValue (5));

   if (interferenceMatrix == "aloha")
   {
       LoraInterferenceHelper::collisionMatrix = LoraInterferenceHelper::ALOHA;
   }
   else if (interferenceMatrix == "goursaud")
   {
       LoraInterferenceHelper::collisionMatrix = LoraInterferenceHelper::GOURSAUD;
   }

   /***********
    *  Setup  *
    ***********/

   // Mobility
   MobilityHelper mobility;
   mobility.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
                                 "rho",
                                 DoubleValue(radiusMeters),
                                 "X",
                                 DoubleValue(0.0),
                                 "Y",
                                 DoubleValue(0.0));
   mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

   /************************
    *  Create the channel  *
    ************************/

   // Create the lora channel object
   Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
   loss->SetPathLossExponent(3.76);
   loss->SetReference(1, 7.7);

   if (realisticChannelModel)
   {
       // Create the correlated shadowing component
       Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
           CreateObject<CorrelatedShadowingPropagationLossModel>();

       // Aggregate shadowing to the logdistance loss
       loss->SetNext(shadowing);

       // Add the effect to the channel propagation loss
       Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss>();

       shadowing->SetNext(buildingLoss);
   }

   Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();

   Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);

   /************************
    *  Create the helpers  *
    ************************/

   // Create the LoraPhyHelper
   LoraPhyHelper phyHelper = LoraPhyHelper();
   phyHelper.SetChannel(channel);

   // Create the LorawanMacHelper
   LorawanMacHelper macHelper = LorawanMacHelper();
   macHelper.SetRegion(LorawanMacHelper::EU);

   // Create the LoraHelper
   LoraHelper helper = LoraHelper();
   helper.EnablePacketTracking(); // Output filename

   // Create the NetworkServerHelper
   NetworkServerHelper nsHelper = NetworkServerHelper();

   // Create the ForwarderHelper
   ForwarderHelper forHelper = ForwarderHelper();

   /************************
    *  Create End Devices  *
    ************************/

   uint32_t numClusters = 10;
   double radiusInsideClusterMeters = 0.5;
   auto centers = GenerateClusterCentersWithAllocator(numClusters, radiusMeters);
   NS_ASSERT(centers.size() == numClusters);

   // Create a set of nodes
   NodeContainer endDevices;
   endDevices.Create(nDevices);

   //    // Assign a mobility model to each node
   //    mobility.Install(endDevices);

   // Make it so that nodes are at a certain height > 0
   {
       uint32_t devicesPerClusterLower = nDevices / numClusters;
       uint32_t localIdx = 0;
       uint32_t clusterIdx = 0;
       for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
       {
           NS_ASSERT(clusterIdx < numClusters);
           mobility.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
                                         "rho",
                                         DoubleValue(radiusInsideClusterMeters),
                                         "X",
                                         DoubleValue(centers[clusterIdx].x),
                                         "Y",
                                         DoubleValue(centers[clusterIdx].y));
           mobility.Install(*j);

           Ptr<MobilityModel> localMobility = (*j)->GetObject<MobilityModel>();
           Vector pos = localMobility->GetPosition();
           localMobility->SetPosition(Vector(pos.x, pos.y, 1.2));

           ++localIdx;
           if (localIdx == devicesPerClusterLower && clusterIdx != numClusters - 1) {
               localIdx = 0;
               ++clusterIdx;
           }
       }
   }

   {
       // TODO: delete (debug logs)
       uint32_t i = 0;
       for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
       {
           auto p = (*j)->GetObject<MobilityModel>()->GetPosition();
           NS_LOG_WARN(
               "{" << "i: " << i
                   << ", x: "
                   << p.x
                   << ", y: "
                   << p.y
                   << ", z: "
                   << p.z
                   << "},"
           );
           ++i;
       }
   }

   // Create the LoraNetDevices of the end devices
   uint8_t nwkId = 54;
   uint32_t nwkAddr = 1864;
   Ptr<LoraDeviceAddressGenerator> addrGen =
       CreateObject<LoraDeviceAddressGenerator>(nwkId, nwkAddr);

   // Create the LoraNetDevices of the end devices
   macHelper.SetAddressGenerator(addrGen);
   phyHelper.SetDeviceType(LoraPhyHelper::ED);
   macHelper.SetDeviceType(LorawanMacHelper::ED_A);
   NetDeviceContainer endDevicesNetDevices = helper.Install(phyHelper, macHelper, endDevices);

   // Now end devices are connected to the channel

   // Connect trace sources
   for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
   {
       Ptr<Node> node = *j;
       Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(node->GetDevice(0));
       Ptr<LoraPhy> phy = loraNetDevice->GetPhy();
   }

   /*********************
    *  Create Gateways  *
    *********************/

   // Create the gateway nodes (allocate them uniformly on the disc)
   NodeContainer gateways;
   gateways.Create(nGateways);

   Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
   // Make it so that nodes are at a certain height > 0
   allocator->Add(Vector(0.0, 0.0, 15.0));
   mobility.SetPositionAllocator(allocator);
   mobility.Install(gateways);

   // Create a netdevice for each gateway
   phyHelper.SetDeviceType(LoraPhyHelper::GW);
   macHelper.SetDeviceType(LorawanMacHelper::GW);
   helper.Install(phyHelper, macHelper, gateways);

   /**********************
    *  Handle buildings  *
    **********************/

   double xLength = 100;
   double deltaX = 32;
   double yLength = 10;
   double deltaY = 17;
   int gridWidth = 2 * radiusMeters / (xLength + deltaX);
   int gridHeight = 2 * radiusMeters / (yLength + deltaY);
   if (!realisticChannelModel)
   {
       gridWidth = 0;
       gridHeight = 0;
   }
   Ptr<GridBuildingAllocator> gridBuildingAllocator;
   gridBuildingAllocator = CreateObject<GridBuildingAllocator>();
   gridBuildingAllocator->SetAttribute("GridWidth", UintegerValue(gridWidth));
   gridBuildingAllocator->SetAttribute("LengthX", DoubleValue(xLength));
   gridBuildingAllocator->SetAttribute("LengthY", DoubleValue(yLength));
   gridBuildingAllocator->SetAttribute("DeltaX", DoubleValue(deltaX));
   gridBuildingAllocator->SetAttribute("DeltaY", DoubleValue(deltaY));
   gridBuildingAllocator->SetAttribute("Height", DoubleValue(12));
   gridBuildingAllocator->SetBuildingAttribute("NRoomsX", UintegerValue(4));
   gridBuildingAllocator->SetBuildingAttribute("NRoomsY", UintegerValue(8));
   gridBuildingAllocator->SetBuildingAttribute("NFloors", UintegerValue(10));
   gridBuildingAllocator->SetAttribute(
       "MinX",
       DoubleValue(-gridWidth * (xLength + deltaX) / 2 + deltaX / 2));
   gridBuildingAllocator->SetAttribute(
       "MinY",
       DoubleValue(-gridHeight * (yLength + deltaY) / 2 + deltaY / 2));
   BuildingContainer bContainer = gridBuildingAllocator->Create(gridWidth * gridHeight);

   BuildingsHelper::Install(endDevices);
   BuildingsHelper::Install(gateways);

   // Print the buildings
   if (printBuildingInfo)
   {
       std::ofstream myfile;
       myfile.open("buildings.txt");
       std::vector<Ptr<Building>>::const_iterator it;
       int j = 1;
       for (it = bContainer.Begin(); it != bContainer.End(); ++it, ++j)
       {
           Box boundaries = (*it)->GetBoundaries();
           myfile << "set object " << j << " rect from " << boundaries.xMin << ","
                  << boundaries.yMin << " to " << boundaries.xMax << "," << boundaries.yMax
                  << std::endl;
       }
       myfile.close();
   }

   NS_LOG_DEBUG("Completed configuration");

   /*********************************************
    *  Install applications on the end devices  *
    *********************************************/

   Time appStopTime = Seconds(simulationTimeSeconds);
   int packetSize = 10;

   PeriodicSenderHelper appHelper = PeriodicSenderHelper();
   appHelper.SetPacketSize(packetSize);

   ApplicationContainer appContainer{};

   // Разброс периодов
   for (int i = 0; i < nDevices; ++i)
   {
       appHelper.SetPeriod(Seconds((double)appPeriodSeconds / 2 +
                                   (double)appPeriodSeconds / 10 * (double)i / (double)nDevices));
       //        appHelper.SetPeriod(Seconds(appPeriodSeconds - appPeriodSeconds / (i % 10 + 1)));
       appContainer.Add(appHelper.Install(endDevices));
   }

   appContainer.Start(Seconds(0));
   appContainer.Stop(appStopTime);

   std::ofstream outputFile;
   // Delete contents of the file as it is opened
   outputFile.open("durations.txt", std::ofstream::out | std::ofstream::trunc);
   for (uint8_t sf = 7; sf <= 12; sf++)
   {
       LoraTxParameters txParams;
       txParams.sf = sf;
       txParams.headerDisabled = false;
       txParams.codingRate = 1;
       txParams.bandwidthHz = 125000;
       txParams.nPreamble = 8;
       txParams.crcEnabled = true;
       txParams.lowDataRateOptimizationEnabled = LoraPhy::GetTSym(txParams) > MilliSeconds(16);
       Ptr<Packet> pkt = Create<Packet>(packetSize);

       LoraFrameHeader frameHdr = LoraFrameHeader();
       frameHdr.SetAsUplink();
       frameHdr.SetFPort(1);
       frameHdr.SetAddress(LoraDeviceAddress());
       frameHdr.SetAdr(false);
       //        frameHdr.SetAdr(true);
       frameHdr.SetAdrAckReq(false);
       frameHdr.SetFCnt(0);
       pkt->AddHeader(frameHdr);

       LorawanMacHeader macHdr = LorawanMacHeader();
       macHdr.SetMType(LorawanMacHeader::UNCONFIRMED_DATA_UP);
       macHdr.SetMajor(1);
       pkt->AddHeader(macHdr);

       NS_LOG_WARN("pkt->GetSize(): " << pkt->GetSize());

       outputFile << LoraPhy::GetOnAirTime(pkt, txParams).GetMicroSeconds() << " ";
   }
   outputFile.close();

   /**************************
    *  Create network server  *
    ***************************/

   // Create the network server node
   Ptr<Node> networkServer = CreateObject<Node>();

   // PointToPoint links between gateways and server
   PointToPointHelper p2p;
   p2p.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
   p2p.SetChannelAttribute("Delay", StringValue("2ms"));
   // Store network server app registration details for later
   P2PGwRegistration_t gwRegistration;
   for (auto gw = gateways.Begin(); gw != gateways.End(); ++gw)
   {
       auto container = p2p.Install(networkServer, *gw);
       auto serverP2PNetDev = DynamicCast<PointToPointNetDevice>(container.Get(0));
       gwRegistration.emplace_back(serverP2PNetDev, *gw);
   }

   // Create a network server for the network
   nsHelper.SetGatewaysP2P(gwRegistration);
   nsHelper.SetEndDevices(endDevices);
   nsHelper.Install(networkServer);

   // Create a forwarder for each gateway
   forHelper.Install(gateways);

   // Install trace sources
   for (auto node = gateways.Begin(); node != gateways.End(); node++)
   {
       DynamicCast<LoraNetDevice>((*node)->GetDevice(0))
           ->GetPhy()
           ->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(OnPacketReceptionCallback));
   }

   // Install trace sources
   for (auto node = endDevices.Begin(); node != endDevices.End(); node++)
   {
       DynamicCast<LoraNetDevice>((*node)->GetDevice(0))
           ->GetPhy()
           ->TraceConnectWithoutContext("StartSending", MakeCallback(OnTransmissionCallback));
   }

   LorawanMacHelper::SetSimilarSpreadingFactorsUp(endDevices);

   /************************
    * Install Energy Model *
    ************************/

   BasicEnergySourceHelper basicSourceHelper;
   LoraRadioEnergyModelHelper radioEnergyHelper;

   // configure energy source
   basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(10000)); // Energy in J
   basicSourceHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(3.3));

   radioEnergyHelper.Set("StandbyCurrentA", DoubleValue(0.0014));
   radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.028));
   radioEnergyHelper.Set("SleepCurrentA", DoubleValue(0.0000015));
   radioEnergyHelper.Set("RxCurrentA", DoubleValue(0.0112));

   radioEnergyHelper.SetTxCurrentModel("ns3::ConstantLoraTxCurrentModel",
                                       "TxCurrent",
                                       DoubleValue(0.028));

   // install source on end devices' nodes
   EnergySourceContainer sources = basicSourceHelper.Install(endDevices);

   uint32_t step = sources.GetN() / 5;

   for (uint32_t i = 0; i < sources.GetN(); i += step)
   {
       Names::Add("/Names/EnergySource" + std::to_string(i), sources.Get(i));
   }

   std::cout << "Size: " << sources.GetN() << std::endl;

   // install device model
   DeviceEnergyModelContainer deviceModels =
       radioEnergyHelper.Install(endDevicesNetDevices, sources);

   /**************
    * Get output *
    **************/

   std::vector<std::string> energySourceNames;
   for (uint32_t i = 0; i < sources.GetN(); i += step)
   {
       energySourceNames.push_back("/Names/EnergySource" + std::to_string(i) + "/RemainingEnergy");
   }

   std::string cadFlag = csmaEnabled ? "cad" : "without_cad";

   FileHelper fileHelper; //    fileHelper.ConfigureFile("battery-level-without-cad-" +
                          //    std::to_string(nDevices),
   //    FileAggregator::SPACE_SEPARATED);
   fileHelper.ConfigureFile("battery-level-" + cadFlag + "-" + std::to_string(simulationTimeSeconds) + "" + std::to_string(nDevices),
                            FileAggregator::SPACE_SEPARATED);
   //    fileHelper.Set2dFormat("Time (Seconds) = %.3e\tEnergy Level = %.0f");

   // TODO раскомментировать
   fileHelper.WriteProbeArray("ns3::DoubleProbe", energySourceNames, "Output");
   //    fileHelper.WriteProbe("ns3::DoubleProbe", "/Names/EnergySource1/RemainingEnergy",
   //    "Output");

   ////////////////
   // Simulation //
   ////////////////

   Simulator::Stop(appStopTime + Hours(1));

   NS_LOG_INFO("Running simulation...");
   Simulator::Run();

   Simulator::Destroy();

   /////////////////////////////
   // Print results to stdout //
   /////////////////////////////
   NS_LOG_INFO("Computing performance metrics...");

   for (int i = 0; i < 6; i++)
   {
       std::cout << "Packet sent at SF=" << i + 7 << ": " << packetsSent.at(i) << "; "
                 << "Packet received at SF=" << i + 7 << ": " << packetsReceived.at(i)
                 << std::endl;
   }

   for (int i = 0; i < 6; i++)
   {
       std::cout <<i + 7 << ": " << sources.GetN() << ": "
                 << "(" << packetsSent.at(i) << ", "
                 << packetsReceived.at(i) << ")"
                 << std::endl;
   }

   return 0;
}
