/*
 * nslora-sim.cc
 *
 * Created on: Nov 8, 2018
 * Author: ibnufebry
 */

#include "ns3/point-to-point-module.h"
#include "ns3/forwarder-helper.h"
#include "ns3/network-server-helper.h"
#include "ns3/lora-channel.h"
#include "ns3/mobility-helper.h"
#include "ns3/lora-phy-helper.h"
#include "ns3/lora-mac-helper.h"
#include "ns3/lora-helper.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/periodic-sender.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/lora-device-address-generator.h"
#include "ns3/one-shot-sender-helper.h"
#include "ns3/simple-network-server.h"
#include <string.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NsLoraSim");

class NsLoraSim {
public:
	NsLoraSim ();
	NsLoraSim (int, uint8_t, double, double, uint64_t);
	NsLoraSim (int, int, double, uint64_t);
	NsLoraSim (int, double, uint8_t, uint64_t);
	~NsLoraSim ();
	void Run (void);
private:
	int nDevices;
	uint8_t gatewayRings;
	int nGateways;
	double radius;
	double gatewayRadius;
	double simulationTime;
	uint8_t appPeriodSeconds;

	int noMoreReceivers = 0;
	int interfered = 0;
	int received = 0;
	int underSensitivity = 0;

	uint16_t transmittedPkt = 0;
	uint64_t rRand = 0;

	bool printdev;
	int mode = 0;

	enum PacketOutcome {
	  RECEIVED,
	  INTERFERED,
	  NO_MORE_RECEIVERS,
	  UNDER_SENSITIVITY,
	  UNSET
	};

	struct PacketStatus {
	  Ptr<Packet const> packet;
	  uint32_t senderId;
	  int outcomeNumber;
	  std::vector<enum PacketOutcome> outcomes;
	};

	std::map<Ptr<Packet const>, PacketStatus> packetTracker;

	void CheckReceptionByAllGWsComplete (std::map<Ptr<Packet const>, PacketStatus>::iterator );
	void TransmissionCallback (Ptr<Packet const>, uint32_t );
	void PacketReceptionCallback (Ptr<Packet const> , uint32_t );
	void InterferenceCallback (Ptr<Packet const> , uint32_t );
	void NoMoreReceiversCallback (Ptr<Packet const> , uint32_t );
	void UnderSensitivityCallback (Ptr<Packet const> , uint32_t );
	void CreateMap (NodeContainer , NodeContainer , NodeContainer , std::string );
};

NsLoraSim::NsLoraSim () :
		nDevices (100),
		radius (7500),
		gatewayRadius (1),
		simulationTime (100.0),
		appPeriodSeconds (10),
		printdev (true)
{
	gatewayRings = 1;
	nGateways = 3*gatewayRings*gatewayRings-3*gatewayRings+1;
	gatewayRadius = 7500/((gatewayRings-1)*2+1);
}

NsLoraSim::NsLoraSim (int m_ndevice, uint8_t m_gatewayRings, double m_radius, double m_simulationTime, uint64_t m_rand) :
		nDevices (100),
		radius (7500),
		gatewayRadius (7500),
		simulationTime (100.0),
		appPeriodSeconds (10),
		printdev (true)
{
	nDevices = m_ndevice;
	gatewayRings = m_gatewayRings;
	nGateways = 3*gatewayRings*gatewayRings-3*gatewayRings+1;
	gatewayRadius = 7500/((gatewayRings-1)*2+1);
	rRand = m_rand;
    simulationTime = m_simulationTime;
}

NsLoraSim::NsLoraSim (int m_ndevice, int m_nGateways, double m_simulationTime, uint64_t m_rand) :
		nDevices (100),
		gatewayRings (2),
		radius (7500),
		gatewayRadius (3500),
		simulationTime (100.0),
		appPeriodSeconds (10),
		printdev (true)
{
	nDevices = m_ndevice;
	nGateways = m_nGateways;
	rRand = m_rand;
    simulationTime = m_simulationTime;

    mode = 0;
}

NsLoraSim::NsLoraSim (int m_nGateways, double m_simulationTime, uint8_t m_appPeriod, uint64_t m_rand) :
		nDevices (100),
		gatewayRings (2),
		radius (7500),
		gatewayRadius (3500),
		simulationTime (100.0),
		appPeriodSeconds (10),
		printdev (true)
{
	nGateways = m_nGateways;
	appPeriodSeconds = m_appPeriod;
	rRand = m_rand;
	simulationTime = m_simulationTime;

	mode = 1;
}

NsLoraSim::~NsLoraSim()
{
	NS_LOG_INFO ("finishing simulation...");
}

void
NsLoraSim::CheckReceptionByAllGWsComplete (std::map<Ptr<Packet const>, PacketStatus>::iterator it)
{
  // Check whether this packet is received by all gateways
  if ((*it).second.outcomeNumber == nGateways)
    {
      // Update the statistics
      PacketStatus status = (*it).second;
      for (int j = 0; j < nGateways; j++)
        {
          switch (status.outcomes.at (j))
            {
            case RECEIVED:
              {
                received += 1;
                break;
              }
            case UNDER_SENSITIVITY:
              {
                underSensitivity += 1;
                break;
              }
            case NO_MORE_RECEIVERS:
              {
                noMoreReceivers += 1;
                break;
              }
            case INTERFERED:
              {
                interfered += 1;
                break;
              }
            case UNSET:
              {
                break;
              }
            }
        }
      // Remove the packet from the tracker
      packetTracker.erase (it);
    }
}

void
NsLoraSim::TransmissionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // NS_LOG_DEBUG ("Transmitted a packet from device " << systemId);
  // Create a packetStatus
  PacketStatus status;
  status.packet = packet;
  status.senderId = systemId;
  status.outcomeNumber = 0;
  status.outcomes = std::vector<enum PacketOutcome> (nGateways, UNSET);

  packetTracker.insert (std::pair<Ptr<Packet const>, PacketStatus> (packet, status));
}

void
NsLoraSim::PacketReceptionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  std::map<Ptr<Packet const>, PacketStatus>::iterator it = packetTracker.find (packet);
  (*it).second.outcomes.at (systemId - nDevices) = RECEIVED;
  (*it).second.outcomeNumber += 1;

  // Ptr<Packet> pkt = packet->Copy ();
  // LoraTag tag;
  // pkt->RemovePacketTag(tag);
  // uint16_t sendtime = tag.GetSendtime();

  // Remove the successfully received packet from the list of sent ones
  // NS_LOG_INFO ("A packet was successfully received at server " << systemId << " stime: " << sendtime);

  CheckReceptionByAllGWsComplete (it);
}

void
NsLoraSim::InterferenceCallback (Ptr<Packet const> packet, uint32_t systemId)
{
	// NS_LOG_INFO ("A packet was interferenced " << systemId);

	std::map<Ptr<Packet const>, PacketStatus>::iterator it = packetTracker.find (packet);
	it->second.outcomes.at (systemId - nDevices) = INTERFERED;
	it->second.outcomeNumber += 1;
}

void
NsLoraSim::NoMoreReceiversCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // NS_LOG_INFO ("A packet was lost because there were no more receivers at gateway " << systemId);

  std::map<Ptr<Packet const>, PacketStatus>::iterator it = packetTracker.find (packet);
  (*it).second.outcomes.at (systemId - nDevices) = NO_MORE_RECEIVERS;
  (*it).second.outcomeNumber += 1;

  CheckReceptionByAllGWsComplete (it);
}

void
NsLoraSim::UnderSensitivityCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // NS_LOG_INFO ("A packet arrived at the gateway under sensitivity at gateway " << systemId);

  std::map<Ptr<Packet const>, PacketStatus>::iterator it = packetTracker.find (packet);
  (*it).second.outcomes.at (systemId - nDevices) = UNDER_SENSITIVITY;
  (*it).second.outcomeNumber += 1;

  CheckReceptionByAllGWsComplete (it);
}

void
NsLoraSim::CreateMap (NodeContainer eds, NodeContainer gws, NodeContainer svr, std::string fname)
{
	const char* cfname = fname.c_str();
	std::ofstream fd;
	fd.open (cfname);

	for (NodeContainer::Iterator i = eds.Begin(); i != eds.End(); ++i)
	{
		Ptr<Node> object = *i;
		Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
		NS_ASSERT (position != 0);
		Ptr<NetDevice> netDevice = object->GetDevice (0);
		Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
		NS_ASSERT (loraNetDevice != 0);
		Ptr<EndDeviceLoraMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLoraMac> ();
		int sf = int(mac->GetDataRate ());
		Vector pos = position->GetPosition ();
		fd << pos.x << " " << pos.y << " " << sf << std::endl;
	}
	fd.close();
	std::ostringstream oss;
	oss << "dat/"<< mode <<"/gw-"<< nDevices <<"-"<< rRand <<"-r-"<< nGateways  <<"-p"<< appPeriodSeconds <<".dat";
	fd.open (oss.str());
	for (NodeContainer::Iterator i = gws.Begin(); i != gws.End(); ++i)
	{
		Ptr<Node> object = *i;
		Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
		NS_ASSERT (position != 0);

		Vector pos = position->GetPosition ();
		fd << pos.x << " " << pos.y << " 2" << std::endl;
	}
	fd.close();

	oss.clear();
	oss << "dat/"<< mode <<"/srv-"<< nDevices <<"-"<< rRand <<"-r-"<< nGateways  <<"-p"<< appPeriodSeconds <<".dat";
	fd.open (oss.str());
	Ptr<MobilityModel> position = svr.Get(0)->GetObject<MobilityModel>();
	NS_ASSERT (position != 0);

	Vector pos = position->GetPosition ();
	fd << pos.x << " " << pos.y << " 7" << std::endl;
	fd.close();
	oss.clear ();
}

void
NsLoraSim::Run (void)
{
	RngSeedManager::SetRun(rRand);
	RngSeedManager::SetSeed(1);

	// Create a simple wireless channel
	Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
	loss->SetPathLossExponent (3.76);
	loss->SetReference (1, 8.1);

	Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
	Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

	// Helpers
	// End Device mobility
	MobilityHelper mobilityEd, mobilityGw, mobilitySv;
	mobilityEd.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
									"rho",DoubleValue (radius),
									 "X", DoubleValue (0.0),
									 "Y", DoubleValue (0.0));
	mobilityEd.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

	// Gateway mobility
	Ptr<ListPositionAllocator> positionAllocGw = CreateObject<ListPositionAllocator> ();
	positionAllocGw->Add (Vector (-3500.0, 3500.0, 0.0));
	positionAllocGw->Add (Vector (3500.0, 3500.0, 0.0));
	positionAllocGw->Add (Vector (3500.0, -3500.0, 0.0));
	positionAllocGw->Add (Vector (-3500.0, -3500.0, 0.0));
	mobilityGw.SetPositionAllocator(positionAllocGw);
	mobilityGw.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

	// Server mobility
	Ptr<ListPositionAllocator> positionAllocSv = CreateObject<ListPositionAllocator> ();
	positionAllocSv->Add (Vector (0.0, 0.0, 0.0));
	mobilitySv.SetPositionAllocator(positionAllocSv);

	// Create the LoraPhyHelper
	LoraPhyHelper phyHelper = LoraPhyHelper ();
	phyHelper.SetChannel (channel);

	// Create the LoraMacHelper
	LoraMacHelper macHelper = LoraMacHelper ();

	// Create the LoraHelper
	LoraHelper helper = LoraHelper ();

	// Create EDs
	NodeContainer endDevices;
	endDevices.Create (nDevices);
	mobilityEd.Install (endDevices);
	for (NodeContainer::Iterator i = endDevices.Begin(); i!= endDevices.End(); ++i)
	{
	  Ptr<Node> nd = *i;
	  Ptr<MobilityModel> mob = nd->GetObject<MobilityModel> ();
	  Vector pos = mob->GetPosition();
	  pos.z = 1.2;
	  mob->SetPosition( pos);
	}

	// Create a LoraDeviceAddressGenerator
	uint8_t nwkId = 54;
	uint32_t nwkAddr = 1864;
	Ptr<LoraDeviceAddressGenerator> addrGen = CreateObject<LoraDeviceAddressGenerator> (nwkId,nwkAddr);

	// Create the LoraNetDevices of the end devices
	phyHelper.SetDeviceType (LoraPhyHelper::ED);
	macHelper.SetDeviceType (LoraMacHelper::ED);
	macHelper.SetAddressGenerator (addrGen);
	macHelper.SetRegion (LoraMacHelper::EU);
	helper.Install (phyHelper, macHelper, endDevices);

	// Install applications in EDs
	Time appStopTime = Seconds (simulationTime);
	PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
	appHelper.SetPeriod (Seconds (appPeriodSeconds));
	ApplicationContainer appContainer = appHelper.Install (endDevices);

	// GW setup
	NodeContainer gateways;
	gateways.Create (nGateways);
	mobilityGw.Install (gateways);

	for (NodeContainer::Iterator i= gateways.Begin (); i != gateways.End (); ++i)
	{
	  Ptr<Node> gw = *i;
	  Ptr<MobilityModel> mob = gw->GetObject<MobilityModel> ();
	  Vector pos = mob->GetPosition();
	  pos.z = 1.2;
	  mob->SetPosition(pos);
	}

	// LoraNetDevices GW
	phyHelper.SetDeviceType (LoraPhyHelper::GW);
	macHelper.SetDeviceType (LoraMacHelper::GW);
	helper.Install (phyHelper, macHelper, gateways);

	// Set spreading factors up
	macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);

	// NS setup
	NodeContainer networkServers;
	networkServers.Create (1);

	// Install the SimpleNetworkServer application on the network server
	NetworkServerHelper networkServerHelper;
	networkServerHelper.SetGateways (gateways);
	networkServerHelper.SetEndDevices (endDevices);
	ApplicationContainer serverContainer = networkServerHelper.Install (networkServers);

	mobilitySv.Install(networkServers);

	// Install the Forwarder application on the gateways
	ForwarderHelper forwarderHelper;
	forwarderHelper.Install (gateways);

	// Register the events
	for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
	{
	Ptr<Node> node = *j;
	Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
	Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
	phy->TraceConnectWithoutContext ("StartSending",
									 MakeCallback (&NsLoraSim::TransmissionCallback, this));
	}

	// Install reception paths on gateways
	for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); j++)
	{

	Ptr<Node> object = *j;
	// Get the device
	Ptr<NetDevice> netDevice = object->GetDevice (0);
	Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
	NS_ASSERT (loraNetDevice != 0);
	Ptr<GatewayLoraPhy> gwPhy = loraNetDevice->GetPhy ()->GetObject<GatewayLoraPhy> ();

	// Global callbacks (every gateway)
	gwPhy->TraceConnectWithoutContext ("ReceivedPacket",
									   MakeCallback (&NsLoraSim::PacketReceptionCallback, this));
	gwPhy->TraceConnectWithoutContext ("LostPacketBecauseInterference",
									   MakeCallback (&NsLoraSim::InterferenceCallback, this));
	gwPhy->TraceConnectWithoutContext ("LostPacketBecauseNoMoreReceivers",
									   MakeCallback (&NsLoraSim::NoMoreReceiversCallback, this));
	gwPhy->TraceConnectWithoutContext ("LostPacketBecauseUnderSensitivity",
									   MakeCallback (&NsLoraSim::UnderSensitivityCallback, this));
	}

	if (printdev)
	{
		std::ostringstream oss;
		oss << "dat/"<< mode <<"/endDevices-"<< nDevices <<"-"<< rRand <<"-r-" << nGateways  << "-p"<< appPeriodSeconds <<".dat";
		CreateMap (endDevices, gateways, networkServers, oss.str());
	}

	// Start simulation
	appContainer.Start (Seconds (0));
	appContainer.Stop (appStopTime);

	Simulator::Stop (appStopTime);
	Simulator::Run ();
	Simulator::Destroy ();

	Ptr<SimpleNetworkServer> aps = DynamicCast<SimpleNetworkServer>(serverContainer.Get(0));
	NS_ASSERT (aps != 0);
	double receivedProb = double(received)/nDevices;
	double interferedProb = double(interfered)/nDevices;
	double noMoreReceiversProb = double(noMoreReceivers)/nDevices;
	double underSensitivityProb = double(underSensitivity)/nDevices;

	double receivedProbGivenAboveSensitivity = double(received)/(nDevices - underSensitivity);
	double interferedProbGivenAboveSensitivity = double(interfered)/(nDevices - underSensitivity);
	double noMoreReceiversProbGivenAboveSensitivity = double(noMoreReceivers)/(nDevices - underSensitivity);

	std::ofstream fd;
	std::ostringstream oss;
	oss << "dat/"<< mode <<"/dat-" << nDevices << "-" << simulationTime  << "-r-" << nGateways  << "-p" << appPeriodSeconds  << ".csv";
	fd.open (oss.str(), std::ofstream::app);

	fd << rRand << ";" << nDevices << ";" << double(nDevices)/simulationTime << ";" << receivedProb << ";" << interferedProb << ";" << noMoreReceiversProb << ";" << underSensitivityProb <<
	";" << receivedProbGivenAboveSensitivity << ";" << interferedProbGivenAboveSensitivity << ";" << noMoreReceiversProbGivenAboveSensitivity << ";" << aps->GetAverageDelay() << std::endl;

	fd.close ();
}

int main (int argc, char *argv[])
{

  int verbose = 4;
  bool printdev = false;

  CommandLine cmd;
  cmd.AddValue ("verbose", "Whether to print output [1=ALL,2=DEBUG,3=INFO]", verbose);
  cmd.AddValue ("printdev", "Print devices' location or not", printdev);
  cmd.Parse (argc, argv);

  // Logging
  if (verbose == 1)
  {
	  LogComponentEnable ("NsLoraSim", LOG_LEVEL_ALL);
	  LogComponentEnable ("SimpleNetworkServer", LOG_LEVEL_ALL);
	  LogComponentEnable ("PointToPointNetDevice", LOG_LEVEL_ALL);
	  LogComponentEnable ("Forwarder", LOG_LEVEL_ALL);
	  LogComponentEnable ("DeviceStatus", LOG_LEVEL_ALL);
	  LogComponentEnable ("GatewayStatus", LOG_LEVEL_ALL);
  }
  else if (verbose == 2)
  {
	  LogComponentEnable ("NsLoraSim", LOG_LEVEL_DEBUG);
	  LogComponentEnable ("SimpleNetworkServer", LOG_LEVEL_DEBUG);
	  LogComponentEnable ("PeriodicSender", LOG_LEVEL_DEBUG);
	  LogComponentEnable ("PointToPointNetDevice", LOG_LEVEL_DEBUG);
  }
  else if (verbose == 3)
  {
	  LogComponentEnable ("NsLoraSim", LOG_LEVEL_INFO);
	  LogComponentEnable ("SimpleNetworkServer", LOG_LEVEL_INFO);
	  LogComponentEnable ("PeriodicSender", LOG_LEVEL_INFO);
	  LogComponentEnable ("PointToPointNetDevice", LOG_LEVEL_INFO);
  }
  LogComponentEnable ("NsLoraSim", LOG_LEVEL_INFO);
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnableAll (LOG_PREFIX_NODE);
  LogComponentEnableAll (LOG_PREFIX_TIME);

  // m_ndevice, m_nGateways, m_simulationTime, m_rand
  NsLoraSim sim1;
  NS_LOG_INFO ("vary gateways..");
  // ndevice increase
  for (int j=1; j<=7; j++)
  {
      // rRand
	  for (int i=1; i<=5; i++)
	  {
		  for (int k=1; k<=2; k++)
		  {
			  sim1 = NsLoraSim (125*j, k*k, 120.0, i);
			  NS_LOG_INFO (i << "-th iteration... (" << 250*j << ", r"<< k <<")");
			  sim1.Run ();
			  NS_LOG_INFO ("DONE");
		  }
	  }
  }
  NS_LOG_INFO ("vary datarate..");
  // m_nGateways, m_simulationTime, m_appPeriod, m_rand
  for (int j=1; j<=7; j++)
  {
	  // rRand
	  for (int k=1; k<=2; k++)
	  {
		  sim1 = NsLoraSim (4, 120.0, 10*j, k);
		  NS_LOG_INFO (k << "-th iteration... (" << 10*j << ", r"<< k <<")");
		  sim1.Run ();
		  NS_LOG_INFO ("DONE");
	  }
  }
  return 0;
}



