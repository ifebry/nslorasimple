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
#include "ns3/end-device-lora-mac.h"
#include <string.h>
#include <math.h>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NsLoraSim");

class NsLoraSim {
public:
	NsLoraSim ();
	NsLoraSim (int, uint8_t, double, double, uint64_t);
	NsLoraSim (int, double, double, uint64_t);
	NsLoraSim (int, double, uint8_t, uint64_t);
	~NsLoraSim ();
	void Run (void);
	double GetPDR (void);
	double GetDelay (void);
	int GetGW (void);
	int GetReceived (void);
	float GetNoMoreRcvProb (void);
	float GetInterferedProb (void);
private:
	int nDevices;
	uint8_t gatewayRings;
	int nGateways = 1;
	double radius;
	double gatewayRadius;
	double simulationTime;
	uint8_t appPeriodSeconds;

	int noMoreReceivers = 0;
	int interfered = 0;
	int received = 0;
	int underSensitivity = 0;

	uint16_t txPkt = 0;
	uint16_t rxPkt = 0;
	uint64_t rRand = 0;

	bool printdev;
	int mode = 0;

	double arWidth = 10000.0;
	double edInterval = 500.0;
	double gwInterval = 2000.0;

	double pdr = 100.0;
	double e2edelay = 0.0;

	PeriodicSendPktCallback sendcallback;

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
	std::map<uint64_t, Time> timeTracker;

	void CheckReceptionByAllGWsComplete (std::map<Ptr<Packet const>, PacketStatus>::iterator );
	void TransmissionCallback (Ptr<Packet const>, uint32_t );
	void PacketReceptionCallback (Ptr<Packet const> , uint32_t );
	void InterferenceCallback (Ptr<Packet const> , uint32_t );
	void NoMoreReceiversCallback (Ptr<Packet const> , uint32_t );
	void UnderSensitivityCallback (Ptr<Packet const> , uint32_t );
	void CreateMap (NodeContainer , NodeContainer , NodeContainer , std::string );
	void TransmitPacketCallback (uint64_t, Time);
	void ServerRcvCallback (uint64_t, Time);
	void FailedTxCallback (Ptr<Packet const>);
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

NsLoraSim::NsLoraSim (int m_devices, double m_gwInterval, double m_simulationTime, uint64_t m_rand) :
		gatewayRings (2),
		radius (5000),
		gatewayRadius (3500),
		simulationTime (100.0),
		appPeriodSeconds (15),
		printdev (true)
{
	gwInterval = m_gwInterval;
	uint8_t div = arWidth / gwInterval - 1;
	nGateways = std::pow (div, 2.0);

	nDevices = m_devices;
	rRand = m_rand;
    simulationTime = m_simulationTime;

    mode = 1312;
    NS_LOG_DEBUG ("nDev:" << std::to_string(nDevices) << " nGw:" << std::to_string(nGateways));
}

NsLoraSim::NsLoraSim (int m_devices, double m_gwInterval, uint8_t m_appPeriod, uint64_t m_rand) :
		nDevices (100),
		gatewayRings (2),
		radius (7500),
		gatewayRadius (3500),
		simulationTime (120.0),
		appPeriodSeconds (10),
		printdev (true)
{
	gwInterval = m_gwInterval;
	uint8_t div = arWidth / gwInterval - 1;
	nGateways = std::pow (div, 2.0);

	appPeriodSeconds = m_appPeriod;
	rRand = m_rand;

	nDevices = m_devices;
	mode = 1312;
	NS_LOG_DEBUG ("per: "<< std::to_string(appPeriodSeconds) << " nDev:" << std::to_string(nDevices) << " nGw:" << std::to_string(nGateways));
}

NsLoraSim::~NsLoraSim()
{
	NS_LOG_INFO ("finishing simulation...");
	packetTracker.clear();
	timeTracker.clear();
}

double
NsLoraSim::GetPDR ()
{
	return pdr;
}

int
NsLoraSim::GetReceived ()
{
	return received;
}

double
NsLoraSim::GetDelay ()
{
	return e2edelay/(double)received;
}

int
NsLoraSim::GetGW ()
{
	return nGateways;
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
//   NS_LOG_DEBUG ("Tx: " << systemId);
  // Create a packetStatus
  PacketStatus status;
  status.packet = packet;
  status.senderId = systemId;
  status.outcomeNumber = 0;
  status.outcomes = std::vector<enum PacketOutcome> (nGateways, UNSET);

  txPkt++;
  packetTracker.insert (std::pair<Ptr<Packet const>, PacketStatus> (packet, status));
}

void
NsLoraSim::PacketReceptionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
//  NS_LOG_DEBUG ("!");
  // NS_LOG_DEBUG ("A packet was successfully received at server " << systemId << Simulator::Now().GetMilliSeconds() << " : " << std::to_string(tag.GetSendtime()));
  std::map<Ptr<Packet const>, PacketStatus>::iterator it = packetTracker.find (packet);
  (*it).second.outcomes.at (systemId - nDevices) = RECEIVED;
  (*it).second.outcomeNumber += 1;

  // Remove the successfully received packet from the list of sent ones
  CheckReceptionByAllGWsComplete (it);
}

void
NsLoraSim::InterferenceCallback (Ptr<Packet const> packet, uint32_t systemId)
{
	// NS_LOG_INFO ("A packet was interferenced " << systemId);

	std::map<Ptr<Packet const>, PacketStatus>::iterator it = packetTracker.find (packet);
	it->second.outcomes.at (systemId - nDevices) = INTERFERED;
	it->second.outcomeNumber += 1;

	CheckReceptionByAllGWsComplete (it);
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
	oss << "dat/"<< mode <<"/gw-"<< nDevices <<"-"<< rRand <<"-r-"<< nGateways  <<"-p"<< std::to_string(appPeriodSeconds) <<".dat";
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
	oss << "dat/"<< mode <<"/srv-"<< nDevices <<"-"<< rRand <<"-r-"<< nGateways  <<"-p"<< std::to_string(appPeriodSeconds) <<".dat";
	fd.open (oss.str());
	Ptr<MobilityModel> position = svr.Get(0)->GetObject<MobilityModel>();
	NS_ASSERT (position != 0);

	Vector pos = position->GetPosition ();
	fd << pos.x << " " << pos.y << " 7" << std::endl;
	fd.close();
	oss.clear ();
}

float
NsLoraSim::GetNoMoreRcvProb (void)
{
	return float(noMoreReceivers)/nDevices;
}

float
NsLoraSim::GetInterferedProb (void)
{
	return float(interfered)/nDevices;
}

void
NsLoraSim::TransmitPacketCallback (uint64_t pid, Time t)
{
//	NS_LOG_DEBUG (std::to_string(pid) << " " << t.GetMilliSeconds());
	timeTracker.insert(std::pair<uint64_t, Time>(pid, t));
}

void
NsLoraSim::ServerRcvCallback (uint64_t pid, Time t)
{
	// Get transmission time for this packet
	std::map<uint64_t, Time>::iterator it2 = timeTracker.find(pid);

	e2edelay =  e2edelay + Simulator::Now().GetMilliSeconds() - it2->second.GetMilliSeconds();
	NS_LOG_INFO (std::to_string(e2edelay));
	rxPkt++;
}

void
NsLoraSim::FailedTxCallback (Ptr<Packet const> packet)
{
	NS_LOG_DEBUG (" f: " << packet->GetUid());
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
	mobilityEd.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
								   	 "X", StringValue ("ns3::UniformRandomVariable[Min=-5000|Max=5000]"),
									 "Y", StringValue ("ns3::UniformRandomVariable[Min=-5000|Max=5000]"));
	mobilityEd.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

	// Gateway mobility
	mobilityGw.SetPositionAllocator("ns3::GridPositionAllocator",
							   "MinX", DoubleValue (-1 * (arWidth/2) + (gwInterval)),
							   "MinY", DoubleValue (-1 * (arWidth/2) + (gwInterval)),
							   "DeltaX", DoubleValue (gwInterval),
							   "DeltaY", DoubleValue (gwInterval),
							   "GridWidth", UintegerValue (arWidth/gwInterval - 1),
							   "LayoutType", StringValue ("RowFirst"));
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
		Ptr<EndDeviceLoraMac> mac = loraNetDevice->GetMac()->GetObject<EndDeviceLoraMac>();
		mac->TraceConnectWithoutContext("CannotSendBecauseDutyCycle",
										MakeCallback (&NsLoraSim::FailedTxCallback, this));
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

	for (ApplicationContainer::Iterator j = appContainer.Begin(); j != appContainer.End (); j++)
	{
		Ptr<PeriodicSender> ps = DynamicCast<PeriodicSender>(*j);
		ps->SetPeriodicSendPktCallback(MakeCallback(&NsLoraSim::TransmitPacketCallback, this));
	}

	if (printdev)
	{
		std::ostringstream oss;
		oss << "dat/"<< mode <<"/endDevices-"<< nDevices <<"-"<< rRand <<"-r-" << nGateways  << "-p"<< std::to_string(appPeriodSeconds) <<".dat";
		CreateMap (endDevices, gateways, networkServers, oss.str());
	}

	Ptr<SimpleNetworkServer> aps = DynamicCast<SimpleNetworkServer>(serverContainer.Get(0));
	aps->SetServerRcvCallback(MakeCallback(&NsLoraSim::ServerRcvCallback, this));

	// Start simulation
	appContainer.Start (Seconds (0));
	appContainer.Stop (appStopTime);

	Simulator::Stop (appStopTime);
	Simulator::Run ();
	Simulator::Destroy ();


	Ptr<PeriodicSender> ps = DynamicCast<PeriodicSender>(appContainer.Get(0));
	NS_ASSERT (aps != 0);

	double receivedProb = double(received)/nDevices;					// 3
	double interferedProb = double(interfered)/nDevices;				// 4
	double noMoreReceiversProb = double(noMoreReceivers)/nDevices;		// 5
	double underSensitivityProb = double(underSensitivity)/nDevices;	// 6

	double receivedProbGivenAboveSensitivity = double(received)/(nDevices - underSensitivity);	// 7
	double interferedProbGivenAboveSensitivity = double(interfered)/(nDevices - underSensitivity);	// 8
	double noMoreReceiversProbGivenAboveSensitivity = double(noMoreReceivers)/(nDevices - underSensitivity); // 9

	pdr = receivedProb;
	std::ofstream fd;
	std::ostringstream oss;
	oss << "dat/"<< mode <<"/dat-" << nDevices << "-" << simulationTime  << "-r-" << nGateways  << "-p" << std::to_string(appPeriodSeconds)  << ".csv";
	fd.open (oss.str(), std::ofstream::app);

	fd << rRand << " " \
	   << nDevices << " "
	   << double(nDevices)/simulationTime << " " \
	   << receivedProb << " " \
	   << interferedProb << " " \
	   << noMoreReceiversProb << " " \
	   << underSensitivityProb << " " \
	   << receivedProbGivenAboveSensitivity << " " \
	   << interferedProbGivenAboveSensitivity << " " \
	   << noMoreReceiversProbGivenAboveSensitivity << " " \
	   << aps->GetAverageDelay() << " " \
	   << aps->GetTotalPkts() << " " \
	   << std::to_string (e2edelay / received) << " " \
	   << std::to_string (received) << " " \
	   << std::endl;

	fd.close ();
	NS_LOG_DEBUG (" rx/tx: " << (float)rxPkt/(float)txPkt << "_" << std::to_string(rxPkt) << " " << std::to_string(txPkt) << "rceived: " << std::to_string(received));

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
  }
  else if (verbose == 3)
  {
	  LogComponentEnable ("NsLoraSim", LOG_LEVEL_INFO);
	  LogComponentEnable ("SimpleNetworkServer", LOG_LEVEL_INFO);
	  LogComponentEnable ("PeriodicSender", LOG_LEVEL_INFO);
	  LogComponentEnable ("PointToPointNetDevice", LOG_LEVEL_INFO);
  }
  LogComponentEnable ("NsLoraSim", LOG_LEVEL_DEBUG);
//  LogComponentEnable ("PeriodicSender", LOG_LEVEL_ALL);
//  LogComponentEnable ("LoraInterferenceHelper", LOG_LEVEL_DEBUG);
//  LogComponentEnable ("EndDeviceLoraPhy", LOG_LEVEL_ALL);
//  LogComponentEnable ("EndDeviceLoraMac", LOG_LEVEL_ALL);
//  LogComponentEnable ("LoraMac", LOG_LEVEL_ALL);
//  LogComponentEnable ("DeviceStatus", LOG_LEVEL_ALL);
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnableAll (LOG_PREFIX_NODE);
  LogComponentEnableAll (LOG_PREFIX_TIME);

  std::ofstream ofs;
  std::ostringstream oss;
  oss << "dat/1312/dat-n200-t120-gw.csv";
  ofs.open(oss.str());
  double avgpdr = 0.0;
  double avgdelay = 0.0;
  float avginterfered = 0.0;
  float avgnomorercv = 0.0;
  int initDev, incDev, r;
  initDev = 200; incDev = 100; r = 1;
  double arr[6] = { 1250.0, 1400.0, 1500.0, 1750.0, 2000.0, 2500.0 };


   /* FIRST SCENARIO */


  NsLoraSim sim1;
//  for (int intGw = 0; intGw < 6; intGw++)
//  {
//	  for (int ndev=0; ndev <= 4; ndev++)
//	  {
//		  for (int j=1; j<=r; j++)
//		  {
//			  sim1 = NsLoraSim (initDev + ndev*incDev, arr[intGw] , 120.0, j*j);
//			  sim1.Run ();
//			  avgpdr += sim1.GetPDR ();
//			  avgdelay += sim1.GetDelay ();
//			  avginterfered += sim1.GetInterferedProb();
//			  avgnomorercv += sim1.GetNoMoreRcvProb();
//		  }
//
//		  NS_LOG_DEBUG (std::to_string(initDev + ndev*incDev) << " " << std::to_string(sim1.GetGW()) << \
//				  	  " " << std::to_string(arr[intGw]) << " PDR: " << std::to_string(avgpdr/r) << \
//					  " delay: " << std::to_string(avgdelay/r) << " interfered:" << std::to_string(avginterfered/r) << \
//					  " nomore: " << std::to_string(avgnomorercv/r));
//
//		  ofs << std::to_string(initDev + ndev*incDev) << " " \
//				  << std::to_string(sim1.GetGW()) << " " \
//				  << std::to_string(arr[intGw]) << " " \
//				  << std::to_string(avgpdr/r) << " " \
//				  << std::to_string(avgdelay/r) << " " \
//				  << std::to_string(avginterfered/r) << " " \
//				  << std::to_string(avgnomorercv/r) << std::endl;
//
//		  avgpdr = 0.0;
//		  avgdelay = 0.0;
//		  avginterfered = 0.0;
//		  avgnomorercv = 0.0;
//	  }
//  }
//  ofs.close();
//  oss.flush();

   /*  SECOND SCENARIO */

//  std::ostringstream oss2;
//  oss2 << "dat/1312/dat-n600-gw9-period.csv";
//  ofs.open(oss2.str());
//  avgdelay = avgpdr = avginterfered = avgnomorercv = 0.0;
//  uint8_t period[5] = { 15, 20, 30, 45, 60 };
//
//  for (int intGw=0; intGw<6; intGw++)
//  {
//	  for (uint8_t intper=0; intper<5; intper++)
//	  {
//		  for (int j=1; j<=r; j++)
//		  {
//			  sim1 = NsLoraSim (120, arr[intGw], period[intper], j*j);
//			  sim1.Run ();
//			  avgpdr += sim1.GetPDR ();
//			  avgdelay += sim1.GetDelay ();
//			  avginterfered += sim1.GetInterferedProb();
//			  avgnomorercv += sim1.GetNoMoreRcvProb();
//		  }
//
//		  NS_LOG_DEBUG (std::to_string(period[intper]) << " " << std::to_string(sim1.GetGW()) << \
//		  				  	  " " << std::to_string(arr[intGw]) << " PDR: " << std::to_string(avgpdr/r) << \
//		  					  " delay: " << std::to_string(avgdelay/r) << " interfered:" << std::to_string(avginterfered/r) << \
//		  					  " nomore: " << std::to_string(avgnomorercv/r));
//
//		  ofs << std::to_string(period[intper]) << " " \
//				  << std::to_string(sim1.GetGW()) << " " \
//				  << std::to_string(arr[intGw]) << " " \
//				  << std::to_string(avgpdr/r) << " " \
//				  << std::to_string(avgdelay/r) << " " \
//				  << std::to_string(avginterfered/r) << " " \
//				  << std::to_string(avgnomorercv/r) << std::endl;
//
//		  avgpdr = 0.0;
//		  avgdelay = 0.0;
//		  avginterfered = 0.0;
//		  avgnomorercv = 0.0;
//	  }
//  }
//  ofs.close();
//  oss2.flush();
//
//  oss.clear();
//  oss2.clear();

  int r = 1;
  uint8_t p = 30;
  NsLoraSim sim1 = NsLoraSim (100, 2500.0, p, r);
  sim1.Run ();
  NS_LOG_DEBUG (std::to_string(sim1.GetDelay ()) << ":" << std::to_string(sim1.GetPDR ()));

  p = 10;
  sim1 = NsLoraSim (100, 2500.0, p, r);
  sim1.Run ();
  NS_LOG_DEBUG (std::to_string(sim1.GetDelay ()) << ":" << std::to_string(sim1.GetPDR ()));

  return 0;
}
