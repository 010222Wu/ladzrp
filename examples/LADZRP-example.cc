#include <fstream>
#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/ladzrp-module.h"
#include "ns3/applications-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;
using namespace dsr;

NS_LOG_COMPONENT_DEFINE("fanet-routings");

static std::map<uint32_t, uint32_t> hopCountMap;
static int dataPacket = 0;
static int controlPacket = 0;

class RoutingExperiment
{
public:
  RoutingExperiment();
  void Run();
  std::string CommandSetup(int argc, char **argv);
  // print arguments
  void PrintArguments();
  // print node positions
  void PrintPositions();
  // print routing tables
  void PrintRoutingTable();

private:
  Ptr<Socket> SetupPacketReceive(Ipv4Address addr, Ptr<Node> node);
  void ReceivePacket(Ptr<Socket> socket);

  // create the nodes
  void CreateNodes();
  // create the devices
  void CreateDevices();
  // create network
  void CreateInternetStacks();
  // create application
  void InstallApplications();

  // nodes used in this example
  NodeContainer adhocNodes;
  // netdevices used in this example
  NetDeviceContainer adhocDevices;
  // interfaces used in this example
  Ipv4InterfaceContainer adhocInterfaces;

  // client nodes
  std::vector<int> m_clients;
  // server nodes
  std::vector<int> m_servers;

  // intervals
  double m_interval;

  uint32_t port;
  uint32_t bytesTotal;
  uint32_t packetsReceived;

  std::string m_CSVfileName;
  int m_nSinks;
  std::string m_protocolName;
  double m_txp;
  uint32_t m_protocol;
  int m_nWifis;
  int m_totalTime;
  double startTime;
  // prefix for filenames
  std::string m_prefix;
  std::string phyMode;
  std::string tr_name;
  bool traceMetric;
  bool printRouting;
  // int packets per second
  int m_packets_per_second;
  std::string rate;
};

// 初始化变量
RoutingExperiment::RoutingExperiment()
    : port(9),
      bytesTotal(0),
      packetsReceived(0),

      m_CSVfileName("output.csv"),
      // Change
      m_nSinks(5),
      m_protocolName("protocol"),
      m_txp(18),

      m_protocol(3), // 1 olsr 2 aodv 3 dsdv 4 dsr
      // Change
      m_nWifis(10),
      m_totalTime(100),
      startTime(0.0),
      m_prefix("project-baseline"),
      phyMode("OfdmRate36Mbps"),
      // phyMode("DsssRate11Mbps"),
      tr_name("fanet-routings"),
      traceMetric(true),
      printRouting(true),
      // udp echo app setting
      m_packets_per_second(2),
      // on-off app setting
      rate("2048bps")

{
  // routing name
  switch (m_protocol)
  {
  case 1:
    m_protocolName = "OLSR";
    break;
  case 2:
    m_protocolName = "AODV";
    break;
  case 3:
    m_protocolName = "DSDV";
    break;
  case 4:
    m_protocolName = "DSR";
    break;
  default:
    NS_FATAL_ERROR("No such protocol:" << m_protocol);
  }
  m_CSVfileName = "fanet-" + m_protocolName + "-" + m_CSVfileName;
}

std::string
RoutingExperiment::CommandSetup(int argc, char **argv)
{
  CommandLine cmd(__FILE__);
  cmd.AddValue("protocol", "1=OLSR;2=AODV;3=DSDV;4=DSR", m_protocol);
  cmd.Parse(argc, argv);
  return m_CSVfileName;
}

void DevTxTrace (std::string context, Ptr<const Packet> p) {
  if (p->GetSize() > 1000) {
    dataPacket++;
  } else {
    controlPacket++;
  }
  // std::cout << "!Packet " << p->GetUid() << " size " << p->GetSize() << std::endl;
}

void RoutingExperiment::CreateNodes()
{
  // 集群的数目
  int numOfGroup = 5;
  int memberOfGroup = 10;
  
  NodeContainer nodeOfGroup;
  GroupMobilityHelper mobilityAdhoc;
  int64_t streamIndex = 0; // used to get consistent mobility across scenarios
  ObjectFactory parentPos;
  ObjectFactory childPos;

  parentPos.SetTypeId("ns3::RandomBoxPositionAllocator");
  parentPos.Set("X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
  parentPos.Set("Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
  parentPos.Set("Z", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=200.0]"));

  Ptr<PositionAllocator> taPositionAlloc1 = parentPos.Create()->GetObject<PositionAllocator>();
  streamIndex += taPositionAlloc1->AssignStreams(streamIndex);

  childPos.SetTypeId("ns3::RandomBoxPositionAllocator");
  childPos.Set("X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"));
  childPos.Set("Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"));
  childPos.Set("Z", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"));

  Ptr<PositionAllocator> taPositionAlloc2 = childPos.Create()->GetObject<PositionAllocator>();
  streamIndex += taPositionAlloc1->AssignStreams(streamIndex);

  for (int i = 0; i < numOfGroup; i++)
  {
    nodeOfGroup = NodeContainer();
    nodeOfGroup.Create(memberOfGroup);
    adhocNodes.Add(nodeOfGroup);

    mobilityAdhoc.SetReferenceMobilityModel("ns3::GaussMarkovMobilityModel",
                                            "Bounds", BoxValue(Box(0, 1000, 0, 1000, 0, 200)),
                                            "TimeStep", TimeValue(Seconds(0.5) ),
                                            "Alpha", DoubleValue(0.85),
                                            "MeanVelocity", StringValue("ns3::UniformRandomVariable[Min=10|Max=20]"),
                                            "MeanDirection", StringValue("ns3::UniformRandomVariable[Min=0|Max=6.283185307]"),
                                            "MeanPitch", StringValue("ns3::UniformRandomVariable[Min=0.05|Max=0.05]"),
                                            "NormalVelocity", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
                                            "NormalDirection", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.2|Bound=0.4]"),
                                            "NormalPitch", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.02|Bound=0.04]"));
    mobilityAdhoc.SetReferencePositionAllocator(taPositionAlloc1);

    mobilityAdhoc.SetMemberMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityAdhoc.SetMemberPositionAllocator(taPositionAlloc2);
    
    mobilityAdhoc.Install(nodeOfGroup);
  }

  streamIndex += mobilityAdhoc.AssignStreams(adhocNodes, streamIndex);

  NS_UNUSED(streamIndex); // From this point, streamIndex is unused

  std::cout << "nodes created\n";
}

void RoutingExperiment::CreateDevices()
{

  // setting up wifi phy and channel using helpers
  WifiHelper wifi;
  // wifi.SetStandard(WIFI_STANDARD_80211p);
  wifi.SetStandard (WIFI_STANDARD_80211n_5GHZ);

  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
  // wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(2000.0));
  // wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
  // wifiChannel.AddPropagationLoss("ns3::MatrixPropagationLossModel");
  wifiPhy.SetChannel(wifiChannel.Create());

  // Add a mac and disable rate control
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                               "DataMode", StringValue("HtMcs7"),
                               "ControlMode", StringValue("HtMcs0"));
  // wifi gonglv
  wifiPhy.Set("TxPowerStart", DoubleValue(m_txp));
  wifiPhy.Set("TxPowerEnd", DoubleValue(m_txp));

  wifiMac.SetType("ns3::AdhocWifiMac");
  adhocDevices = wifi.Install(wifiPhy, wifiMac, adhocNodes);

  AsciiTraceHelper ascii;
  wifiPhy.EnableAsciiAll(ascii.CreateFileStream(m_prefix + "-" + m_protocolName + "-phy.tr"));

  std::cout << "devices created\n";
}

// ADD
uint64_t preUid = -1;
static void 
MyCallback(std::string context,Ptr<const Packet> packet,const Address& from,const Address& to) {
  if (packet->GetUid() != preUid) {
    preUid = packet->GetUid();
    std::cout << "Packet " << packet->GetUid() <<" sent at " << Simulator::Now().GetSeconds() << " seconds from node " << InetSocketAddress::ConvertFrom(from).GetIpv4() << " to node " << InetSocketAddress::ConvertFrom(to).GetIpv4() << std::endl;
  }
}

void RoutingExperiment::CreateInternetStacks()
{

  AodvHelper aodv;
  OlsrHelper olsr;
  LADZRPHelper dsdv;
  DsrHelper dsr;
  DsrMainHelper dsrMain;
  Ipv4ListRoutingHelper list;
  InternetStackHelper internet;

  switch (m_protocol)
  {
  case 1:
    list.Add(olsr, 100);
    break;
  case 2:
    list.Add(aodv, 100);
    break;
  case 3:
    list.Add(dsdv, 100);
    break;
  default:
    break;
  }

  if (m_protocol < 4)
  {
    internet.SetRoutingHelper(list);
    internet.Install(adhocNodes);
  }
  else if (m_protocol == 4)
  {
    internet.Install(adhocNodes);
    dsrMain.Install(dsr, adhocNodes);
  }

  NS_LOG_INFO("assigning ip address");

  Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase("10.1.1.0", "255.255.255.0");
  adhocInterfaces = addressAdhoc.Assign(adhocDevices);

  AsciiTraceHelper ascii;
  internet.EnableAsciiIpv4All(ascii.CreateFileStream(m_prefix + "-" + m_protocolName + "-ipv4.tr"));

  if (printRouting)
  {
    Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>((tr_name + "-" + m_protocolName + ".routes"), std::ios::out);
    switch (m_protocol)
    {
    case 1:
      olsr.PrintRoutingTableAllAt(Seconds(10), routingStream);
      break;
    case 2:
      aodv.PrintRoutingTableAllAt(Seconds(10), routingStream);
      break;
    case 3:
      dsdv.PrintRoutingTableAllAt(Seconds(10), routingStream);
      break;
    }
  }
}

void RoutingExperiment::InstallApplications()
{

  UdpEchoServerHelper echoServer(9);

  int coupleNum = 4;

  m_clients.reserve(coupleNum);
  m_servers.reserve(coupleNum);
  
  m_servers[0] = 1;
  m_servers[1] = 2;
  m_servers[2] = 3;
  m_servers[3] = 4;

  m_clients[0] = 15;
  m_clients[1] = 25;
  m_clients[2] = 35;
  m_clients[3] = 45;
  
 
  m_interval = 1.0 / m_packets_per_second;

  std::cout << "Setting up flows " << std::endl;

  OnOffHelper onoff("ns3::UdpSocketFactory", Address());
  onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
  Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("1000"));
  Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue("20000000bps"));
  for (int i = 0; i < coupleNum; i++)
  {
    Ptr<Socket> sink = SetupPacketReceive(adhocInterfaces.GetAddress(m_clients[i]), adhocNodes.Get(m_clients[i]));
    AddressValue remoteAddress(InetSocketAddress(adhocInterfaces.GetAddress(m_clients[i]), port));
    onoff.SetAttribute("Remote", remoteAddress);
    Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable>();
    ApplicationContainer adhocApps = onoff.Install(adhocNodes.Get(m_servers[i]));
    // begin time?
    Config::Connect("NodeList/*/ApplicationList/0/$ns3::OnOffApplication/TxWithAddresses",MakeCallback(&MyCallback));
    
    adhocApps.Start(Seconds(var->GetValue(startTime, startTime + 1.0)));
    adhocApps.Stop(Seconds(m_totalTime));
    std::cout << adhocInterfaces.GetAddress(m_servers[i]) << " ---> " << adhocInterfaces.GetAddress(m_clients[i]) << std::endl;
  }

  // ADD
  Config::Connect("/NodeList/*/DeviceList/*/Mac/MacRx", MakeCallback(&DevTxTrace));

  // for (int i = 0; i < m_nSinks; i++)
  //   {
  //     Ptr<Socket> sink = SetupPacketReceive(adhocInterfaces.GetAddress(i), adhocNodes.Get(i));
  //     Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable>();
  //     ApplicationContainer serverApps = echoServer.Install(adhocNodes.Get(m_servers[i]));
  //     serverApps.Start(Seconds(var->GetValue(startTime, startTime + 1.0)));
  //     serverApps.Stop(Seconds(m_totalTime));
  //     // ADD UdpTraceClient
  //     UdpTraceClientHelper traceClient(adhocInterfaces.GetAddress(m_servers[i]), port, "0-output1.dat");

  //     // UdpEchoClientHelper echoClient(adhocInterfaces.GetAddress(m_servers[i]), port);
  //     // echoClient.SetAttribute("MaxPackets", UintegerValue(10000));
  //     // echoClient.SetAttribute("Interval", TimeValue(Seconds(m_interval)));
  //     // echoClient.SetAttribute("PacketSize", UintegerValue(64));
  //     ApplicationContainer clientApps = traceClient.Install(adhocNodes.Get(m_clients[i]));
  //     clientApps.Start(Seconds(var->GetValue(startTime, startTime + 1.0)));
  //     clientApps.Stop(Seconds(m_totalTime));
  //     std::cout << adhocInterfaces.GetAddress(m_clients[i]) << " ---> " << adhocInterfaces.GetAddress(m_servers[i]) << std::endl;
  //   }
}

void RoutingExperiment::PrintPositions()
{
  std::cout << "Time: " << Simulator::Now().GetSeconds() << "\n";
  for (auto nodeIt = adhocNodes.Begin (); nodeIt != adhocNodes.End (); ++nodeIt)
  {
    Ptr<MobilityModel> model = (*nodeIt)->GetObject<MobilityModel> ();
    std::cout << (*nodeIt)->GetId () << " " << model->GetPosition () << std::endl;
  }
  Simulator::Schedule(Seconds(1.0), &RoutingExperiment::PrintPositions, this);
}

void RoutingExperiment::PrintArguments()
{
  std::cout << "Simulation arguments:\n";
  std::cout << "nodes: " << m_nWifis << "\n";
  std::cout << "flows: " << m_nSinks << "\n";
  std::cout << "routing: " << m_protocolName << "\n";
  std::cout << "time: " << m_totalTime << "\n";
  std::cout << "pps: " << m_packets_per_second << "\n";
  std::cout << "--------------------------\n";
}

static inline std::string
PrintReceivedPacket(Ptr<Socket> socket, Ptr<Packet> packet, Address senderAddress)
{
  std::ostringstream oss;

  oss << Simulator::Now().GetSeconds() << " " << socket->GetNode()->GetId() + 1;

  if (InetSocketAddress::IsMatchingType(senderAddress))
  {
    InetSocketAddress addr = InetSocketAddress::ConvertFrom(senderAddress);
    oss << " received one packet from " << addr.GetIpv4();

    // ADD
    std::cout << Simulator::Now().GetSeconds() << " " << socket->GetNode()->GetId() + 1 << " received one packet from " << addr.GetIpv4()
              << " Packet " << packet->GetUid() << std::endl;
  }
  else
  {
    oss << " received one packet!";
  }
  return oss.str();
}

void RoutingExperiment::ReceivePacket(Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address senderAddress;
  while ((packet = socket->RecvFrom(senderAddress)))
  {
    bytesTotal += packet->GetSize();
    packetsReceived += 1;
    NS_LOG_UNCOND(PrintReceivedPacket(socket, packet, senderAddress));
  }
}

Ptr<Socket>
RoutingExperiment::SetupPacketReceive(Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket(node, tid);
  InetSocketAddress local = InetSocketAddress(addr, port);
  sink->Bind(local);
  sink->SetRecvCallback(MakeCallback(&RoutingExperiment::ReceivePacket, this));

  return sink;
}

void RoutingExperiment::Run()
{

  PrintArguments();

  Packet::EnablePrinting();

  // Set Non-unicastMode rate to unicast mode
  Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(phyMode));

  std::cout << "Setting networks...\n";
  CreateNodes();
  CreateDevices();
  CreateInternetStacks();
  InstallApplications();

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  NS_LOG_INFO("Running simulations...");

  // 打印位置
  // PrintPositions();

  Simulator::Stop(Seconds(m_totalTime));

  std::ofstream positionPrint("project-baseline-node-positions.txt");
  positionPrint.close();

  Simulator::Run();

  // flowmon->SerializeToXmlFile ((tr_name + ".flowmon").c_str(), false, false);
  if (traceMetric)
  {
    int j = 0;
    float TotalThroughput = 0;
    float AvgThroughput = 0;
    Time Jitter;
    Time Delay;
    uint32_t SentPackets = 0;
    uint32_t ReceivedPackets = 0;
    uint32_t LostPackets = 0;

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    std::ofstream flowOut("TaskB/" + m_prefix + "-" + m_protocolName + "-flowstats");

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin(); iter != stats.end(); ++iter)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(iter->first);
      if ((t.sourceAddress == "10.1.1.2" && t.destinationAddress == "10.1.1.16") ||
          (t.sourceAddress == "10.1.1.3" && t.destinationAddress == "10.1.1.26") ||
          (t.sourceAddress == "10.1.1.4" && t.destinationAddress == "10.1.1.36") ||
          (t.sourceAddress == "10.1.1.5" && t.destinationAddress == "10.1.1.46")
        ){

      NS_LOG_UNCOND("----Flow ID:" << iter->first);
      NS_LOG_UNCOND("Src Addr " << t.sourceAddress << " Dst Addr " << t.destinationAddress);
      NS_LOG_UNCOND("Sent Packets=" << iter->second.txPackets);
      NS_LOG_UNCOND("Received Packets =" << iter->second.rxPackets);
      NS_LOG_UNCOND("Lost Packets =" << iter->second.txPackets - iter->second.rxPackets);
      NS_LOG_UNCOND("Packet delivery ratio =" << iter->second.rxPackets * 100 / iter->second.txPackets << "%");
      NS_LOG_UNCOND("Packet loss ratio =" << (iter->second.txPackets - iter->second.rxPackets) * 100 / iter->second.txPackets << "%");
      NS_LOG_UNCOND("Delay =" << iter->second.delaySum);
      NS_LOG_UNCOND("Jitter =" << iter->second.jitterSum);
      NS_LOG_UNCOND("Throughput =" << iter->second.rxBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds()) / 1024 << "Kbps");
      // NS_LOG_UNCOND("RxBytes =" << iter->second.rxBytes * 8.0 << " Time =" << (iter->second.timeLastRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds()));


      // std::cout << "----Flow ID:" << iter->first << std::endl;
      // std::cout << "Src Addr" <<t.sourceAddress << "Dst Addr "<< t.destinationAddress << std::endl;
      // std::cout << "Sent Packets=" <<iter->second.txPackets << std::endl;
      // std::cout << "Received Packets =" <<iter->second.rxPackets << std::endl;
      // std::cout << "Lost Packets =" <<iter->second.txPackets-iter->second.rxPackets << std::endl;
      // std::cout << "Packet delivery ratio =" <<iter->second.rxPackets*100/iter->second.txPackets << "%" << std::endl;
      // std::cout << "Packet loss ratio =" << (iter->second.txPackets-iter->second.rxPackets)*100/iter->second.txPackets << "%" << std::endl;
      // std::cout << "Delay =" <<iter->second.delaySum << std::endl;
      // std::cout << "Jitter =" <<iter->second.jitterSum << std::endl;
      // std::cout << "Throughput =" <<iter->second.rxBytes * 8.0/(iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds())/1024<<"Kbps" << std::endl;

      SentPackets = SentPackets + (iter->second.txPackets);
      ReceivedPackets = ReceivedPackets + (iter->second.rxPackets);
      LostPackets = LostPackets + (iter->second.txPackets - iter->second.rxPackets);
      AvgThroughput = AvgThroughput + (iter->second.rxBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds()) / 1024);
      Delay = Delay + (iter->second.delaySum);
      Jitter = Jitter + (iter->second.jitterSum);

      j = j + 1;
          }
    }

    flowOut << "--------Total Results of the simulation----------" << std::endl
            << "\n";
    flowOut << "Nodes: " << m_nWifis << "\n";
    flowOut << "Flows: " << m_nSinks << "\n";
    flowOut << "pps: " << m_packets_per_second << "\n";
    flowOut << "Total Received Packets = " << ReceivedPackets << "\n";
    flowOut << "Total sent packets  = " << SentPackets << "\n";
    flowOut << "Total Lost Packets = " << LostPackets << "\n";
    flowOut << "Packet Loss ratio = " << ((LostPackets * 100.00) / SentPackets) << " %"
            << "\n";
    flowOut << "Packet delivery ratio = " << ((ReceivedPackets * 100.00) / SentPackets) << " %"
            << "\n";
    flowOut << "Average Throughput = " << AvgThroughput << " Kbps"
            << "\n";
    flowOut << "End to End Delay = " << Delay << "\n";
    flowOut << "Total Flow id " << j << "\n";

    TotalThroughput = AvgThroughput;
    AvgThroughput = AvgThroughput / j;
    NS_LOG_UNCOND("--------Total Results of the simulation----------" << std::endl);
    NS_LOG_UNCOND("Total sent packets  =" << SentPackets);
    NS_LOG_UNCOND("Total Received Packets =" << ReceivedPackets);
    NS_LOG_UNCOND("Total Lost Packets =" << LostPackets);
    NS_LOG_UNCOND("Packet Loss ratio =" << ((LostPackets * 100) / (SentPackets + 0.001)) << "%");
    NS_LOG_UNCOND("Packet delivery ratio =" << ((ReceivedPackets * 100) / (SentPackets + 0.001)) << "%");
    NS_LOG_UNCOND("Average Throughput =" << AvgThroughput << "Kbps");
    NS_LOG_UNCOND("Total Throughput =" << TotalThroughput << "Kbps");
    NS_LOG_UNCOND("End to End Delay =" << Delay);
    NS_LOG_UNCOND("End to End Jitter delay =" << Jitter);
    NS_LOG_UNCOND("Total Flow id " << j);
    monitor->SerializeToXmlFile("fanet-routing-" + m_protocolName + ".xml", true, true);

    flowOut.close();
  }

  Simulator::Destroy();
}

int main(int argc, char *argv[])
{
  RngSeedManager::SetSeed(1500);
  RngSeedManager::SetRun(1500);

  RoutingExperiment experiment;
  std::string CSVfileName = experiment.CommandSetup(argc, argv);

  // blank out the last output file and write the column headers
  std::ofstream out(CSVfileName.c_str());
  out << "SimulationSecond,"
      << "ReceiveRate,"
      << "PacketsReceived,"
      << "NumberOfSinks,"
      << "RoutingProtocol,"
      << "TransmissionPower" << std::endl;
  out.close();


  experiment.Run();
  std::cout << "Overhead: " << controlPacket / dataPacket << "Control Packet: " << controlPacket << " Data Packet: " << dataPacket << std::endl;
  std::cout << "AvgDistance: " << ns3::ladzrp::RoutingProtocol::dis / ns3::ladzrp::RoutingProtocol::discount << 
               " Dis: " << ns3::ladzrp::RoutingProtocol::dis << " DisCount: " << ns3::ladzrp::RoutingProtocol::discount << std::endl;
}