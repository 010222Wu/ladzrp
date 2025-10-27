#include "ladzrp-routing-protocol.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/integer.h"
#include "ns3/uinteger.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/simulator.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/vector.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace ns3 {
namespace ladzrp {

NS_LOG_COMPONENT_DEFINE ("LadzrpRoutingProtocol");

// 中文说明：该实现文件涵盖LA-DZRP协议的完整流程，包括两级位置维护
// (TTLM)与多阶段转发(MPF)的每个细节。

const uint32_t RoutingProtocol::LADZRP_PORT = 269;

NS_OBJECT_ENSURE_REGISTERED (RoutingProtocol);

TypeId
RoutingProtocol::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ladzrp::RoutingProtocol")
    .SetParent<Ipv4RoutingProtocol> ()
    .SetGroupName ("Ladzrp")
    .AddConstructor<RoutingProtocol> ()
    .AddAttribute ("HelloInterval",
                   "Period for periodic HELLO beacons.",
                   TimeValue (Seconds (1.0)),
                   MakeTimeAccessor (&RoutingProtocol::m_helloInterval),
                   MakeTimeChecker ())
    .AddAttribute ("GlobalUpdateInterval",
                   "Interval between deviation checks for global updates.",
                   TimeValue (Seconds (5.0)),
                   MakeTimeAccessor (&RoutingProtocol::m_globalUpdateInterval),
                   MakeTimeChecker ())
    .AddAttribute ("ZoneRadiusMid",
                   "Mid-tier zone radius in hops.",
                   UintegerValue (2),
                   MakeUintegerAccessor (&RoutingProtocol::m_zoneRadiusMid),
                   MakeUintegerChecker<uint8_t> (1))
    .AddAttribute ("ZoneRadiusLow",
                   "Low-tier zone radius in hops.",
                   UintegerValue (1),
                   MakeUintegerAccessor (&RoutingProtocol::m_zoneRadiusLow),
                   MakeUintegerChecker<uint8_t> (1))
    .AddAttribute ("ZoneRadiusHigh",
                   "High-tier zone radius in hops.",
                   UintegerValue (3),
                   MakeUintegerAccessor (&RoutingProtocol::m_zoneRadiusHigh),
                   MakeUintegerChecker<uint8_t> (1))
    .AddAttribute ("BeaconRate",
                   "HELLO beaconing rate in Hertz.",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&RoutingProtocol::m_beaconRate),
                   MakeDoubleChecker<double> (0.1))
    .AddAttribute ("BeaconPayloadSize",
                   "Payload size used to evaluate control-plane budget (bytes).",
                   UintegerValue (48),
                   MakeUintegerAccessor (&RoutingProtocol::m_beaconPayloadSize),
                   MakeUintegerChecker<uint32_t> (16))
    .AddAttribute ("BandwidthBudget",
                   "Available control-plane bandwidth per node (Bytes/s).",
                   DoubleValue (1024.0),
                   MakeDoubleAccessor (&RoutingProtocol::m_bandwidthBudget),
                   MakeDoubleChecker<double> (1.0))
    .AddAttribute ("PreExpiryAdvance",
                   "Time before expiry when a unicast refresh should be requested.",
                   TimeValue (Seconds (1.0)),
                   MakeTimeAccessor (&RoutingProtocol::m_preExpiryAdvance),
                   MakeTimeChecker ())
    .AddAttribute ("MinRequestHold",
                   "Minimum suppression interval for unicast refresh requests.",
                   TimeValue (MilliSeconds (200)),
                   MakeTimeAccessor (&RoutingProtocol::m_minRequestHold),
                   MakeTimeChecker ())
    .AddAttribute ("MaxRequestHold",
                   "Maximum suppression interval for unicast refresh requests.",
                   TimeValue (Seconds (1.0)),
                   MakeTimeAccessor (&RoutingProtocol::m_maxRequestHold),
                   MakeTimeChecker ());
  return tid;
}

RoutingProtocol::RoutingProtocol ()
  : m_ipv4 (nullptr),
    m_mainAddress (),
    m_lo (nullptr),
    m_socketAddresses (),
    m_routingTable (),
    m_idCache (Seconds (10)),
    m_helloEvent (),
    m_globalEvent (),
    m_maintenanceEvent (),
    m_sequenceNumber (0),
    m_helloInterval (Seconds (1)),
    m_globalUpdateInterval (Seconds (5)),
    m_preExpiryAdvance (Seconds (1)),
    m_minRequestHold (MilliSeconds (200)),
    m_maxRequestHold (Seconds (1)),
    m_zoneRadiusMid (2),
    m_zoneRadiusLow (1),
    m_zoneRadiusHigh (3),
    m_beaconRate (1.0),
    m_beaconPayloadSize (48),
    m_bandwidthBudget (1024.0),
    m_lastGlobalPosition (Vector (0.0, 0.0, 0.0)),
    m_lastGlobalVelocity (Vector (0.0, 0.0, 0.0)),
    m_lastGlobalTimestamp (Seconds (0)),
    m_uniformRandom (CreateObject<UniformRandomVariable> ())
{
}

RoutingProtocol::~RoutingProtocol ()
{
  Stop ();
}

void
RoutingProtocol::DoDispose ()
{
  Stop ();
  m_ipv4 = nullptr;
  m_lo = nullptr;
  m_socketAddresses.clear ();
  Ipv4RoutingProtocol::DoDispose ();
}

int64_t
RoutingProtocol::AssignStreams (int64_t stream)
{
  m_uniformRandom->SetStream (stream);
  return 1;
}

void
RoutingProtocol::Start ()
{
  NS_ASSERT (m_ipv4 != nullptr);

  if (m_helloEvent.IsRunning () || m_globalEvent.IsRunning () || m_maintenanceEvent.IsRunning ())
    {
      return;
    }

  // 中文说明：遍历所有IPv4接口，为每个接口创建控制套接字并绑定协议端口。
  m_lo = m_ipv4->GetNetDevice (0);
  m_mainAddress = Ipv4Address ();

  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); ++i)
    {
      Ipv4Address address = m_ipv4->GetAddress (i, 0).GetLocal ();
      if (address == Ipv4Address ())
        {
          continue;
        }
      Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (), UdpSocketFactory::GetTypeId ());
      socket->SetAllowBroadcast (true);
      InetSocketAddress local = InetSocketAddress (address, LADZRP_PORT);
      if (socket->Bind (local) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind LADZRP socket");
        }
      // 中文说明：所有控制报文统一交由RecvControl处理。
      socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvControl, this));
      m_socketAddresses.insert (std::make_pair (socket, m_ipv4->GetAddress (i, 0)));
      if (m_mainAddress == Ipv4Address ())
        {
          m_mainAddress = address;
        }
    }

  Ptr<MobilityModel> mobility = m_ipv4->GetObject<MobilityModel> ();
  if (mobility)
    {
      m_lastGlobalPosition = mobility->GetPosition ();
      m_lastGlobalVelocity = mobility->GetVelocity ();
      m_lastGlobalTimestamp = Simulator::Now ();
    }

  // 中文说明：协议启动后立即激活HELLO、全局更新与维护定时器。
  ScheduleHello (Seconds (0));
  ScheduleGlobalUpdate (Seconds (0));
  ScheduleMaintenance ();
}

void
RoutingProtocol::Stop ()
{
  if (m_helloEvent.IsRunning ())
    {
      m_helloEvent.Cancel ();
    }
  if (m_globalEvent.IsRunning ())
    {
      m_globalEvent.Cancel ();
    }
  if (m_maintenanceEvent.IsRunning ())
    {
      m_maintenanceEvent.Cancel ();
    }

  for (auto &entry : m_socketAddresses)
    {
      entry.first->Close ();
    }
  m_socketAddresses.clear ();
}

void
RoutingProtocol::SetIpv4 (Ptr<Ipv4> ipv4)
{
  NS_ASSERT (ipv4 != nullptr);
  m_ipv4 = ipv4;
  Start ();
}

void
RoutingProtocol::NotifyInterfaceUp (uint32_t interface)
{
  if (m_ipv4->GetAddress (interface, 0).GetLocal () == Ipv4Address ())
    {
      return;
    }
  Start ();
}

void
RoutingProtocol::NotifyInterfaceDown (uint32_t interface)
{
  Stop ();
  Start ();
}

void
RoutingProtocol::NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address)
{
  Start ();
}

void
RoutingProtocol::NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address)
{
  Start ();
}

void
RoutingProtocol::ScheduleHello (Time delay)
{
  if (m_helloEvent.IsRunning ())
    {
      m_helloEvent.Cancel ();
    }
  // 中文说明：为周期HELLO增加0~10ms随机抖动，降低同步拥塞概率。
  m_helloEvent = Simulator::Schedule (delay + Seconds (m_uniformRandom->GetValue (0.0, 0.01)), &RoutingProtocol::SendHello, this, Ipv4Address ());
}

void
RoutingProtocol::SendHello (Ipv4Address unicast)
{
  Ptr<MobilityModel> mobility = m_ipv4->GetObject<MobilityModel> ();
  if (mobility == nullptr)
    {
      return;
    }

  Vector position = mobility->GetPosition ();
  Vector velocity = mobility->GetVelocity ();
  Time now = Simulator::Now ();

  // 中文说明：HELLO在周期广播时使用较大TTL，在定向刷新时仅单跳传播。
  MessageHeader msg (MessageType::HELLO, unicast.IsAny () ? m_zoneRadiusHigh : 1, ++m_sequenceNumber);
  HelloHeader hello;
  hello.SetSourcePosition (position);
  hello.SetSourceVelocity (velocity);
  hello.SetLastHopPosition (position);
  hello.SetAverageHopDistance (0.0);
  hello.SetHopCount (1);
  hello.SetTimestamp (now);
  hello.SetOriginator (m_mainAddress);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (hello);
  packet->AddHeader (msg);

  if (unicast.IsAny ())
    {
      // 中文说明：常规HELLO通过接口广播覆盖整个分区范围。
      for (const auto &socketAddress : m_socketAddresses)
        {
          Ptr<Socket> socket = socketAddress.first;
          Ipv4InterfaceAddress iface = socketAddress.second;
          Ipv4Address destination = iface.GetBroadcast ();
          socket->SendTo (packet->Copy (), 0, InetSocketAddress (destination, LADZRP_PORT));
        }
      ScheduleHello (m_helloInterval);
    }
  else
    {
      // 中文说明：触发式刷新仅需单播给请求节点，无需重新调度周期任务。
      if (!m_socketAddresses.empty ())
        {
          Ptr<Socket> socket = m_socketAddresses.begin ()->first;
          socket->SendTo (packet, 0, InetSocketAddress (unicast, LADZRP_PORT));
        }
    }
}

void
RoutingProtocol::ProcessHello (Ptr<Packet> packet, const MessageHeader &msg, const HelloHeader &hello, Ipv4Address receiver, Ipv4Address previousHop)
{
  Ipv4Address origin = hello.GetOriginator ();
  if (m_idCache.IsDuplicate (origin, msg.GetSequenceNumber ()))
    {
      return;
    }

  Ptr<MobilityModel> mobility = m_ipv4->GetObject<MobilityModel> ();
  if (mobility == nullptr)
    {
      return;
    }
  Vector myPosition = mobility->GetPosition ();
  Vector myVelocity = mobility->GetVelocity ();

  double avgHd = hello.GetAverageHopDistance ();
  if (hello.GetHopCount () > 1)
    {
      // 中文说明：根据式(3)更新平均跳距信息以反映网络密度。
      double segment = CalculateDistance (myPosition, hello.GetLastHopPosition ());
      avgHd = (avgHd * static_cast<double> (hello.GetHopCount () - 1) + segment) / static_cast<double> (hello.GetHopCount ());
    }

  RoutingTableEntry entry;
  entry.SetAddress (origin);
  entry.SetPosition (hello.GetSourcePosition ());
  entry.SetVelocity (hello.GetSourceVelocity ());
  entry.SetTimestamp (hello.GetTimestamp ());
  entry.SetAverageHopDistance (avgHd);
  entry.SetHopCount (hello.GetHopCount ());
  entry.SetLastHopPosition (myPosition);

  double lifetimeSeconds = ComputeLifetime (hello, avgHd);
  if (std::isfinite (lifetimeSeconds) && lifetimeSeconds > 0.0)
    {
      // 中文说明：采用式(4)计算有效期并写回，用于后续刷新判定。
      entry.SetExpiry (Simulator::Now () + Seconds (lifetimeSeconds));
    }
  else
    {
      entry.SetExpiry (Simulator::Now () + m_helloInterval * 3);
    }
  entry.ResetFailureStatistics ();
  UpdateZoneEntry (entry);

  if (msg.GetTtl () <= 1)
    {
      return;
    }

  MessageHeader forwardMsg = msg;
  forwardMsg.SetTtl (msg.GetTtl () - 1);

  HelloHeader forwardHello = hello;
  forwardHello.SetHopCount (hello.GetHopCount () + 1);
  forwardHello.SetAverageHopDistance (avgHd);
  forwardHello.SetLastHopPosition (myPosition);
  forwardHello.SetSourcePosition (hello.GetSourcePosition ());
  forwardHello.SetSourceVelocity (hello.GetSourceVelocity ());

  Ptr<Packet> forwardPacket = Create<Packet> ();
  forwardPacket->AddHeader (forwardHello);
  forwardPacket->AddHeader (forwardMsg);

  for (const auto &socketAddress : m_socketAddresses)
    {
      Ptr<Socket> socket = socketAddress.first;
      Ipv4InterfaceAddress iface = socketAddress.second;
      if (iface.GetLocal () == receiver)
        {
          continue;
        }
      // 中文说明：继续广播到除来源接口外的其余接口，实现分区泛洪。
      socket->SendTo (forwardPacket->Copy (), 0, InetSocketAddress (iface.GetBroadcast (), LADZRP_PORT));
    }
}

void
RoutingProtocol::ScheduleGlobalUpdate (Time delay)
{
  if (m_globalEvent.IsRunning ())
    {
      m_globalEvent.Cancel ();
    }
  m_globalEvent = Simulator::Schedule (delay, &RoutingProtocol::MaybeSendGlobalUpdate, this, false);
}

void
RoutingProtocol::MaybeSendGlobalUpdate (bool force)
{
  Ptr<MobilityModel> mobility = m_ipv4->GetObject<MobilityModel> ();
  if (mobility == nullptr)
    {
      return;
    }

  Vector position = mobility->GetPosition ();
  Vector velocity = mobility->GetVelocity ();
  Time now = Simulator::Now ();

  double avgHd = m_routingTable.ComputeAverageHopDistance ();
  double threshold = std::max (avgHd, 1.0) * static_cast<double> (m_zoneRadiusMid);

  Vector predicted (m_lastGlobalPosition.x + m_lastGlobalVelocity.x * (now - m_lastGlobalTimestamp).GetSeconds (),
                    m_lastGlobalPosition.y + m_lastGlobalVelocity.y * (now - m_lastGlobalTimestamp).GetSeconds (),
                    m_lastGlobalPosition.z + m_lastGlobalVelocity.z * (now - m_lastGlobalTimestamp).GetSeconds ());
  double deviation = CalculateDistance (predicted, position);

  if (force || deviation > threshold)
    {
      // 中文说明：当预测误差超过阈值时广播新的全局方向信息。
      MessageHeader msg (MessageType::GLOBAL_UPDATE, std::max<uint8_t> (m_zoneRadiusHigh, static_cast<uint8_t> (m_zoneRadiusMid * 2)), ++m_sequenceNumber);
      GlobalUpdateHeader header;
      header.SetPosition (position);
      header.SetVelocity (velocity);
      header.SetTimestamp (now);
      header.SetOriginator (m_mainAddress);

      Ptr<Packet> packet = Create<Packet> ();
      packet->AddHeader (header);
      packet->AddHeader (msg);

      for (const auto &socketAddress : m_socketAddresses)
        {
          Ptr<Socket> socket = socketAddress.first;
          Ipv4InterfaceAddress iface = socketAddress.second;
          socket->SendTo (packet->Copy (), 0, InetSocketAddress (iface.GetBroadcast (), LADZRP_PORT));
        }

      m_lastGlobalPosition = position;
      m_lastGlobalVelocity = velocity;
      m_lastGlobalTimestamp = now;
    }

  ScheduleGlobalUpdate (m_globalUpdateInterval);
}

void
RoutingProtocol::ProcessGlobalUpdate (Ptr<Packet> packet, const MessageHeader &msg, const GlobalUpdateHeader &header, Ipv4Address previousHop)
{
  Ipv4Address origin = header.GetOriginator ();
  if (m_idCache.IsDuplicate (origin, msg.GetSequenceNumber ()))
    {
      return;
    }

  RoutingTableEntry entry;
  entry.SetAddress (origin);
  entry.SetPosition (header.GetPosition ());
  entry.SetVelocity (header.GetVelocity ());
  entry.SetTimestamp (header.GetTimestamp ());
  entry.SetExpiry (Simulator::Now () + m_globalUpdateInterval * 3);
  entry.SetGlobalEntry (true);
  UpdateGlobalEntry (entry);

  if (msg.GetTtl () <= 1)
    {
      return;
    }

  MessageHeader forwardMsg = msg;
  forwardMsg.SetTtl (msg.GetTtl () - 1);

  Ptr<Packet> forwardPacket = Create<Packet> ();
  forwardPacket->AddHeader (header);
  forwardPacket->AddHeader (forwardMsg);

  for (const auto &socketAddress : m_socketAddresses)
    {
      Ptr<Socket> socket = socketAddress.first;
      Ipv4InterfaceAddress iface = socketAddress.second;
      if (iface.GetLocal () == previousHop)
        {
          continue;
        }
      // 中文说明：避免向上一跳接口回送，确保泛洪沿拓扑向前推进。
      socket->SendTo (forwardPacket->Copy (), 0, InetSocketAddress (iface.GetBroadcast (), LADZRP_PORT));
    }
}

void
RoutingProtocol::ScheduleMaintenance ()
{
  if (m_maintenanceEvent.IsRunning ())
    {
      m_maintenanceEvent.Cancel ();
    }
  m_maintenanceEvent = Simulator::Schedule (Seconds (1), &RoutingProtocol::PerformMaintenance, this);
}

void
RoutingProtocol::PerformMaintenance ()
{
  Time now = Simulator::Now ();
  m_routingTable.Purge (now);

  auto zoneSnapshot = m_routingTable.GetZoneSnapshot ();
  for (auto &kv : zoneSnapshot)
    {
      RoutingTableEntry entry = kv.second;
      if (entry.GetExpiry () != Seconds (0) && entry.GetExpiry () - now <= m_preExpiryAdvance)
        {
          if (now >= entry.GetSuppressUntil ())
            {
              // 中文说明：临近过期时发送一次定向刷新请求，并更新抑制时间。
              SendUnicastRequest (kv.first, entry);
              entry.SetSuppressUntil (now + ComputeSuppressionWindow (entry));
              entry.SetLastRequest (now);
              m_routingTable.AddOrUpdateZone (entry);
            }
        }
      if (entry.GetFailureRatio () > 0.5 && now >= entry.GetSuppressUntil ())
        {
          // 中文说明：失败率超过50%视为链路不稳定，同样触发刷新并清空统计。
          SendUnicastRequest (kv.first, entry);
          entry.ResetFailureStatistics ();
          entry.SetSuppressUntil (now + ComputeSuppressionWindow (entry));
          entry.SetLastRequest (now);
          m_routingTable.AddOrUpdateZone (entry);
        }
    }

  ScheduleMaintenance ();
}

void
RoutingProtocol::SendUnicastRequest (Ipv4Address neighbor, const RoutingTableEntry &entry)
{
  MessageHeader msg (MessageType::UNICAST_REQUEST, 1, ++m_sequenceNumber);
  HelloHeader hello;
  Ptr<MobilityModel> mobility = m_ipv4->GetObject<MobilityModel> ();
  if (mobility)
    {
      hello.SetSourcePosition (mobility->GetPosition ());
      hello.SetSourceVelocity (mobility->GetVelocity ());
      hello.SetLastHopPosition (mobility->GetPosition ());
    }
  else
    {
      hello.SetSourcePosition (Vector (0.0, 0.0, 0.0));
      hello.SetSourceVelocity (Vector (0.0, 0.0, 0.0));
      hello.SetLastHopPosition (Vector (0.0, 0.0, 0.0));
    }
  hello.SetAverageHopDistance (entry.GetAverageHopDistance ());
  hello.SetHopCount (1);
  hello.SetTimestamp (Simulator::Now ());
  hello.SetOriginator (m_mainAddress);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (hello);
  packet->AddHeader (msg);

  if (!m_socketAddresses.empty ())
    {
      // 中文说明：使用首个接口即可完成单播请求，目标节点收到后会立即回传HELLO。
      Ptr<Socket> socket = m_socketAddresses.begin ()->first;
      socket->SendTo (packet, 0, InetSocketAddress (neighbor, LADZRP_PORT));
    }
}

void
RoutingProtocol::ProcessUnicastRequest (Ipv4Address requester)
{
  // 中文说明：收到刷新请求后立即定向发送一帧HELLO，补全对端的分区信息。
  SendHello (requester);
}

void
RoutingProtocol::RecvControl (Ptr<Socket> socket)
{
  Address sourceAddress;
  Ptr<Packet> packet = socket->RecvFrom (sourceAddress);
  InetSocketAddress inetSource = InetSocketAddress::ConvertFrom (sourceAddress);
  Ipv4Address sender = inetSource.GetIpv4 ();
  Ipv4InterfaceAddress iface = m_socketAddresses[socket];

  MessageHeader msg;
  if (packet->RemoveHeader (msg) == 0)
    {
      return;
    }

  switch (msg.GetMessageType ())
    {
    case MessageType::HELLO:
      {
        HelloHeader hello;
        packet->RemoveHeader (hello);
        // 中文说明：更新分区内路由表，同时继续泛洪HELLO。
        ProcessHello (packet, msg, hello, iface.GetLocal (), sender);
        break;
      }
    case MessageType::GLOBAL_UPDATE:
      {
        GlobalUpdateHeader header;
        packet->RemoveHeader (header);
        // 中文说明：写入全局方向表并向其他接口转发。
        ProcessGlobalUpdate (packet, msg, header, sender);
        break;
      }
    case MessageType::UNICAST_REQUEST:
      {
        ProcessUnicastRequest (sender);
        break;
      }
    default:
      break;
    }
}

Vector
RoutingProtocol::PredictPosition (const RoutingTableEntry &entry) const
{
  // 中文说明：使用线性外推预测目标节点当前所在位置。
  Time delta = Simulator::Now () - entry.GetTimestamp ();
  Vector predicted (entry.GetPosition ().x + entry.GetVelocity ().x * delta.GetSeconds (),
                    entry.GetPosition ().y + entry.GetVelocity ().y * delta.GetSeconds (),
                    entry.GetPosition ().z + entry.GetVelocity ().z * delta.GetSeconds ());
  return predicted;
}

Vector
RoutingProtocol::PredictFromHeader (const DataHeader &header) const
{
  // 中文说明：根据数据头内嵌的目的信息推算最新位置。
  Time delta = Simulator::Now () - header.GetDestinationTimestamp ();
  Vector predicted (header.GetDestinationPosition ().x + header.GetDestinationVelocity ().x * delta.GetSeconds (),
                    header.GetDestinationPosition ().y + header.GetDestinationVelocity ().y * delta.GetSeconds (),
                    header.GetDestinationPosition ().z + header.GetDestinationVelocity ().z * delta.GetSeconds ());
  return predicted;
}

double
RoutingProtocol::ComputeLifetime (const HelloHeader &hello, double avgHopDistance) const
{
  Ptr<MobilityModel> mobility = m_ipv4->GetObject<MobilityModel> ();
  if (mobility == nullptr)
    {
      return m_helloInterval.GetSeconds ();
    }

  Vector myPos = mobility->GetPosition ();
  Vector myVel = mobility->GetVelocity ();
  Vector otherPos = hello.GetSourcePosition ();
  Vector otherVel = hello.GetSourceVelocity ();

  Vector r0 (otherPos.x - myPos.x, otherPos.y - myPos.y, otherPos.z - myPos.z);
  Vector v (otherVel.x - myVel.x, otherVel.y - myVel.y, otherVel.z - myVel.z);

  // 中文说明：将问题转化为相对运动的入球时间，radius=avgHd*H表示可接受误差。
  double radius = std::max (avgHopDistance, 1.0) * static_cast<double> (m_zoneRadiusMid);
  double r0sq = r0.x * r0.x + r0.y * r0.y + r0.z * r0.z;
  double vsq = v.x * v.x + v.y * v.y + v.z * v.z;
  double dot = r0.x * v.x + r0.y * v.y + r0.z * v.z;
  double c = r0sq - radius * radius;

  if (vsq < 1e-9)
    {
      if (c >= 0.0)
        {
          return 0.0;
        }
      return std::numeric_limits<double>::infinity ();
    }

  double a = vsq;
  double b = 2.0 * dot;
  double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0)
    {
      if (c < 0.0)
        {
          return std::numeric_limits<double>::infinity ();
        }
      return 0.0;
    }

  double sqrtD = std::sqrt (discriminant);
  double t1 = (-b - sqrtD) / (2.0 * a);
  double t2 = (-b + sqrtD) / (2.0 * a);

  double result = std::numeric_limits<double>::infinity ();
  if (t1 > 0.0)
    {
      result = std::min (result, t1);
    }
  if (t2 > 0.0)
    {
      result = std::min (result, t2);
    }
  if (!std::isfinite (result))
    {
      if (c >= 0.0)
        {
          return 0.0;
        }
    }
  return result;
}

Time
RoutingProtocol::ComputeSuppressionWindow (const RoutingTableEntry &entry) const
{
  Time now = Simulator::Now ();
  Time remaining = entry.GetExpiry () - now;
  if (remaining.IsNegative ())
    {
      remaining = Seconds (0);
    }
  // 中文说明：抑制窗口默认取剩余寿命的20%，并限制在[min,max]之间。
  Time suggested = Seconds (0.2 * remaining.GetSeconds ());
  if (suggested < m_minRequestHold)
    {
      suggested = m_minRequestHold;
    }
  if (suggested > m_maxRequestHold)
    {
      suggested = m_maxRequestHold;
    }
  return suggested;
}

void
RoutingProtocol::UpdateZoneEntry (const RoutingTableEntry &entry)
{
  m_routingTable.AddOrUpdateZone (entry);
}

void
RoutingProtocol::UpdateGlobalEntry (const RoutingTableEntry &entry)
{
  m_routingTable.AddOrUpdateGlobal (entry);
}

Ptr<Ipv4Route>
RoutingProtocol::CreateRouteToNeighbor (Ipv4Address neighbor, Ipv4Address destination, Ptr<NetDevice> oif)
{
  Ptr<Ipv4Route> route = Create<Ipv4Route> ();
  uint32_t interface = (oif == nullptr) ? m_ipv4->GetInterfaceForAddress (m_mainAddress) : m_ipv4->GetInterfaceForDevice (oif);
  if (interface == static_cast<uint32_t> (-1))
    {
      return nullptr;
    }
  // 中文说明：单播路由通过设置网关为选定邻居，实现定向波束传输。
  route->SetDestination (destination);
  route->SetGateway (neighbor);
  route->SetSource (m_ipv4->GetAddress (interface, 0).GetLocal ());
  route->SetOutputDevice (m_ipv4->GetNetDevice (interface));
  return route;
}

Ptr<Ipv4Route>
RoutingProtocol::CreateBroadcastRoute (Ipv4Address destination, Ptr<NetDevice> oif)
{
  Ptr<Ipv4Route> route = Create<Ipv4Route> ();
  uint32_t interface = (oif == nullptr) ? m_ipv4->GetInterfaceForAddress (m_mainAddress) : m_ipv4->GetInterfaceForDevice (oif);
  if (interface == static_cast<uint32_t> (-1))
    {
      return nullptr;
    }
  // 中文说明：备份阶段需要广播发射，因此将网关设置为IPv4广播地址。
  route->SetDestination (destination);
  route->SetGateway (Ipv4Address::GetBroadcast ());
  route->SetSource (m_ipv4->GetAddress (interface, 0).GetLocal ());
  route->SetOutputDevice (m_ipv4->GetNetDevice (interface));
  return route;
}

void
RoutingProtocol::UpdateFailureStatistics (Ipv4Address neighbor, bool success)
{
  RoutingTableEntry entry;
  if (m_routingTable.LookupZone (neighbor, entry))
    {
      entry.RecordTransmissionResult (success);
      m_routingTable.AddOrUpdateZone (entry);
    }
}

bool
RoutingProtocol::IsOwnAddress (Ipv4Address address) const
{
  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); ++i)
    {
      for (uint32_t j = 0; j < m_ipv4->GetNAddresses (i); ++j)
        {
          if (m_ipv4->GetAddress (i, j).GetLocal () == address)
            {
              return true;
            }
        }
    }
  return false;
}

Ptr<Ipv4Route>
RoutingProtocol::RouteOutput (Ptr<Packet> packet, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr)
{
  Ptr<Ipv4Route> route = Create<Ipv4Route> ();
  route->SetDestination (header.GetDestination ());
  Ptr<NetDevice> outDevice = oif;
  if (outDevice == nullptr)
    {
      uint32_t interface = m_ipv4->GetInterfaceForAddress (m_mainAddress);
      if (interface != static_cast<uint32_t> (-1))
        {
          outDevice = m_ipv4->GetNetDevice (interface);
        }
    }
  if (outDevice != nullptr)
    {
      route->SetOutputDevice (outDevice);
      uint32_t interface = m_ipv4->GetInterfaceForDevice (outDevice);
      if (interface != static_cast<uint32_t> (-1))
        {
          route->SetSource (m_ipv4->GetAddress (interface, 0).GetLocal ());
        }
    }
  Ptr<MobilityModel> mobility = m_ipv4->GetObject<MobilityModel> ();
  Vector myPosition = mobility ? mobility->GetPosition () : Vector (0.0, 0.0, 0.0);
  Vector myVelocity = mobility ? mobility->GetVelocity () : Vector (0.0, 0.0, 0.0);

  RoutingTableEntry destEntry;
  Socket::SocketErrno error = Socket::ERROR_NOTERROR;
  bool success = false;

  uint8_t phase = 2;
  DataHeader dataHeader;
  // 中文说明：数据包始终附带发送者位姿，方便后续节点比较距离。
  dataHeader.SetSenderPosition (myPosition);
  dataHeader.SetSenderVelocity (myVelocity);
  dataHeader.SetHopCount (0);

  if (m_routingTable.LookupZone (header.GetDestination (), destEntry))
    {
      dataHeader.SetDestinationPosition (destEntry.GetPosition ());
      dataHeader.SetDestinationVelocity (destEntry.GetVelocity ());
      dataHeader.SetDestinationTimestamp (destEntry.GetTimestamp ());
      phase = 0;
      success = ForwardIntraZone (packet, route, header, destEntry, error);
    }
  else if (m_routingTable.LookupGlobal (header.GetDestination (), destEntry))
    {
      dataHeader.SetDestinationPosition (destEntry.GetPosition ());
      dataHeader.SetDestinationVelocity (destEntry.GetVelocity ());
      dataHeader.SetDestinationTimestamp (destEntry.GetTimestamp ());
      phase = 1;
      success = ForwardInterZone (packet, route, header, destEntry, error);
    }
  else
    {
      dataHeader.SetDestinationPosition (myPosition);
      dataHeader.SetDestinationVelocity (myVelocity);
      dataHeader.SetDestinationTimestamp (Simulator::Now ());
      success = ForwardBackup (packet, route, header, error);
      phase = 2;
    }

  if (!success || route == nullptr)
    {
      sockerr = error;
      return nullptr;
    }
  if (route->GetGateway () == Ipv4Address::GetBroadcast ())
    {
      phase = 2;
    }
  // 中文说明：写回阶段信息供下一跳继续执行同一转发策略。
  dataHeader.SetPhaseFlags (phase);
  packet->AddHeader (dataHeader);
  sockerr = Socket::ERROR_NOTERROR;
  return route;
}

bool
RoutingProtocol::ForwardIntraZone (Ptr<Packet> packet, Ptr<Ipv4Route> route, const Ipv4Header &header, RoutingTableEntry &entry, Socket::SocketErrno &sockerr)
{
  Ptr<MobilityModel> mobility = m_ipv4->GetObject<MobilityModel> ();
  Vector myPosition = mobility ? mobility->GetPosition () : Vector (0.0, 0.0, 0.0);
  Vector predictedDest = PredictPosition (entry);

  // 中文说明：基于区内表选择角度最小且距离更近的邻居作为下一跳。
  auto bestNeighbor = m_routingTable.BestNeighbor (predictedDest, myPosition);
  if (!bestNeighbor)
    {
      entry.RecordTransmissionResult (false);
      m_routingTable.AddOrUpdateZone (entry);
      sockerr = Socket::ERROR_NOROUTETOHOST;
      // 中文说明：若无合适邻居，进入备份阶段的波束广播。
      return ForwardBackup (packet, route, header, sockerr);
    }

  Ptr<Ipv4Route> nextRoute = CreateRouteToNeighbor (*bestNeighbor, header.GetDestination (), route->GetOutputDevice ());
  if (nextRoute == nullptr)
    {
      sockerr = Socket::ERROR_NOROUTETOHOST;
      return false;
    }
  *route = *nextRoute;
  UpdateFailureStatistics (*bestNeighbor, true);
  entry.RecordTransmissionResult (true);
  m_routingTable.AddOrUpdateZone (entry);
  return true;
}

bool
RoutingProtocol::ForwardInterZone (Ptr<Packet> packet, Ptr<Ipv4Route> route, const Ipv4Header &header, RoutingTableEntry &entry, Socket::SocketErrno &sockerr)
{
  Ptr<MobilityModel> mobility = m_ipv4->GetObject<MobilityModel> ();
  Vector myPosition = mobility ? mobility->GetPosition () : Vector (0.0, 0.0, 0.0);
  Vector predictedDest = PredictPosition (entry);

  // 中文说明：区间阶段同样使用贪婪准则，但目的位置来自全局表。
  auto bestNeighbor = m_routingTable.BestNeighbor (predictedDest, myPosition);
  if (!bestNeighbor)
    {
      entry.RecordTransmissionResult (false);
      m_routingTable.AddOrUpdateZone (entry);
      sockerr = Socket::ERROR_NOROUTETOHOST;
      // 中文说明：找不到更优邻居时回退到备份广播，防止传输中断。
      return ForwardBackup (packet, route, header, sockerr);
    }

  Ptr<Ipv4Route> nextRoute = CreateRouteToNeighbor (*bestNeighbor, header.GetDestination (), route->GetOutputDevice ());
  if (nextRoute == nullptr)
    {
      sockerr = Socket::ERROR_NOROUTETOHOST;
      return false;
    }
  *route = *nextRoute;
  UpdateFailureStatistics (*bestNeighbor, true);
  entry.RecordTransmissionResult (true);
  m_routingTable.AddOrUpdateZone (entry);
  return true;
}

bool
RoutingProtocol::ForwardBackup (Ptr<Packet> packet, Ptr<Ipv4Route> route, const Ipv4Header &header, Socket::SocketErrno &sockerr)
{
  Ptr<Ipv4Route> broadcastRoute = CreateBroadcastRoute (header.GetDestination (), route->GetOutputDevice ());
  if (broadcastRoute == nullptr)
    {
      sockerr = Socket::ERROR_NOROUTETOHOST;
      return false;
    }
  *route = *broadcastRoute;
  sockerr = Socket::ERROR_NOTERROR;
  // 中文说明：备份阶段实际发送广播，由接收方自行筛选更靠近目标的节点。
  return true;
}

bool
RoutingProtocol::RouteInput (Ptr<const Packet> p, const Ipv4Header &header, Ptr<const NetDevice> idev,
                             UnicastForwardCallback ucb, MulticastForwardCallback mcb, LocalDeliverCallback lcb,
                             ErrorCallback ecb)
{
  if (IsOwnAddress (header.GetDestination ()))
    {
      Ptr<Packet> packet = p->Copy ();
      DataHeader dataHeader;
      if (packet->PeekHeader (dataHeader))
        {
          packet->RemoveHeader (dataHeader);
        }
      if (!lcb.IsNull ())
        {
          lcb (packet, header, 0);
        }
      return true;
    }

  Ptr<Packet> packet = p->Copy ();
  DataHeader dataHeader;
  if (!packet->RemoveHeader (dataHeader))
    {
      return false;
    }

  Ptr<MobilityModel> mobility = m_ipv4->GetObject<MobilityModel> ();
  Vector myPosition = mobility ? mobility->GetPosition () : Vector (0.0, 0.0, 0.0);
  Vector myVelocity = mobility ? mobility->GetVelocity () : Vector (0.0, 0.0, 0.0);

  if (dataHeader.GetPhaseFlags () == 2)
    {
      // 中文说明：备份阶段要求只有距离更近的节点才继续转发，避免环路扩散。
      Vector predicted = PredictFromHeader (dataHeader);
      double senderDistance = CalculateDistance (dataHeader.GetSenderPosition (), predicted);
      double myDistance = CalculateDistance (myPosition, predicted);
      if (myDistance >= senderDistance)
        {
          return false;
        }
    }

  dataHeader.SetHopCount (dataHeader.GetHopCount () + 1);
  dataHeader.SetSenderPosition (myPosition);
  dataHeader.SetSenderVelocity (myVelocity);

  Ptr<Ipv4Route> route = Create<Ipv4Route> ();
  Ptr<NetDevice> outDevice = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (m_mainAddress));
  if (outDevice != nullptr)
    {
      route->SetOutputDevice (outDevice);
      route->SetSource (m_mainAddress);
    }
  Socket::SocketErrno error = Socket::ERROR_NOTERROR;
  bool success = false;
  RoutingTableEntry destEntry;

  if (m_routingTable.LookupZone (header.GetDestination (), destEntry))
    {
      success = ForwardIntraZone (packet, route, header, destEntry, error);
      dataHeader.SetPhaseFlags (0);
      dataHeader.SetDestinationPosition (destEntry.GetPosition ());
      dataHeader.SetDestinationVelocity (destEntry.GetVelocity ());
      dataHeader.SetDestinationTimestamp (destEntry.GetTimestamp ());
    }
  else if (m_routingTable.LookupGlobal (header.GetDestination (), destEntry))
    {
      success = ForwardInterZone (packet, route, header, destEntry, error);
      dataHeader.SetPhaseFlags (1);
      dataHeader.SetDestinationPosition (destEntry.GetPosition ());
      dataHeader.SetDestinationVelocity (destEntry.GetVelocity ());
      dataHeader.SetDestinationTimestamp (destEntry.GetTimestamp ());
    }
  else
    {
      success = ForwardBackup (packet, route, header, error);
      dataHeader.SetPhaseFlags (2);
    }

  if (!success)
    {
      return false;
    }

  if (route->GetGateway () == Ipv4Address::GetBroadcast ())
    {
      dataHeader.SetPhaseFlags (2);
    }

  // 中文说明：重新压入数据头，后续节点即可获取最新的预测信息。
  packet->AddHeader (dataHeader);
  if (!ucb.IsNull ())
    {
      ucb (route, packet, header);
    }
  return true;
}

void
RoutingProtocol::PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit) const
{
  *stream->GetStream () << "Zone table for node " << m_ipv4->GetObject<Node> ()->GetId () << std::endl;
  m_routingTable.PrintZoneTable (stream);
  *stream->GetStream () << "Global table for node " << m_ipv4->GetObject<Node> ()->GetId () << std::endl;
  m_routingTable.PrintGlobalTable (stream);
}

} // namespace ladzrp
} // namespace ns3
