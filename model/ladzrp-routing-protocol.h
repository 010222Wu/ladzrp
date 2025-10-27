#ifndef LADZRP_ROUTING_PROTOCOL_H
#define LADZRP_ROUTING_PROTOCOL_H

#include "ladzrp-id-cache.h"
#include "ladzrp-packet.h"
#include "ladzrp-rtable.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/random-variable-stream.h"
#include "ns3/timer.h"
#include <map>

// 中文说明：RoutingProtocol类实现LA-DZRP协议的核心流程，包括TTLM两级
// 维护与MPF多阶段转发。本头文件声明了核心接口、内部定时任务以及用于
// 维护路由状态的成员变量。

namespace ns3 {
namespace ladzrp {

class RoutingProtocol : public Ipv4RoutingProtocol
{
public:
  static const uint32_t LADZRP_PORT;
  static TypeId GetTypeId (void);

  RoutingProtocol ();
  ~RoutingProtocol () override;

  void DoDispose () override;

  Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr) override;
  bool RouteInput (Ptr<const Packet> p, const Ipv4Header &header, Ptr<const NetDevice> idev,
                   UnicastForwardCallback ucb, MulticastForwardCallback mcb, LocalDeliverCallback lcb,
                   ErrorCallback ecb) override;
  void NotifyInterfaceUp (uint32_t interface) override;
  void NotifyInterfaceDown (uint32_t interface) override;
  void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address) override;
  void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address) override;
  void SetIpv4 (Ptr<Ipv4> ipv4) override;
  void PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit = Time::S) const override;
  int64_t AssignStreams (int64_t stream) override;

private:
  // 中文说明：生命周期控制，负责初始化/释放套接字和定时任务。
  void Start ();
  void Stop ();

  // 中文说明：分区内HELLO的定时发送与接收处理，实现TTLM的第一层维护。
  void ScheduleHello (Time delay = Seconds (0));
  void SendHello (Ipv4Address unicast = Ipv4Address ());
  void ProcessHello (Ptr<Packet> packet, const MessageHeader &msg, const HelloHeader &hello, Ipv4Address receiver, Ipv4Address previousHop);

  // 中文说明：全局更新用于维护粗粒度方向信息，以下函数负责触发与处理。
  void ScheduleGlobalUpdate (Time delay = Seconds (0));
  void MaybeSendGlobalUpdate (bool force = false);
  void ProcessGlobalUpdate (Ptr<Packet> packet, const MessageHeader &msg, const GlobalUpdateHeader &header, Ipv4Address previousHop);

  // 中文说明：周期性维护任务，触发条目刷新与失败率检测。
  void ScheduleMaintenance ();
  void PerformMaintenance ();

  // 中文说明：当条目即将过期或失败率过高时，触发定向刷新请求。
  void SendUnicastRequest (Ipv4Address neighbor, const RoutingTableEntry &entry);
  void ProcessUnicastRequest (Ipv4Address requester);

  // 中文说明：控制通道统一入口，根据消息类型派发处理。
  void RecvControl (Ptr<Socket> socket);

  // 中文说明：MPF三阶段转发，分别处理区内、区间与备份广播。
  bool ForwardIntraZone (Ptr<Packet> packet, Ptr<Ipv4Route> route, const Ipv4Header &header, RoutingTableEntry &entry, Socket::SocketErrno &sockerr);
  bool ForwardInterZone (Ptr<Packet> packet, Ptr<Ipv4Route> route, const Ipv4Header &header, RoutingTableEntry &entry, Socket::SocketErrno &sockerr);
  bool ForwardBackup (Ptr<Packet> packet, Ptr<Ipv4Route> route, const Ipv4Header &header, Socket::SocketErrno &sockerr);

  // 中文说明：辅助函数，用于预测位置、计算有效期与抑制窗口等数学处理。
  Vector PredictPosition (const RoutingTableEntry &entry) const;
  Vector PredictFromHeader (const DataHeader &header) const;
  double ComputeLifetime (const HelloHeader &hello, double avgHopDistance) const;
  Time ComputeSuppressionWindow (const RoutingTableEntry &entry) const;

  // 中文说明：向路由表写回更新后的条目，保持封装性。
  void UpdateZoneEntry (const RoutingTableEntry &entry);
  void UpdateGlobalEntry (const RoutingTableEntry &entry);

  // 中文说明：构造单播/广播路由，用于Ipv4L3Protocol向下发送数据。
  Ptr<Ipv4Route> CreateRouteToNeighbor (Ipv4Address neighbor, Ipv4Address destination, Ptr<NetDevice> oif);
  Ptr<Ipv4Route> CreateBroadcastRoute (Ipv4Address destination, Ptr<NetDevice> oif);

  // 中文说明：根据发送结果更新失败统计，驱动TTLM触发逻辑。
  void UpdateFailureStatistics (Ipv4Address neighbor, bool success);

  // 中文说明：判断目的地址是否本机，用于决定本地交付。
  bool IsOwnAddress (Ipv4Address address) const;

  Ptr<Ipv4> m_ipv4;
  Ipv4Address m_mainAddress;
  Ptr<NetDevice> m_lo;
  std::map<Ptr<Socket>, Ipv4InterfaceAddress> m_socketAddresses;
  RoutingTable m_routingTable;
  IdCache m_idCache;

  EventId m_helloEvent;
  EventId m_globalEvent;
  EventId m_maintenanceEvent;
  uint32_t m_sequenceNumber;

  Time m_helloInterval;
  Time m_globalUpdateInterval;
  Time m_preExpiryAdvance;
  Time m_minRequestHold;
  Time m_maxRequestHold;

  uint8_t m_zoneRadiusMid;
  uint8_t m_zoneRadiusLow;
  uint8_t m_zoneRadiusHigh;
  double m_beaconRate;
  uint32_t m_beaconPayloadSize;
  double m_bandwidthBudget;

  Vector m_lastGlobalPosition;
  Vector m_lastGlobalVelocity;
  Time m_lastGlobalTimestamp;

  Ptr<UniformRandomVariable> m_uniformRandom;
};

} // namespace ladzrp
} // namespace ns3

#endif // LADZRP_ROUTING_PROTOCOL_H
