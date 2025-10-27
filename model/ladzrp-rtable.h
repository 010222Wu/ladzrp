#ifndef LADZRP_RTABLE_H
#define LADZRP_RTABLE_H

#include <cstdint>
#include <map>
#include <optional>
#include "ns3/ipv4-address.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/vector.h"
#include "ns3/nstime.h"

// 中文说明：路由表模块封装了LA-DZRP在TTLM中维护的分区内/全局元数据。
// 通过扩展的表项我们可以跟踪平均跳距、失效率以及触发刷新所需的抑制窗口。

namespace ns3 {
namespace ladzrp {

// 中文说明：单个节点的路由条目，既用于分区内表，也用于全局表。
class RoutingTableEntry
{
public:
  RoutingTableEntry ();

  void SetAddress (Ipv4Address address);
  Ipv4Address GetAddress () const;

  void SetPosition (const Vector &position);
  Vector GetPosition () const;

  void SetVelocity (const Vector &velocity);
  Vector GetVelocity () const;

  void SetTimestamp (Time timestamp);
  Time GetTimestamp () const;

  void SetHopCount (uint8_t hopCount);
  uint8_t GetHopCount () const;

  void SetAverageHopDistance (double avgHopDistance);
  double GetAverageHopDistance () const;

  void SetLastHopPosition (const Vector &position);
  Vector GetLastHopPosition () const;

  void SetExpiry (Time expiry);
  Time GetExpiry () const;
  bool IsExpired (Time now) const;

  void RecordTransmissionResult (bool success);
  double GetFailureRatio () const;
  void ResetFailureStatistics ();

  void SetSuppressUntil (Time time);
  Time GetSuppressUntil () const;

  void SetLastRequest (Time time);
  Time GetLastRequest () const;

  void SetGlobalEntry (bool isGlobal);
  bool IsGlobalEntry () const;

private:
  Ipv4Address m_address;
  Vector m_position;
  Vector m_velocity;
  Time m_timestamp;
  uint8_t m_hopCount;
  double m_avgHopDistance;
  Vector m_lastHopPosition;
  Time m_expiry;
  uint32_t m_txAttempts;
  uint32_t m_txFailures;
  Time m_suppressUntil;
  Time m_lastRequest;
  bool m_isGlobal;
};

// 中文说明：封装两套路由表（分区与全局），并提供常用查询与最优邻居选择接口。
class RoutingTable
{
public:
  RoutingTable ();

  bool LookupZone (Ipv4Address id, RoutingTableEntry &entry) const;
  bool LookupGlobal (Ipv4Address id, RoutingTableEntry &entry) const;

  void AddOrUpdateZone (const RoutingTableEntry &entry);
  void AddOrUpdateGlobal (const RoutingTableEntry &entry);

  void RemoveZone (Ipv4Address id);
  void RemoveGlobal (Ipv4Address id);

  void Purge (Time now);

  void PrintZoneTable (Ptr<OutputStreamWrapper> stream) const;
  void PrintGlobalTable (Ptr<OutputStreamWrapper> stream) const;

  std::map<Ipv4Address, RoutingTableEntry> GetZoneSnapshot () const;
  std::map<Ipv4Address, RoutingTableEntry> GetGlobalSnapshot () const;

  std::optional<Ipv4Address> BestNeighbor (const Vector &destination, const Vector &origin) const;

  double ComputeAverageHopDistance () const;

private:
  std::map<Ipv4Address, RoutingTableEntry> m_zoneTable;
  std::map<Ipv4Address, RoutingTableEntry> m_globalTable;
};

} // namespace ladzrp
} // namespace ns3

#endif // LADZRP_RTABLE_H
