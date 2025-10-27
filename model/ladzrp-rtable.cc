#include "ladzrp-rtable.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace ns3 {
namespace ladzrp {

NS_LOG_COMPONENT_DEFINE ("LadzrpRoutingTable");

// 中文说明：RoutingTableEntry主要作为数据容器，方法均为简单赋值或统计；
// 由于协议需要大量动态元数据，我们在此集中实现访问接口。

RoutingTableEntry::RoutingTableEntry ()
  : m_address (),
    m_position (Vector (0.0, 0.0, 0.0)),
    m_velocity (Vector (0.0, 0.0, 0.0)),
    m_timestamp (Seconds (0)),
    m_hopCount (0),
    m_avgHopDistance (0.0),
    m_lastHopPosition (Vector (0.0, 0.0, 0.0)),
    m_expiry (Seconds (0)),
    m_txAttempts (0),
    m_txFailures (0),
    m_suppressUntil (Seconds (0)),
    m_lastRequest (Seconds (0)),
    m_isGlobal (false)
{
}

void
RoutingTableEntry::SetAddress (Ipv4Address address)
{
  m_address = address;
}

Ipv4Address
RoutingTableEntry::GetAddress () const
{
  return m_address;
}

void
RoutingTableEntry::SetPosition (const Vector &position)
{
  m_position = position;
}

Vector
RoutingTableEntry::GetPosition () const
{
  return m_position;
}

void
RoutingTableEntry::SetVelocity (const Vector &velocity)
{
  m_velocity = velocity;
}

Vector
RoutingTableEntry::GetVelocity () const
{
  return m_velocity;
}

void
RoutingTableEntry::SetTimestamp (Time timestamp)
{
  m_timestamp = timestamp;
}

Time
RoutingTableEntry::GetTimestamp () const
{
  return m_timestamp;
}

void
RoutingTableEntry::SetHopCount (uint8_t hopCount)
{
  m_hopCount = hopCount;
}

uint8_t
RoutingTableEntry::GetHopCount () const
{
  return m_hopCount;
}

void
RoutingTableEntry::SetAverageHopDistance (double avgHopDistance)
{
  m_avgHopDistance = avgHopDistance;
}

double
RoutingTableEntry::GetAverageHopDistance () const
{
  return m_avgHopDistance;
}

void
RoutingTableEntry::SetLastHopPosition (const Vector &position)
{
  m_lastHopPosition = position;
}

Vector
RoutingTableEntry::GetLastHopPosition () const
{
  return m_lastHopPosition;
}

void
RoutingTableEntry::SetExpiry (Time expiry)
{
  m_expiry = expiry;
}

Time
RoutingTableEntry::GetExpiry () const
{
  return m_expiry;
}

bool
RoutingTableEntry::IsExpired (Time now) const
{
  return (m_expiry != Seconds (0)) && (now > m_expiry);
}

void
RoutingTableEntry::RecordTransmissionResult (bool success)
{
  // 中文说明：统计成功与失败的比例，用于失败率触发刷新逻辑。
  ++m_txAttempts;
  if (!success)
    {
      ++m_txFailures;
    }
}

double
RoutingTableEntry::GetFailureRatio () const
{
  if (m_txAttempts == 0)
    {
      return 0.0;
    }
  return static_cast<double> (m_txFailures) / static_cast<double> (m_txAttempts);
}

void
RoutingTableEntry::ResetFailureStatistics ()
{
  m_txAttempts = 0;
  m_txFailures = 0;
}

void
RoutingTableEntry::SetSuppressUntil (Time time)
{
  m_suppressUntil = time;
}

Time
RoutingTableEntry::GetSuppressUntil () const
{
  return m_suppressUntil;
}

void
RoutingTableEntry::SetLastRequest (Time time)
{
  m_lastRequest = time;
}

Time
RoutingTableEntry::GetLastRequest () const
{
  return m_lastRequest;
}

void
RoutingTableEntry::SetGlobalEntry (bool isGlobal)
{
  m_isGlobal = isGlobal;
}

bool
RoutingTableEntry::IsGlobalEntry () const
{
  return m_isGlobal;
}

RoutingTable::RoutingTable ()
  : m_zoneTable (),
    m_globalTable ()
{
}

// 中文说明：LookupZone/LookupGlobal均根据地址在对应字典中查询，供转发与维护调用。

bool
RoutingTable::LookupZone (Ipv4Address id, RoutingTableEntry &entry) const
{
  // 中文说明：优先返回拷贝以避免外部直接修改内部容器，保持语义清晰。
  auto it = m_zoneTable.find (id);
  if (it == m_zoneTable.end ())
    {
      return false;
    }
  entry = it->second;
  return true;
}

bool
RoutingTable::LookupGlobal (Ipv4Address id, RoutingTableEntry &entry) const
{
  auto it = m_globalTable.find (id);
  if (it == m_globalTable.end ())
    {
      return false;
    }
  entry = it->second;
  return true;
}

void
RoutingTable::AddOrUpdateZone (const RoutingTableEntry &entry)
{
  // 中文说明：更新分区内条目，并显式标记为局部表项。
  RoutingTableEntry stored = entry;
  stored.SetGlobalEntry (false);
  m_zoneTable[entry.GetAddress ()] = stored;
}

void
RoutingTable::AddOrUpdateGlobal (const RoutingTableEntry &entry)
{
  // 中文说明：更新全局方向表，供跨区转发时使用。
  RoutingTableEntry stored = entry;
  stored.SetGlobalEntry (true);
  m_globalTable[entry.GetAddress ()] = stored;
}

void
RoutingTable::RemoveZone (Ipv4Address id)
{
  m_zoneTable.erase (id);
}

void
RoutingTable::RemoveGlobal (Ipv4Address id)
{
  m_globalTable.erase (id);
}

void
RoutingTable::Purge (Time now)
{
  // 中文说明：每次维护时清理过期条目，避免节点使用过时方向信息。
  for (auto it = m_zoneTable.begin (); it != m_zoneTable.end (); )
    {
      if (it->second.IsExpired (now))
        {
          it = m_zoneTable.erase (it);
        }
      else
        {
          ++it;
        }
    }

  for (auto it = m_globalTable.begin (); it != m_globalTable.end (); )
    {
      if (it->second.IsExpired (now))
        {
          it = m_globalTable.erase (it);
        }
      else
        {
          ++it;
        }
    }
}

void
RoutingTable::PrintZoneTable (Ptr<OutputStreamWrapper> stream) const
{
  for (const auto &entry : m_zoneTable)
    {
      *stream->GetStream () << entry.first << " pos=" << entry.second.GetPosition ()
                            << " vel=" << entry.second.GetVelocity ()
                            << " hops=" << static_cast<uint32_t> (entry.second.GetHopCount ())
                            << " avgHd=" << entry.second.GetAverageHopDistance ()
                            << " expiry=" << entry.second.GetExpiry ().GetSeconds () << "s" << std::endl;
    }
}

void
RoutingTable::PrintGlobalTable (Ptr<OutputStreamWrapper> stream) const
{
  for (const auto &entry : m_globalTable)
    {
      *stream->GetStream () << entry.first << " pos=" << entry.second.GetPosition ()
                            << " vel=" << entry.second.GetVelocity ()
                            << " ts=" << entry.second.GetTimestamp ().GetSeconds () << "s" << std::endl;
    }
}

std::map<Ipv4Address, RoutingTableEntry>
RoutingTable::GetZoneSnapshot () const
{
  return m_zoneTable;
}

std::map<Ipv4Address, RoutingTableEntry>
RoutingTable::GetGlobalSnapshot () const
{
  return m_globalTable;
}

std::optional<Ipv4Address>
RoutingTable::BestNeighbor (const Vector &destination, const Vector &origin) const
{
  // 中文说明：实现论文中的波束感知贪婪转发，既比较距离又比较波束转角。
  std::optional<Ipv4Address> best;
  double bestAngle = std::numeric_limits<double>::max ();
  double bestDistance = std::numeric_limits<double>::max ();

  Vector toDest (destination.x - origin.x, destination.y - origin.y, destination.z - origin.z);
  double myDistToDest = std::sqrt (toDest.x * toDest.x + toDest.y * toDest.y + toDest.z * toDest.z);

  for (const auto &entry : m_zoneTable)
    {
      const Vector &neighborPos = entry.second.GetPosition ();
      Vector toNeighbor (neighborPos.x - origin.x, neighborPos.y - origin.y, neighborPos.z - origin.z);
      double neighborVecLen = std::sqrt (toNeighbor.x * toNeighbor.x + toNeighbor.y * toNeighbor.y + toNeighbor.z * toNeighbor.z);
      if (neighborVecLen == 0.0)
        {
          continue;
        }
      Vector neighborToDest (destination.x - neighborPos.x, destination.y - neighborPos.y, destination.z - neighborPos.z);
      double neighborDistToDest = std::sqrt (neighborToDest.x * neighborToDest.x + neighborToDest.y * neighborToDest.y + neighborToDest.z * neighborToDest.z);
      if (neighborDistToDest >= myDistToDest)
        {
          continue;
        }

      double denom = myDistToDest * neighborVecLen;
      double angle = 0.0;
      if (denom > 0.0)
        {
          double dot = toDest.x * toNeighbor.x + toDest.y * toNeighbor.y + toDest.z * toNeighbor.z;
          double cosTheta = std::max (-1.0, std::min (1.0, dot / denom));
          angle = std::acos (cosTheta);
        }

      if (angle < bestAngle - 1e-6)
        {
          bestAngle = angle;
          bestDistance = neighborDistToDest;
          best = entry.first;
        }
      else if (std::abs (angle - bestAngle) <= 1e-6 && neighborDistToDest < bestDistance)
        {
          bestDistance = neighborDistToDest;
          best = entry.first;
        }
    }

  return best;
}

double
RoutingTable::ComputeAverageHopDistance () const
{
  // 中文说明：平均跳距用于评估节点密度，并作为全局更新触发阈值的参考。
  if (m_zoneTable.empty ())
    {
      return 0.0;
    }
  double sum = 0.0;
  uint32_t count = 0;
  for (const auto &entry : m_zoneTable)
    {
      if (entry.second.GetAverageHopDistance () > 0.0)
        {
          sum += entry.second.GetAverageHopDistance ();
          ++count;
        }
    }
  if (count == 0)
    {
      return 0.0;
    }
  return sum / static_cast<double> (count);
}

} // namespace ladzrp
} // namespace ns3
