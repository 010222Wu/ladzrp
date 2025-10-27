#include "ladzrp-packet.h"
#include "ns3/log.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace ns3 {
namespace ladzrp {

NS_LOG_COMPONENT_DEFINE ("LadzrpPacket");

namespace
{
// 中文说明：为减少序列化负担，我们将Vector与Time字段编码为32位浮点
// 和64位时间戳，以下辅助函数负责在Packet缓冲区中读写这些数据。
uint32_t
EncodeFloat (double value)
{
  float f = static_cast<float> (value);
  uint32_t raw;
  std::memcpy (&raw, &f, sizeof (raw));
  return raw;
}

double
DecodeFloat (uint32_t raw)
{
  float f;
  std::memcpy (&f, &raw, sizeof (raw));
  return static_cast<double> (f);
}

void
WriteVector (Buffer::Iterator &i, const Vector &vector)
{
  i.WriteHtonU32 (EncodeFloat (vector.x));
  i.WriteHtonU32 (EncodeFloat (vector.y));
  i.WriteHtonU32 (EncodeFloat (vector.z));
}

Vector
ReadVector (Buffer::Iterator &i)
{
  Vector vector (0.0, 0.0, 0.0);
  uint32_t x = i.ReadNtohU32 ();
  uint32_t y = i.ReadNtohU32 ();
  uint32_t z = i.ReadNtohU32 ();
  vector.x = DecodeFloat (x);
  vector.y = DecodeFloat (y);
  vector.z = DecodeFloat (z);
  return vector;
}

void
WriteTime (Buffer::Iterator &i, Time time)
{
  i.WriteHtonU64 (static_cast<uint64_t> (time.GetNanoSeconds ()));
}

Time
ReadTime (Buffer::Iterator &i)
{
  uint64_t value = i.ReadNtohU64 ();
  return NanoSeconds (value);
}
} // namespace

NS_OBJECT_ENSURE_REGISTERED (MessageHeader);

MessageHeader::MessageHeader (MessageType type, uint8_t ttl, uint32_t sequenceNumber)
  : m_type (type),
    m_ttl (ttl),
    m_sequenceNumber (sequenceNumber)
{
}

TypeId
MessageHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ladzrp::MessageHeader")
    .SetParent<Header> ()
    .SetGroupName ("ladzrp")
    .AddConstructor<MessageHeader> ();
  return tid;
}

TypeId
MessageHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

uint32_t
MessageHeader::GetSerializedSize () const
{
  return sizeof (uint8_t) + sizeof (uint8_t) + sizeof (uint32_t);
}

void
MessageHeader::Serialize (Buffer::Iterator start) const
{
  // 中文说明：控制报文公共头部按照“类型-ttl-序号”顺序写入，保持解析一致。
  start.WriteU8 (static_cast<uint8_t> (m_type));
  start.WriteU8 (m_ttl);
  start.WriteHtonU32 (m_sequenceNumber);
}

uint32_t
MessageHeader::Deserialize (Buffer::Iterator start)
{
  m_type = static_cast<MessageType> (start.ReadU8 ());
  m_ttl = start.ReadU8 ();
  m_sequenceNumber = start.ReadNtohU32 ();
  return GetSerializedSize ();
}

void
MessageHeader::Print (std::ostream &os) const
{
  os << "type=" << static_cast<uint32_t> (m_type)
     << " ttl=" << static_cast<uint32_t> (m_ttl)
     << " seq=" << m_sequenceNumber;
}

void
MessageHeader::SetMessageType (MessageType type)
{
  m_type = type;
}

MessageType
MessageHeader::GetMessageType () const
{
  return m_type;
}

void
MessageHeader::SetTtl (uint8_t ttl)
{
  m_ttl = ttl;
}

uint8_t
MessageHeader::GetTtl () const
{
  return m_ttl;
}

void
MessageHeader::SetSequenceNumber (uint32_t seq)
{
  m_sequenceNumber = seq;
}

uint32_t
MessageHeader::GetSequenceNumber () const
{
  return m_sequenceNumber;
}

NS_OBJECT_ENSURE_REGISTERED (HelloHeader);

HelloHeader::HelloHeader ()
  : m_sourcePosition (Vector (0.0, 0.0, 0.0)),
    m_sourceVelocity (Vector (0.0, 0.0, 0.0)),
    m_originator (),
    m_lastHopPosition (Vector (0.0, 0.0, 0.0)),
    m_averageHopDistance (0.0),
    m_hopCount (0),
    m_timestamp (Seconds (0))
{
}

TypeId
HelloHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ladzrp::HelloHeader")
    .SetParent<Header> ()
    .SetGroupName ("ladzrp")
    .AddConstructor<HelloHeader> ();
  return tid;
}

TypeId
HelloHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

uint32_t
HelloHeader::GetSerializedSize () const
{
  return (3 * sizeof (uint32_t) * 3) /* positions and velocities */
         + sizeof (uint32_t) /* avg hop distance */
         + sizeof (uint32_t) /* originator */
         + sizeof (uint8_t) /* hop count */
         + sizeof (uint64_t) /* timestamp */;
}

void
HelloHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;
  // 中文说明：依次写入源位置、速度、发起者、上一跳位置、平均跳距、跳数与时间戳。
  WriteVector (i, m_sourcePosition);
  WriteVector (i, m_sourceVelocity);
  i.WriteHtonU32 (m_originator.Get ());
  WriteVector (i, m_lastHopPosition);
  i.WriteHtonU32 (EncodeFloat (m_averageHopDistance));
  i.WriteU8 (m_hopCount);
  WriteTime (i, m_timestamp);
}

uint32_t
HelloHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;
  // 中文说明：读取顺序需与Serialize保持一致，否则路由解析出的运动状态将出错。
  m_sourcePosition = ReadVector (i);
  m_sourceVelocity = ReadVector (i);
  m_originator = Ipv4Address (i.ReadNtohU32 ());
  m_lastHopPosition = ReadVector (i);
  m_averageHopDistance = DecodeFloat (i.ReadNtohU32 ());
  m_hopCount = i.ReadU8 ();
  m_timestamp = ReadTime (i);
  return GetSerializedSize ();
}

void
HelloHeader::Print (std::ostream &os) const
{
  os << "pos=" << m_sourcePosition
     << " vel=" << m_sourceVelocity
     << " lastHop=" << m_lastHopPosition
     << " avgHd=" << m_averageHopDistance
     << " hopCount=" << static_cast<uint32_t> (m_hopCount)
     << " ts=" << m_timestamp.GetSeconds ();
}

void
HelloHeader::SetSourcePosition (const Vector &pos)
{
  m_sourcePosition = pos;
}

Vector
HelloHeader::GetSourcePosition () const
{
  return m_sourcePosition;
}

void
HelloHeader::SetSourceVelocity (const Vector &vel)
{
  m_sourceVelocity = vel;
}

Vector
HelloHeader::GetSourceVelocity () const
{
  return m_sourceVelocity;
}

void
HelloHeader::SetOriginator (Ipv4Address origin)
{
  m_originator = origin;
}

Ipv4Address
HelloHeader::GetOriginator () const
{
  return m_originator;
}

void
HelloHeader::SetLastHopPosition (const Vector &pos)
{
  m_lastHopPosition = pos;
}

Vector
HelloHeader::GetLastHopPosition () const
{
  return m_lastHopPosition;
}

void
HelloHeader::SetAverageHopDistance (double avgHd)
{
  m_averageHopDistance = avgHd;
}

double
HelloHeader::GetAverageHopDistance () const
{
  return m_averageHopDistance;
}

void
HelloHeader::SetHopCount (uint8_t hopCount)
{
  m_hopCount = hopCount;
}

uint8_t
HelloHeader::GetHopCount () const
{
  return m_hopCount;
}

void
HelloHeader::SetTimestamp (Time timestamp)
{
  m_timestamp = timestamp;
}

Time
HelloHeader::GetTimestamp () const
{
  return m_timestamp;
}

NS_OBJECT_ENSURE_REGISTERED (GlobalUpdateHeader);

GlobalUpdateHeader::GlobalUpdateHeader ()
  : m_position (Vector (0.0, 0.0, 0.0)),
    m_velocity (Vector (0.0, 0.0, 0.0)),
    m_originator (),
    m_timestamp (Seconds (0))
{
}

TypeId
GlobalUpdateHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ladzrp::GlobalUpdateHeader")
    .SetParent<Header> ()
    .SetGroupName ("ladzrp")
    .AddConstructor<GlobalUpdateHeader> ();
  return tid;
}

TypeId
GlobalUpdateHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

uint32_t
GlobalUpdateHeader::GetSerializedSize () const
{
  return (3 * sizeof (uint32_t) * 2) + sizeof (uint32_t) + sizeof (uint64_t);
}

void
GlobalUpdateHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;
  // 中文说明：全局更新与HELLO共享向量编码工具，这里写入当前位置、速度、发起者地址与时间戳。
  WriteVector (i, m_position);
  WriteVector (i, m_velocity);
  i.WriteHtonU32 (m_originator.Get ());
  WriteTime (i, m_timestamp);
}

uint32_t
GlobalUpdateHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;
  m_position = ReadVector (i);
  m_velocity = ReadVector (i);
  m_originator = Ipv4Address (i.ReadNtohU32 ());
  m_timestamp = ReadTime (i);
  return GetSerializedSize ();
}

void
GlobalUpdateHeader::Print (std::ostream &os) const
{
  os << "pos=" << m_position
     << " vel=" << m_velocity
     << " ts=" << m_timestamp.GetSeconds ();
}

void
GlobalUpdateHeader::SetPosition (const Vector &pos)
{
  m_position = pos;
}

Vector
GlobalUpdateHeader::GetPosition () const
{
  return m_position;
}

void
GlobalUpdateHeader::SetVelocity (const Vector &vel)
{
  m_velocity = vel;
}

Vector
GlobalUpdateHeader::GetVelocity () const
{
  return m_velocity;
}

void
GlobalUpdateHeader::SetOriginator (Ipv4Address origin)
{
  m_originator = origin;
}

Ipv4Address
GlobalUpdateHeader::GetOriginator () const
{
  return m_originator;
}

void
GlobalUpdateHeader::SetTimestamp (Time timestamp)
{
  m_timestamp = timestamp;
}

Time
GlobalUpdateHeader::GetTimestamp () const
{
  return m_timestamp;
}

NS_OBJECT_ENSURE_REGISTERED (DataHeader);

DataHeader::DataHeader ()
  : m_destinationPosition (Vector (0.0, 0.0, 0.0)),
    m_destinationVelocity (Vector (0.0, 0.0, 0.0)),
    m_destinationTimestamp (Seconds (0)),
    m_senderPosition (Vector (0.0, 0.0, 0.0)),
    m_senderVelocity (Vector (0.0, 0.0, 0.0)),
    m_phaseFlags (0),
    m_hopCount (0)
{
}

TypeId
DataHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ladzrp::DataHeader")
    .SetParent<Header> ()
    .SetGroupName ("ladzrp")
    .AddConstructor<DataHeader> ();
  return tid;
}

TypeId
DataHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

uint32_t
DataHeader::GetSerializedSize () const
{
  return (3 * sizeof (uint32_t) * 4) + sizeof (uint64_t) + sizeof (uint8_t) * 2;
}

void
DataHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;
  // 中文说明：数据报头需要携带目的预测、发送节点状态以及阶段位，供多阶段转发决策使用。
  WriteVector (i, m_destinationPosition);
  WriteVector (i, m_destinationVelocity);
  WriteTime (i, m_destinationTimestamp);
  WriteVector (i, m_senderPosition);
  WriteVector (i, m_senderVelocity);
  i.WriteU8 (m_phaseFlags);
  i.WriteU8 (m_hopCount);
}

uint32_t
DataHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;
  // 中文说明：按照写入顺序读取所有字段，确保转发节点能正确计算预测位置与阶段状态。
  m_destinationPosition = ReadVector (i);
  m_destinationVelocity = ReadVector (i);
  m_destinationTimestamp = ReadTime (i);
  m_senderPosition = ReadVector (i);
  m_senderVelocity = ReadVector (i);
  m_phaseFlags = i.ReadU8 ();
  m_hopCount = i.ReadU8 ();
  return GetSerializedSize ();
}

void
DataHeader::Print (std::ostream &os) const
{
  os << "dstPos=" << m_destinationPosition
     << " dstVel=" << m_destinationVelocity
     << " dstTs=" << m_destinationTimestamp.GetSeconds ()
     << " senderPos=" << m_senderPosition
     << " senderVel=" << m_senderVelocity
     << " flags=" << static_cast<uint32_t> (m_phaseFlags)
     << " hop=" << static_cast<uint32_t> (m_hopCount);
}

void
DataHeader::SetDestinationPosition (const Vector &pos)
{
  m_destinationPosition = pos;
}

Vector
DataHeader::GetDestinationPosition () const
{
  return m_destinationPosition;
}

void
DataHeader::SetDestinationVelocity (const Vector &vel)
{
  m_destinationVelocity = vel;
}

Vector
DataHeader::GetDestinationVelocity () const
{
  return m_destinationVelocity;
}

void
DataHeader::SetDestinationTimestamp (Time timestamp)
{
  m_destinationTimestamp = timestamp;
}

Time
DataHeader::GetDestinationTimestamp () const
{
  return m_destinationTimestamp;
}

void
DataHeader::SetSenderPosition (const Vector &pos)
{
  m_senderPosition = pos;
}

Vector
DataHeader::GetSenderPosition () const
{
  return m_senderPosition;
}

void
DataHeader::SetSenderVelocity (const Vector &vel)
{
  m_senderVelocity = vel;
}

Vector
DataHeader::GetSenderVelocity () const
{
  return m_senderVelocity;
}

void
DataHeader::SetPhaseFlags (uint8_t flags)
{
  m_phaseFlags = flags;
}

uint8_t
DataHeader::GetPhaseFlags () const
{
  return m_phaseFlags;
}

void
DataHeader::SetHopCount (uint8_t hopCount)
{
  m_hopCount = hopCount;
}

uint8_t
DataHeader::GetHopCount () const
{
  return m_hopCount;
}

} // namespace ladzrp
} // namespace ns3
