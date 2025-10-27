#ifndef LADZRP_PACKET_H
#define LADZRP_PACKET_H

#include <cstdint>
#include "ns3/header.h"
#include "ns3/ipv4-address.h"
#include "ns3/vector.h"
#include "ns3/nstime.h"

// 中文说明：本文件定义了LA-DZRP协议中所有自定义报文头部，
// 包括控制消息（HELLO、全局更新等）与数据转发过程中携带的
// 定位信息。通过这些结构体，路由协议可以在不同阶段传递节
// 点的位置信息、速度向量以及中继状态，保证TTLM与MPF机制可
// 靠运行。

namespace ns3 {
namespace ladzrp {

// 中文说明：区分控制与数据报文的类型，用于RecvControl中快速
// 解析对应的头部并调用正确的处理流程。
enum class MessageType : uint8_t
{
  HELLO = 0,
  GLOBAL_UPDATE = 1,
  UNICAST_REQUEST = 2,
  DATA = 3,
  BACKUP_DATA = 4
};

// 中文说明：所有控制报文的公共头部，携带消息类型、TTL与序列号。
class MessageHeader : public Header
{
public:
  MessageHeader (MessageType type = MessageType::HELLO, uint8_t ttl = 0, uint32_t sequenceNumber = 0);

  static TypeId GetTypeId (void);
  TypeId GetInstanceTypeId () const override;
  uint32_t GetSerializedSize () const override;
  void Serialize (Buffer::Iterator start) const override;
  uint32_t Deserialize (Buffer::Iterator start) override;
  void Print (std::ostream &os) const override;

  void SetMessageType (MessageType type);
  MessageType GetMessageType () const;

  void SetTtl (uint8_t ttl);
  uint8_t GetTtl () const;

  void SetSequenceNumber (uint32_t seq);
  uint32_t GetSequenceNumber () const;

private:
  MessageType m_type;
  uint8_t m_ttl;
  uint32_t m_sequenceNumber;
};

// 中文说明：HELLO报文体，记录源节点运动状态、平均跳距等局部
// 维护所需的信息。
class HelloHeader : public Header
{
public:
  HelloHeader ();

  static TypeId GetTypeId (void);
  TypeId GetInstanceTypeId () const override;
  uint32_t GetSerializedSize () const override;
  void Serialize (Buffer::Iterator start) const override;
  uint32_t Deserialize (Buffer::Iterator start) override;
  void Print (std::ostream &os) const override;

  void SetSourcePosition (const Vector &pos);
  Vector GetSourcePosition () const;

  void SetSourceVelocity (const Vector &vel);
  Vector GetSourceVelocity () const;

  void SetOriginator (Ipv4Address origin);
  Ipv4Address GetOriginator () const;

  void SetLastHopPosition (const Vector &pos);
  Vector GetLastHopPosition () const;

  void SetAverageHopDistance (double avgHd);
  double GetAverageHopDistance () const;

  void SetHopCount (uint8_t hopCount);
  uint8_t GetHopCount () const;

  void SetTimestamp (Time timestamp);
  Time GetTimestamp () const;

private:
  Vector m_sourcePosition;
  Vector m_sourceVelocity;
  Ipv4Address m_originator;
  Vector m_lastHopPosition;
  double m_averageHopDistance;
  uint8_t m_hopCount;
  Time m_timestamp;
};

// 中文说明：全局位置更新报文体，向网络广播节点的粗粒度方向信息。
class GlobalUpdateHeader : public Header
{
public:
  GlobalUpdateHeader ();

  static TypeId GetTypeId (void);
  TypeId GetInstanceTypeId () const override;
  uint32_t GetSerializedSize () const override;
  void Serialize (Buffer::Iterator start) const override;
  uint32_t Deserialize (Buffer::Iterator start) override;
  void Print (std::ostream &os) const override;

  void SetPosition (const Vector &pos);
  Vector GetPosition () const;

  void SetVelocity (const Vector &vel);
  Vector GetVelocity () const;

  void SetOriginator (Ipv4Address origin);
  Ipv4Address GetOriginator () const;

  void SetTimestamp (Time timestamp);
  Time GetTimestamp () const;

private:
  Vector m_position;
  Vector m_velocity;
  Ipv4Address m_originator;
  Time m_timestamp;
};

// 中文说明：数据报文头部，包含目的节点预测信息、当前发送节点
// 位姿以及当前处于的转发阶段（0=区内，1=区间，2=备份广播）。
class DataHeader : public Header
{
public:
  DataHeader ();

  static TypeId GetTypeId (void);
  TypeId GetInstanceTypeId () const override;
  uint32_t GetSerializedSize () const override;
  void Serialize (Buffer::Iterator start) const override;
  uint32_t Deserialize (Buffer::Iterator start) override;
  void Print (std::ostream &os) const override;

  void SetDestinationPosition (const Vector &pos);
  Vector GetDestinationPosition () const;

  void SetDestinationVelocity (const Vector &vel);
  Vector GetDestinationVelocity () const;

  void SetDestinationTimestamp (Time timestamp);
  Time GetDestinationTimestamp () const;

  void SetSenderPosition (const Vector &pos);
  Vector GetSenderPosition () const;

  void SetSenderVelocity (const Vector &vel);
  Vector GetSenderVelocity () const;

  void SetPhaseFlags (uint8_t flags);
  uint8_t GetPhaseFlags () const;

  void SetHopCount (uint8_t hopCount);
  uint8_t GetHopCount () const;

private:
  Vector m_destinationPosition;
  Vector m_destinationVelocity;
  Time m_destinationTimestamp;
  Vector m_senderPosition;
  Vector m_senderVelocity;
  uint8_t m_phaseFlags;
  uint8_t m_hopCount;
};

} // namespace ladzrp
} // namespace ns3

#endif // LADZRP_PACKET_H
