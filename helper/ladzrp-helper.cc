#include "ladzrp-helper.h"
#include "ns3/ladzrp-routing-protocol.h"
#include "ns3/node-list.h"
#include "ns3/names.h"
#include "ns3/ipv4-list-routing.h"

namespace ns3 {
LADZRPHelper::~LADZRPHelper ()
{
}

LADZRPHelper::LADZRPHelper () : Ipv4RoutingHelper ()
{
  m_agentFactory.SetTypeId ("ns3::ladzrp::RoutingProtocol");
}

LADZRPHelper*
LADZRPHelper::Copy (void) const
{
  return new LADZRPHelper (*this);
}

Ptr<Ipv4RoutingProtocol>
LADZRPHelper::Create (Ptr<Node> node) const
{
  Ptr<ladzrp::RoutingProtocol> agent = m_agentFactory.Create<ladzrp::RoutingProtocol> ();
  node->AggregateObject (agent);
  return agent;
}

void
LADZRPHelper::Set (std::string name, const AttributeValue &value)
{
  m_agentFactory.Set (name, value);
}

}
