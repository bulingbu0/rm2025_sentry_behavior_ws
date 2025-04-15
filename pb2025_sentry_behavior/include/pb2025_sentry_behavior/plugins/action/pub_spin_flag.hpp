#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_SPIN_FLAG_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_SPIN_FLAG_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "spin_flag_interfaces/msg/spin_flag.hpp"

namespace pb2025_sentry_behavior
{

class PubSpinFlagAction : public BT::RosTopicPubNode<spin_flag_interfaces::msg::SpinFlag>
{
public:
    PubSpinFlagAction(
        const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);
  
  static BT::PortsList providedPorts();

  bool setMessage(spin_flag_interfaces::msg::SpinFlag & msg) override;

private:
  rclcpp::Logger logger() { return node_->get_logger(); }
  rclcpp::Time now() { return node_->now(); }
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_SPIN_FLAG_HPP_
