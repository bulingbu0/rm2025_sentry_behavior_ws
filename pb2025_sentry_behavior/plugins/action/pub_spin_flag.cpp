#include "pb2025_sentry_behavior/plugins/action/pub_spin_flag.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <rclcpp/logging.hpp>

namespace pb2025_sentry_behavior 
{
PubSpinFlagAction::PubSpinFlagAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
    : RosTopicPubNode<spin_flag_interfaces::msg::SpinFlag>(name, conf, params)
    {
    }

bool PubSpinFlagAction::setMessage(spin_flag_interfaces::msg::SpinFlag &msg)
{
    auto flag = getInput<uint8_t>("spin_flag");
    if (!flag)
    {
        RCLCPP_ERROR(logger(), "Missing required input: spin_flag ");
        return false;
    }
    // msg.spin_flag = flag->spin_flag;
    msg.spin_flag = flag.value();
    RCLCPP_INFO(logger(), "Publishing spin flag: %d", msg.spin_flag);
    return true;
}
BT::PortsList PubSpinFlagAction::providedPorts()
{
    return BT::PortsList{
        BT::InputPort<uint8_t>("spin_flag", "0", "Spin flag"),
        BT::InputPort<std::string>("topic_name", "spin_flag", "spin_flag topic name"),
    };
}

} // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PubSpinFlagAction, "PubSpinFlag");