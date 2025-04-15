#include "pb2025_sentry_behavior/plugins/action/pub_is_reach_goal.hpp"
#include <behaviortree_cpp/basic_types.h>
#include "rclcpp/logging.hpp"

namespace pb2025_sentry_behavior
{
PubIsReachGoalAction::PubIsReachGoalAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
    : BT::RosTopicPubNode<auto_aim_interfaces::msg::IsReachGoal>(name, conf, params)
    {
    }
bool PubIsReachGoalAction::setMessage(auto_aim_interfaces::msg::IsReachGoal &msg)
{
    auto is_goal = getInput<bool>("is_reach_goal");
    if (!is_goal)
    {
        RCLCPP_ERROR(logger(), "Missing required input: is_goal ");
        return false;
    }
    msg.is_reach_goal = is_goal.value();
    RCLCPP_INFO(logger(), "Publishing is reach goal: %d", msg.is_reach_goal);
    return true;
}
BT::PortsList PubIsReachGoalAction::providedPorts()
{
    return BT::PortsList{
        BT::InputPort<bool>("is_reach_goal", "false", "Is reach goal"),
        BT::InputPort<std::string>("topic_name", "is_reach_goal", "is_reach_goal topic name"),
    };
}
} // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PubIsReachGoalAction, "PubIsReachGoal");