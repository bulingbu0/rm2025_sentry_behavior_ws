#include "pb2025_sentry_behavior/plugins/condition/is_reach_goal.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>

namespace pb2025_sentry_behavior
{
IsReachGoalCondition::IsReachGoalCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsReachGoalCondition::checkIsReachGoal, this), config)
{
}
BT::NodeStatus IsReachGoalCondition::checkIsReachGoal()
{
    auto current_pose = getInput<nav_msgs::msg::Odometry>("current_pose");
    if (!current_pose) {
        RCLCPP_ERROR(logger_, "current_pose is not available");
        return BT::NodeStatus::FAILURE;
    }
    // auto goal_pose = getInput<geometry_msgs::msg::Pose>("goal_pose");
    // if (!goal_pose) {
    //     RCLCPP_ERROR(logger_, "goal_pose is not available");
    //     return BT::NodeStatus::FAILURE;
    // }
    double threshold = 0.2;
    if (auto thresh = getInput<double>("threshold")) {
        threshold = *thresh;
    }
    // double dx = current_pose->position.x - goal_pose->position.x;
    // double dy = current_pose->position.y - goal_pose->position.y;
    double dx = current_pose->pose.pose.position.x - 4.73;
    double dy = current_pose->pose.pose.position.y - (-1.78);
    double distance = std::hypot(dx, dy);
    RCLCPP_INFO(logger_, "distance: %f", distance);

    if (distance < threshold) {
        RCLCPP_INFO(logger_, "Reach goal");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}
BT::PortsList IsReachGoalCondition::providedPorts()
{
    return {
        BT::InputPort<nav_msgs::msg::Odometry>(
            "current_pose","{@current_pose}", "Current pose BlackBoard port"),
        // BT::InputPort<geometry_msgs::msg::Pose>(
        //     "goal_pose","{@goal_pose}", "Goal pose BlackBoard port"),
            BT::InputPort<double>(
            "threshold","0.2", "Threshold")
    };
    }
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<pb2025_sentry_behavior::IsReachGoalCondition>("IsReachGoal");
}