#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_REACH_GOAL_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_REACH_GOAL_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose.hpp"

namespace pb2025_sentry_behavior
{
class IsReachGoalCondition : public BT::SimpleConditionNode
{
public:
    IsReachGoalCondition(const std::string & name, const BT::NodeConfig & config);
    
    static BT::PortsList providedPorts();

private:
    BT::NodeStatus checkIsReachGoal();

    rclcpp::Logger logger_ = rclcpp::get_logger("IsReachGoalCondition");
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
} // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_REACH_GOAL_HPP_