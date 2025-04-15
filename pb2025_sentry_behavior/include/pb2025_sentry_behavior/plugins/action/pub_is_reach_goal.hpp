#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_IS_REACH_GOAL_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_IS_REACH_GOAL_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "auto_aim_interfaces/msg/is_reach_goal.hpp"

namespace pb2025_sentry_behavior
{
class PubIsReachGoalAction : public BT::RosTopicPubNode<auto_aim_interfaces::msg::IsReachGoal>
{
public:
    PubIsReachGoalAction(
        const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);
    static BT::PortsList providedPorts();
    bool setMessage(auto_aim_interfaces::msg::IsReachGoal & msg) override;

private:
    rclcpp::Logger logger() { return node_->get_logger(); }
    rclcpp::Time now() { return node_->now(); }
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_IS_REACH_GOAL_HPP_