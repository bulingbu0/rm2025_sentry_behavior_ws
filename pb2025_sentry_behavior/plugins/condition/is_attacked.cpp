#include "pb2025_sentry_behavior/plugins/condition/is_attacked.hpp"
#include <rclcpp/logging.hpp>
#include "pb_rm_interfaces/msg/robot_status.hpp"

namespace pb2025_sentry_behavior
{

IsAttackedCondition::IsAttackedCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsAttackedCondition::checkIsAttacked, this), config)
{
}

BT::NodeStatus IsAttackedCondition::checkIsAttacked()
{
  auto msg = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }
  int current_hp = msg->current_hp;
  RCLCPP_WARN(logger_, "current_hp: %d", current_hp);
  static int previous_hp = -1;

  if (previous_hp == -1) {
    previous_hp = current_hp;
    RCLCPP_ERROR(logger_, "Initialize previous_hp: %d", previous_hp);
    return BT::NodeStatus::FAILURE;
  }

  bool hp_decreased = current_hp < previous_hp;
  bool is_attacked = hp_decreased && (msg->hp_deduction_reason == 0);

  RCLCPP_ERROR(logger_, "current_hp: %d, previous_hp: %d", current_hp, previous_hp);
  previous_hp = current_hp;

  if (is_attacked) {
    RCLCPP_ERROR(logger_, "Armor hit detected");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

BT::PortsList IsAttackedCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "key_port", "{@referee_robotStatus}", "RobotStatus BlackBoard port"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsAttackedCondition>("IsAttacked");
}
