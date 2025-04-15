#include "pb2025_sentry_behavior/plugins/condition/is_detect_enemy.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <auto_aim_interfaces/msg/detail/target__struct.hpp>

namespace pb2025_sentry_behavior
{

IsDetectEnemyCondition::IsDetectEnemyCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsDetectEnemyCondition::checkEnemy, this), config)
{
}

BT::PortsList IsDetectEnemyCondition::providedPorts()
{
  return {
    BT::InputPort<auto_aim_interfaces::msg::Target>(
      "key_port", "{@tracker_target}", "Vision tracking port on blackboard"),
    // BT::InputPort<std::vector<int>>(
    //   "armor_id", "1;2;3;4;5;7",
    //   "Expected id of armors. Multiple numbers should be separated by the character `;` in Groot2"),
    // BT::InputPort<float>("max_distance", 8.0, "Distance to enemy target"),
    BT::InputPort<bool>("tracking", false, "is detect enemy")
  };
}

BT::NodeStatus IsDetectEnemyCondition::checkEnemy()
{
  // std::vector<int> expected_armor_ids;
  // float max_distance;
  auto msg = getInput<auto_aim_interfaces::msg::Target>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "tracking message is not available");
    return BT::NodeStatus::FAILURE;
  }

  // getInput("armor_id", expected_armor_ids);
  // getInput("max_distance", max_distance);
  bool is_detect_enemy;
  getInput("tracking", is_detect_enemy);

  // for (const auto & armor : msg->armors) {
  //   float distance_to_enemy = std::hypot(armor.pose.position.x, armor.pose.position.y);

  //   if (armor.number.empty()) {
  //     continue;
  //   }
  //   int armor_id = std::stoi(armor.number);
  //   const bool is_armor_id_match =
  //     std::find(expected_armor_ids.begin(), expected_armor_ids.end(), armor_id) !=
  //     expected_armor_ids.end();

  //   const bool is_within_distance = (distance_to_enemy <= max_distance);

  //   if (is_armor_id_match && is_within_distance) {
  //     return BT::NodeStatus::SUCCESS;
  //   }
  // }
  is_detect_enemy = msg->tracking;
  if (is_detect_enemy) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsDetectEnemyCondition>("IsDetectEnemy");
}
