#include "rclcpp/rclcpp.hpp"
#include "spin_flag_interfaces/msg/spin_flag.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "auto_aim_interfaces/msg/is_reach_goal.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription.hpp>

class SubMsgs : public rclcpp::Node
{
public:
  SubMsgs() : Node("sub_msgs")
  {
    // spin_flag_sub_ = this->create_subscription<spin_flag_interfaces::msg::SpinFlag>(
    //   "spin_flag", 10, 
    //   std::bind(&SubMsgs::spinflagCallback, this, std::placeholders::_1));
    // robot_status_ = this->create_subscription<pb_rm_interfaces::msg::RobotStatus>(
    //   "referee/robot_status", 10, 
    //   std::bind(&SubMsgs::robotstatusCallback, this, std::placeholders::_1));
    // is_reach_goal_ = this->create_subscription<auto_aim_interfaces::msg::IsReachGoal>(
    //   "is_reach_goal", 10,
    //   std::bind(&SubMsgs::isreachgoalCallback, this, std::placeholders::_1));
    // is_goal_ = this->create_subscription<auto_aim_interfaces::msg::IsReachGoal>(
    //   "is_goal", 10,
    //   std::bind(&SubMsgs::isgoalCallback, this, std::placeholders::_1));
    cmd_vel_chassis_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_chassis", 10,
      std::bind(&SubMsgs::cmdvelchassisCallback, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&SubMsgs::cmdvelCallback, this, std::placeholders::_1));

  }

private:
  // void spinflagCallback(const spin_flag_interfaces::msg::SpinFlag::SharedPtr msg) const
  // {
  //   RCLCPP_INFO(this->get_logger(), "Received spin flag: %d", msg->spin_flag);
  // }  
  // void robotstatusCallback(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg) const
  // {
  //   RCLCPP_INFO(this->get_logger(), "Received RobotStatus1: %d", msg->is_hp_deduced);
  //   RCLCPP_INFO(this->get_logger(), "Received RobotStatus2: %d", msg->ARMOR_HIT);
  // }
  // void isreachgoalCallback(const auto_aim_interfaces::msg::IsReachGoal::SharedPtr msg) const
  // {
  //   RCLCPP_INFO(this->get_logger(), "Received is reach goal: %d", msg->is_reach_goal);
  // }
  // void isgoalCallback(const auto_aim_interfaces::msg::IsReachGoal::SharedPtr msg) const
  // {
  //   RCLCPP_INFO(this->get_logger(), "Received is goal: %d", msg->is_reach_goal);
  // }
  void cmdvelchassisCallback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "cmd vel chassis x: %f y: %f", msg->linear.x, msg->linear.y);
  }
  void cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "cmd vel         x: %f y: %f", msg->linear.x, msg->linear.y);
    RCLCPP_INFO(this->get_logger(), "                           ");
  }
  // rclcpp::Subscription<spin_flag_interfaces::msg::SpinFlag>::SharedPtr spin_flag_sub_;
  // rclcpp::Subscription<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_;
  // rclcpp::Subscription<auto_aim_interfaces::msg::IsReachGoal>::SharedPtr is_reach_goal_;
  // rclcpp::Subscription<auto_aim_interfaces::msg::IsReachGoal>::SharedPtr is_goal_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubMsgs>());
  rclcpp::shutdown();
  return 0;
}