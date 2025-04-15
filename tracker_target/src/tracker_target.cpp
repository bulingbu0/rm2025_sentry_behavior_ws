#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <vector>
#include <chrono>
#include <functional>
#include <thread>

class NavigateToGoal : public rclcpp::Node
{
public:
    NavigateToGoal()
        : Node("navigate_to_goal"), current_goal_index_(0)
    {
        goal_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");

        while (!goal_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        initialize_goals();
        
        if (!goals_.empty()) {
            send_goal(goals_[current_goal_index_]);
        }
    }

private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr goal_client_;
    std::vector<geometry_msgs::msg::PoseStamped> goals_;
    size_t current_goal_index_;

    // 初始化目标点列表
    void initialize_goals()
    {
        // 第一个目标点
        geometry_msgs::msg::PoseStamped target_pose1;
        target_pose1.header.frame_id = "map";
        target_pose1.pose.position.x = 5.43;
        target_pose1.pose.position.y = -0.60;
        target_pose1.pose.orientation.w = 1.0;
        goals_.push_back(target_pose1);

        // 第二个目标点
        // geometry_msgs::msg::PoseStamped target_pose2;
        // target_pose2.header.frame_id = "map";
        // target_pose2.pose.position.x = 4.87;
        // target_pose2.pose.position.y = -4.87;
        // target_pose2.pose.orientation.w = 0.5;
        // goals_.push_back(target_pose2);

    }

    // 封装后的发送目标点函数
    void send_goal(const geometry_msgs::msg::PoseStamped &target_pose)
    {
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = target_pose;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
                this->result_callback(result);
            };
        send_goal_options.feedback_callback = 
            [this](
                rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
                const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
                this->feedback_callback(feedback);
            };

        RCLCPP_INFO(this->get_logger(), "Sending goal %zu of %zu", 
                   current_goal_index_ + 1, goals_.size());
        goal_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void result_callback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal %zu reached!", current_goal_index_ + 1);
                
                // 停留5秒钟
                std::this_thread::sleep_for(std::chrono::seconds(5));
                
                // 发送下一个目标点
                current_goal_index_++;
                if (current_goal_index_ < goals_.size()) {
                    send_goal(goals_[current_goal_index_]);
                } else {
                    RCLCPP_INFO(this->get_logger(), "All goals completed!");
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal %zu aborted", current_goal_index_ + 1);
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal %zu canceled", current_goal_index_ + 1);
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
    }

    void feedback_callback(
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Current position: x=%.2f, y=%.2f, Remaining distance: %.2f",
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y,
            feedback->distance_remaining);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigateToGoal>());
    rclcpp::shutdown();
    return 0;
}