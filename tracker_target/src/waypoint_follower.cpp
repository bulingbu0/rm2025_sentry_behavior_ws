#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class WaypointFollowerClient : public rclcpp::Node
{
public:
    WaypointFollowerClient()
        : Node("waypoint_follower_client")
    {
        // 创建动作客户端
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
            this, "follow_waypoints");

        // 等待服务器启动
        while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "等待动作服务器启动...");
        }

        RCLCPP_INFO(this->get_logger(), "动作服务器已启动，准备发送航点...");
    }

    void send_goal()
    {
        // 创建目标消息
        auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();

        // 定义航点
        geometry_msgs::msg::PoseStamped waypoint1;
        waypoint1.header.frame_id = "map";
        waypoint1.pose.position.x = 3.63;
        waypoint1.pose.position.y = 0.77;
        waypoint1.pose.orientation.w = 1.0;

        geometry_msgs::msg::PoseStamped waypoint2;
        waypoint2.header.frame_id = "map";
        waypoint2.pose.position.x = 3.76;
        waypoint2.pose.position.y = -1.53;
        waypoint2.pose.orientation.w = 1.0;

        geometry_msgs::msg::PoseStamped waypoint3;
        waypoint3.header.frame_id = "map";
        waypoint3.pose.position.x = 1.23;
        waypoint3.pose.position.y = 1.01;
        waypoint3.pose.orientation.w = 1.0;

        // 将航点添加到目标消息
        goal_msg.poses.push_back(waypoint1);
        goal_msg.poses.push_back(waypoint2);
        goal_msg.poses.push_back(waypoint3);

        // 发送目标
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "航点跟随成功！");
            } else {
                RCLCPP_ERROR(this->get_logger(), "航点跟随失败！");
            }
        };
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<WaypointFollowerClient>();
    client->send_goal();
    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}
