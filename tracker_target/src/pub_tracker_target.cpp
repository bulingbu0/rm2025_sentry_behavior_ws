#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <chrono>
#include <thread>

class TargetPublisher : public rclcpp::Node
{
public:
    TargetPublisher()
        : Node("target_publisher"), counter_(0)
    {
        // 创建话题发布者
        target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/tracker/target", 10);

        // 设置定时器，每5秒发布一次目标点
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&TargetPublisher::publish_target, this)
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;

    // 发布目标点
    void publish_target()
    {
        // 目标点数组
        std::vector<geometry_msgs::msg::Point> points;
        geometry_msgs::msg::Point point1;
        point1.x = 1.0;
        point1.y = -1.0;
        point1.z = 0.0;
        points.push_back(point1);

        geometry_msgs::msg::Point point2;
        point2.x = 1.0;
        point2.y = 1.0;
        point2.z = 0.0;
        points.push_back(point2);

        geometry_msgs::msg::Point point3;
        point3.x = 5.28;
        point3.y = -3.69;
        point3.z = 0.0;
        points.push_back(point3);

        // 创建PointStamped消息
        geometry_msgs::msg::PointStamped target_msg;
        target_msg.header.stamp = this->get_clock()->now();
        target_msg.header.frame_id = "base_link";

        // 循环发送目标点
        target_msg.point = points[counter_];

        // 发布目标点
        RCLCPP_INFO(this->get_logger(), "Publishing target: (%.2f, %.2f)", target_msg.point.x, target_msg.point.y);
        target_pub_->publish(target_msg);

        // 更新计数器，循环选择目标点
        counter_ = (counter_ + 1) % points.size();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetPublisher>());
    rclcpp::shutdown();
    return 0;
}
