#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TfRepublisher : public rclcpp::Node
{
public:
    TfRepublisher()
        : Node("tf_republisher")
    {
        // QoS 配置
        rclcpp::QoS tf_qos(10);
        rclcpp::QoS tf_static_qos(10);
        tf_static_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        tf_static_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

        rclcpp::QoS costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        // 订阅和发布 TF 话题
        tf_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/blue_standard_robot1/tf", tf_qos,
            std::bind(&TfRepublisher::tf_callback, this, std::placeholders::_1));

        tf_static_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/blue_standard_robot1/tf_static", tf_static_qos,
            std::bind(&TfRepublisher::tf_static_callback, this, std::placeholders::_1));

        tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", tf_qos);
        tf_static_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", tf_static_qos);

        // 订阅和发布 /global_costmap/costmap 话题（修正 QoS）
        global_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/blue_standard_robot1/global_costmap/costmap", costmap_qos,
            std::bind(&TfRepublisher::global_costmap_callback, this, std::placeholders::_1));

        global_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap", costmap_qos);
    }

private:
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        tf_publisher_->publish(*msg);
    }

    void tf_static_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        tf_static_publisher_->publish(*msg);
    }

    void global_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        global_costmap_pub_->publish(*msg);
    }

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_subscriber_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_publisher_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfRepublisher>());
    rclcpp::shutdown();
    return 0;
}
