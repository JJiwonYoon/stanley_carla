#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <stdio.h>


class carlaSubscriber : public rclcpp::Node
{
public:
    carlaSubscriber() : Node("stanley_main")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/carla/ego_vehicle/odometry", 10, std::bind(&carlaSubscriber::odom_callback, this, std::placeholders::_1));
        // publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("filtered_lidar_1", 10);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        float x = msg->pose.pose.position.x;
        float y = msg->pose.pose.position.y;
        std::cout << "x: " << x << ", y: " << y << std::endl;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<carlaSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}