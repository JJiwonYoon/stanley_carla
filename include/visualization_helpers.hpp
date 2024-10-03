#ifndef VISUALIZATION_HELPERS_HPP
#define VISUALIZATION_HELPERS_HPP

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <vector>
#include <memory>

class VisualizationHelpers
{
public:
    VisualizationHelpers(rclcpp::Node::SharedPtr node)
    {
        path_marker_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker_path", 10);
        position_marker_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker_position", 10);
    }

    void publish_path(const std::shared_ptr<std::vector<double>>& content1, const std::shared_ptr<std::vector<double>>& content2, rclcpp::Time now)
    {
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = now;
        path_marker.ns = "path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.1;
        path_marker.color.a = 1.0;
        path_marker.color.r = 1.0;
        path_marker.color.g = 0.0;
        path_marker.color.b = 0.0;

        for (size_t i = 0; i < content1->size(); ++i)
        {
            geometry_msgs::msg::Point p;
            p.x = content1->at(i);
            p.y = content2->at(i);
            p.z = 0.0;
            path_marker.points.push_back(p);
        }

        path_marker_publisher_->publish(path_marker);
    }

    void publish_position(double odom_x, double odom_y, rclcpp::Time now)
    {
        visualization_msgs::msg::Marker position_marker;
        position_marker.header.frame_id = "map";
        position_marker.header.stamp = now;
        position_marker.ns = "position";
        position_marker.id = 1;
        position_marker.type = visualization_msgs::msg::Marker::SPHERE;
        position_marker.action = visualization_msgs::msg::Marker::ADD;
        position_marker.scale.x = 0.5;
        position_marker.scale.y = 0.5;
        position_marker.scale.z = 0.5;
        position_marker.color.a = 1.0;
        position_marker.color.r = 0.0;
        position_marker.color.g = 1.0;
        position_marker.color.b = 0.0;
        position_marker.pose.position.x = odom_x;
        position_marker.pose.position.y = odom_y;
        position_marker.pose.position.z = 0.0;

        position_marker_publisher_->publish(position_marker);
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr position_marker_publisher_;
};

#endif // VISUALIZATION_HELPERS_HPP