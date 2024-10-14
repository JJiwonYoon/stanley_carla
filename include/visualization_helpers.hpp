// visualization_helpers.hpp

#ifndef VISUALIZATION_HELPERS_HPP
#define VISUALIZATION_HELPERS_HPP

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <memory>
#include <vector>

class VisualizationHelpers
{
public:
    VisualizationHelpers(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher)
        : marker_publisher_(marker_publisher) {}

    void publish_current_position(const std::shared_ptr<double>& odom_x, const std::shared_ptr<double>& odom_y, rclcpp::Clock::SharedPtr clock)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = clock->now();
        marker.ns = "current_position";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = *odom_x;
        marker.pose.position.y = *odom_y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_publisher_->publish(marker);
    }

    void publish_markers(const std::vector<double>& x, const std::vector<double>& y, rclcpp::Clock::SharedPtr clock)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = clock->now();
        marker.ns = "current_slice_path";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        for (size_t i = 0; i < x.size(); ++i) {
            geometry_msgs::msg::Point p;
            p.x = x[i];
            p.y = y[i];
            p.z = 0.0;
            marker.points.push_back(p);
        }

        marker_publisher_->publish(marker);
    }

    void publish_path(const std::shared_ptr<std::vector<double>>& content1,
                    const std::shared_ptr<std::vector<double>>& content2,
                    rclcpp::Clock::SharedPtr clock)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = clock->now();
        marker.ns = "path";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        for (size_t i = 0; i < content1->size(); ++i)
        {
            geometry_msgs::msg::Point p;
            p.x = content1->at(i);
            p.y = content2->at(i);
            p.z = 0.0;
            marker.points.push_back(p);
        }

        marker_publisher_->publish(marker);
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

#endif // VISUALIZATION_HELPERS_HPP
