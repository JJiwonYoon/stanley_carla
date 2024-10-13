#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>
#include <utility>
#include "frenet_optimal_trajectory.hpp"
#include "cubic_spline_planner.hpp"

const double k = 3.0;  // control gain
const double L = 2.0;  // [m] Wheel base of vehicle
const double v = 12;  // [m/s] Vehicle speed
const double max_steer = M_PI / 6.0;  // [rad] max steering angle

class carlaSubscriber : public rclcpp::Node
{
public:
    carlaSubscriber() : Node("stanley_main")
    {
        carla_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "/carla/ego_vehicle/odometry", 10, std::bind(&carlaSubscriber::odom_callback, this, std::placeholders::_1));

        carla_object_x = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/avg_x", 10, std::bind(&carlaSubscriber::object_x_callback, this, std::placeholders::_1));

        carla_object_y = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/avg_y", 10, std::bind(&carlaSubscriber::object_y_callback, this, std::placeholders::_1));

        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        odom_x = std::make_shared<double>(0.0);
        odom_y = std::make_shared<double>(0.0);
        current_yaw = std::make_shared<double>(0.0);
        content1 = std::make_shared<std::vector<double>>();
        content2 = std::make_shared<std::vector<double>>();
        content3 = std::make_shared<std::vector<double>>();

        whileThread = std::thread(&carlaSubscriber::runWhileLoop, this);
    }

    ~carlaSubscriber()
    {
        if (whileThread.joinable())
        {
            whileThread.join();
        }
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        *odom_x = msg->pose.pose.position.x;
        *odom_y = msg->pose.pose.position.y;

        *current_yaw = quaternion_to_euler(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    }

    void object_x_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (!msg->data.empty())
        {
            std::cout << "x values: ";
            for (float value : msg->data)
            {
                std::cout << value << " ";
            }
            std::cout << std::endl;
        }
        else
        {
            std::cout << "Received empty data array." << std::endl;
        }
    }

    void object_y_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (!msg->data.empty())
        {
            std::cout << "y values: ";
            for (float value : msg->data)
            {
                std::cout << value << " ";
            }
            std::cout << std::endl;
        }
    }

    void readFileToVector(const std::string& filePath, std::shared_ptr<std::vector<double>>& content)
    {
        std::ifstream file(filePath);

        if (!file.is_open())
        {
            std::cerr << "파일을 열 수 없습니다: " << filePath << std::endl;
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            double value;
            while (ss >> value) {
                content->push_back(value);
            }
        }

        file.close();
    }

    void readFileToVectors(const std::string& filePath1, const std::string& filePath2, const std::string& filePath3, std::shared_ptr<std::vector<double>>& content1, std::shared_ptr<std::vector<double>>& content2, std::shared_ptr<std::vector<double>>& content3)
    {
        readFileToVector(filePath1, content1);
        readFileToVector(filePath2, content2);
        readFileToVector(filePath3, content3);
    }

    double quaternion_to_euler(double x, double y, double z, double w)
    {
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    std::pair<int, double> calc_target_index(double x, double y, double yaw, const std::vector<double>& cx, const std::vector<double>& cy)
    {
        double fx = x + L * std::cos(yaw);
        double fy = y + L * std::sin(yaw);

        std::vector<double> dx(cx.size()), dy(cy.size());
        for (size_t i = 0; i < cx.size(); ++i)
        {
            dx[i] = fx - cx[i];
            dy[i] = fy - cy[i];
        }

        std::vector<double> d(cx.size());
        for (size_t i = 0; i < cx.size(); ++i)
        {
            d[i] = std::hypot(dx[i], dy[i]);
        }

        auto min_it = std::min_element(d.begin(), d.end());
        int target_idx = std::distance(d.begin(), min_it);

        std::vector<double> front_axle_vec = {-std::cos(yaw + M_PI / 2), -std::sin(yaw + M_PI / 2)};
        double error_front_axle = dx[target_idx] * front_axle_vec[0] + dy[target_idx] * front_axle_vec[1];

        return {target_idx, error_front_axle};
    }

    std::pair<double, int> stanley_control(double x, double y, double yaw, const std::vector<double>& cx, const std::vector<double>& cy, const std::vector<double>& cyaw, int last_target_idx)
    {
        auto [current_target_idx, error_front_axle] = calc_target_index(x, y, yaw, cx, cy);

        if (last_target_idx >= current_target_idx)
        {
            current_target_idx = last_target_idx;
        }

        double theta_e = normalize_angle(cyaw[current_target_idx] - yaw);
        double theta_d = std::atan2(k * error_front_axle, v);
        double delta = theta_e + theta_d;
        delta = std::max(-max_steer, std::min(delta, max_steer));
        return {delta, current_target_idx};
    }

    void publish_current_position()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
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

    void publish_path()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
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

    void runWhileLoop()
    {
        std::string filePath1 = "/home/oskar/Downloads/first_x.txt"; // 읽어올 파일 경로
        std::string filePath2 = "/home/oskar/Downloads/first_y.txt"; // 읽어올 파일 경로
        std::string filePath3 = "/home/oskar/Downloads/first_yaw.txt"; // 읽어올 파일 경로
        int last_target_idx = 0;
        readFileToVectors(filePath1, filePath2, filePath3, content1, content2, content3);
        auto [rx, ry, ryaw, rk, csp] = generate_target_course(*content1, *content2);
        publish_path();

        while (rclcpp::ok())
        {
            auto [delta, current_target_idx] = stanley_control(*odom_x, *odom_y, *current_yaw, *content1, *content2, *content3, last_target_idx);
            std::cout << "Calculated steering angle (delta): " << delta << std::endl;
            std::cout << "Current target index: " << current_target_idx << std::endl;
            auto cmd_vel_msg = geometry_msgs::msg::Twist();
            cmd_vel_msg.linear.x = 2.0;
            cmd_vel_msg.angular.z = delta;
            cmd_publisher_->publish(cmd_vel_msg);

            publish_current_position();

            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100밀리초 대기
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr carla_odometry;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr carla_object_x;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr carla_object_y;
    std::thread whileThread;

    std::shared_ptr<double> odom_x;
    std::shared_ptr<double> odom_y;
    std::shared_ptr<double> current_yaw;
    std::shared_ptr<std::vector<double>> content1;
    std::shared_ptr<std::vector<double>> content2;
    std::shared_ptr<std::vector<double>> content3;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<carlaSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}