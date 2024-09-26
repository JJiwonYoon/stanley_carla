#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>

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

        whileThread = std::thread(&carlaSubscriber::runWhileLoop, this);
        // publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("filtered_lidar_1", 10);
    }

    ~carlaSubscriber()
    {
        if (whileThread.joinable())
        {
            whileThread.join(); // 프로그램 종료 전에 while 루프 스레드 종료 대기
        }
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        float odom_x = msg->pose.pose.position.x;
        float odom_y = msg->pose.pose.position.y;
        std::cout << "x: " << odom_x << ", y: " << odom_y << std::endl;
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
        else
        {
            std::cout << "Received empty data array." << std::endl;
        }
    }

    void readFileToString(const std::string& filePath, std::string& content)
    {
        std::ifstream file(filePath); // 파일을 엽니다.

        if (!file.is_open())
        {
            std::cerr << "파일을 열 수 없습니다: " << filePath << std::endl;
            return;  // 파일 열기 실패 시 함수 종료
        }

        std::stringstream buffer;
        buffer << file.rdbuf();  // 파일 내용을 버퍼에 읽어들입니다.

        content = buffer.str();  // 버퍼 내용을 content에 저장

        file.close();  // 파일을 닫습니다.
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

    void runWhileLoop()
    {
        float path_x, path_y, path_yaw;
        while (1)
        {
            std::cout << "Running while loop..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1)); // 1초 대기
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr carla_odometry;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr carla_object_x;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr carla_object_y;
    std::thread whileThread;
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