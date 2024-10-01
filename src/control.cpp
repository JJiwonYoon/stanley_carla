#include "rclcpp/rclcpp.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_info.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Control : public rclcpp::Node
{
public:
    static constexpr double MAX_LON_ACCELERATION = 10.0;

    Control() : Node("control_node"), max_steering_angle(1.221730351448059)
    {
        cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = cb_group;

        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&Control::converter_to_cmd, this, std::placeholders::_1), sub_options);

        vehicle_info_sub = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleInfo>(
            "/carla/ego_vehicle/vehicle_info", 10, std::bind(&Control::update_vehicle_info, this, std::placeholders::_1), sub_options);

        control_pub = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
            "/carla/ego_vehicle/vehicle_control_cmd", 10);
    }

private:
    void update_vehicle_info(const carla_msgs::msg::CarlaEgoVehicleInfo::SharedPtr vehicle_info)
    {
        if (vehicle_info->wheels.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot determine max steering angle: Vehicle has no wheels.");
            rclcpp::shutdown();
            return;
        }

        max_steering_angle = vehicle_info->wheels[0].max_steer_angle;

        if (max_steering_angle == 0.0)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot determine max steering angle: Value is %f", max_steering_angle);
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Vehicle info received. Max steering angle=%f", max_steering_angle);
    }

    void converter_to_cmd(const geometry_msgs::msg::Twist::SharedPtr twist)
    {
        if (max_steering_angle == 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Did not yet receive vehicle info.");
            return;
        }

        auto control = carla_msgs::msg::CarlaEgoVehicleControl();

        if (*twist == geometry_msgs::msg::Twist())
        {
            // Stop
            control.throttle = 0.0;
            control.brake = 1.0;
            control.steer = 0.0;
        }
        else
        {
            if (twist->linear.x > 0)
            {
                control.throttle = std::min(MAX_LON_ACCELERATION, twist->linear.x) / MAX_LON_ACCELERATION;
            }
            else
            {
                control.reverse = true;
                control.throttle = std::max(-MAX_LON_ACCELERATION, twist->linear.x) / -MAX_LON_ACCELERATION;
            }

            if (twist->angular.z > 0)
            {
                control.steer = -std::min(max_steering_angle, twist->angular.z) / max_steering_angle;
                std::cout << control.steer << std::endl;
            }
            else
            {
                control.steer = -std::max(-max_steering_angle, twist->angular.z) / max_steering_angle;
                std::cout << control.steer << std::endl;
            }
        }

        try
        {
            control_pub->publish(control);
        }
        catch (const std::exception &e)
        {
            if (rclcpp::ok())
            {
                RCLCPP_WARN(this->get_logger(), "Error while publishing control: %s", e.what());
            }
        }
    }

    rclcpp::CallbackGroup::SharedPtr cb_group;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleInfo>::SharedPtr vehicle_info_sub;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_pub;
    double max_steering_angle;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto control_node = std::make_shared<Control>();
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(control_node);

    try
    {
        executor.spin();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}