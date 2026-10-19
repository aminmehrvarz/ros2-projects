#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class DiffDriveKinematics : public rclcpp::Node {
public:
    DiffDriveKinematics() : Node("diff_drive_kinematics") {
        // Physical parameters: Wheel radius (meters) & Track width (meters)
        declare_parameter("wheel_radius", 0.033);
        declare_parameter("track_width", 0.16);

        wheel_radius_ = get_parameter("wheel_radius").as_double();
        track_width_ = get_parameter("track_width").as_double();

        // Subscriber to incoming body velocity commands (/cmd_vel)
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&DiffDriveKinematics::cmdVelCallback, this, std::placeholders::_1)
        );

        // Publishers for left and right wheel angular speeds (rad/s)
        left_wheel_pub_ = create_publisher<std_msgs::msg::Float64>("/wheel_speed_left", 10);
        right_wheel_pub_ = create_publisher<std_msgs::msg::Float64>("/wheel_speed_right", 10);

        RCLCPP_INFO(get_logger(), "C++ Differential Drive Kinematics Node Started");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double v = msg->linear.x;   // Forward linear velocity (m/s)
        double w = msg->angular.z;  // Angular velocity (rad/s)

        // Inverse Kinematics Equations:
        // w_L = (v - (L/2)*w) / r
        // w_R = (v + (L/2)*w) / r
        double w_left = (v - (track_width_ / 2.0) * w) / wheel_radius_;
        double w_right = (v + (track_width_ / 2.0) * w) / wheel_radius_;

        auto left_msg = std_msgs::msg::Float64();
        auto right_msg = std_msgs::msg::Float64();
        left_msg.data = w_left;
        right_msg.data = w_right;

        left_wheel_pub_->publish(left_msg);
        right_wheel_pub_->publish(right_msg);
    }

    double wheel_radius_;
    double track_width_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveKinematics>());
    rclcpp::shutdown();
    return 0;
}