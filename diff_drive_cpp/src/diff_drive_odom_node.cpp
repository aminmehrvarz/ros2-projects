#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

class DiffDriveOdom : public rclcpp::Node {
public:
    DiffDriveOdom() : Node("diff_drive_odom"), x_(0.0), y_(0.0), theta_(0.0) {
        declare_parameter("wheel_radius", 0.033);
        declare_parameter("track_width", 0.16);

        wheel_radius_ = get_parameter("wheel_radius").as_double();
        track_width_ = get_parameter("track_width").as_double();

        left_speed_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/wheel_speed_left", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { w_left_ = msg->data; }
        );

        right_speed_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/wheel_speed_right", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { w_right_ = msg->data; }
        );

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        last_time_ = this->now();

        // Timer loop running at 20 Hz (every 50 ms)
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DiffDriveOdom::updateOdometry, this)
        );

        RCLCPP_INFO(get_logger(), "C++ Odometry Node Started");
    }

private:
    void updateOdometry() {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // 1. Forward Kinematics
        double v_left = w_left_ * wheel_radius_;
        double v_right = w_right_ * wheel_radius_;

        double v = (v_right + v_left) / 2.0;
        double w = (v_right - v_left) / track_width_;

        // 2. Position Integration
        x_ += v * std::cos(theta_) * dt;
        y_ += v * std::sin(theta_) * dt;
        theta_ += w * dt;

        // 3. Publish /odom message
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;

        // Convert yaw (theta) to quaternion format for 3D ROS orientation
        odom_msg.pose.pose.orientation.z = std::sin(theta_ / 2.0);
        odom_msg.pose.pose.orientation.w = std::cos(theta_ / 2.0);

        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.angular.z = w;

        odom_pub_->publish(odom_msg);
    }

    double wheel_radius_;
    double track_width_;
    double x_, y_, theta_;
    double w_left_ = 0.0;
    double w_right_ = 0.0;

    rclcpp::Time last_time_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_speed_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveOdom>());
    rclcpp::shutdown();
    return 0;
}