#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PosePidController : public rclcpp::Node {
public:
  PosePidController() : Node("pose_pid_controller"), x_(0.0), y_(0.0), yaw_(0.0) {
    // Target coordinates and controller gains
    this->declare_parameter<double>("x_goal", 2.0);
    this->declare_parameter<double>("y_goal", 2.0);
    this->declare_parameter<double>("kp_angular", 2.0);
    this->declare_parameter<double>("kp_linear", 0.5);
    this->declare_parameter<double>("goal_tolerance", 0.05);

    // Publishers & Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PosePidController::odomCallback, this, std::placeholders::_1));
    
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Control Loop Timer (20 Hz)
    timer_ = this->create_wall_timer(50ms, std::bind(&PosePidController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Pose PID Controller Node Initialized.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;

    // Convert orientation quaternion to yaw (z-axis rotation)
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    yaw_ = std::atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz));
  }

  void controlLoop() {
    double target_x = this->get_parameter("x_goal").as_double();
    double target_y = this->get_parameter("y_goal").as_double();
    double kp_ang = this->get_parameter("kp_angular").as_double();
    double kp_lin = this->get_parameter("kp_linear").as_double();
    double tolerance = this->get_parameter("goal_tolerance").as_double();

    // Compute error components
    double ex = target_x - x_;
    double ey = target_y - y_;
    double distance_error = std::sqrt(ex * ex + ey * ey);

    geometry_msgs::msg::Twist cmd;

    // Check goal condition
    if (distance_error < tolerance) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal Reached!");
      return;
    }

    // Heading error normalized to [-PI, PI]
    double target_yaw = std::atan2(ey, ex);
    double yaw_error = target_yaw - yaw_;
    yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

    // Proportional angular control
    cmd.angular.z = kp_ang * yaw_error;
    
    // Linear control scaled by cosine of heading error
    cmd.linear.x = kp_lin * distance_error * std::cos(yaw_error);

    // Velocity clamping for safety
    cmd.linear.x = std::max(-0.5, std::min(0.5, cmd.linear.x));
    cmd.angular.z = std::max(-1.0, std::min(1.0, cmd.angular.z));

    cmd_vel_pub_->publish(cmd);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double x_, y_, yaw_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePidController>());
  rclcpp::shutdown();
  return 0;
}