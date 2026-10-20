#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "diff_drive_control/srv/navigate_to_waypoint.hpp"

using namespace std::chrono_literals;

class PosePidController : public rclcpp::Node {
public:
  PosePidController() : Node("pose_pid_controller") {
    // Declare & initialize parameters
    this->declare_parameter<double>("kp_linear", 1.0);
    this->declare_parameter<double>("kp_angular", 2.0);
    this->declare_parameter<double>("goal_tolerance", 0.05);

    // Publishers & Subscribers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&PosePidController::odom_callback, this, std::placeholders::_1));

    // Create Service Server (/navigate_to_waypoint)
    service_ = this->create_service<diff_drive_control::srv::NavigateToWaypoint>(
      "/navigate_to_waypoint",
      std::bind(&PosePidController::handle_waypoint_request, this,
                std::placeholders::_1, std::placeholders::_2));

    // 20Hz Control Loop Timer
    timer_ = this->create_wall_timer(
      50ms, std::bind(&PosePidController::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Pose PID Controller with Waypoint Service Initialized.");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // Quaternion to Yaw conversion
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    current_theta_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  }

  void handle_waypoint_request(
    const std::shared_ptr<diff_drive_control::srv::NavigateToWaypoint::Request> request,
    std::shared_ptr<diff_drive_control::srv::NavigateToWaypoint::Response> response) {
    
    goal_x_ = request->x;
    goal_y_ = request->y;
    has_active_goal_ = true;

    RCLCPP_INFO(this->get_logger(), "Received New Waypoint Request: x=%.2f, y=%.2f", goal_x_, goal_y_);

    response->success = true;
    response->message = "Waypoint set successfully. Navigating...";
  }

  void control_loop() {
    if (!has_active_goal_) return;

    double dx = goal_x_ - current_x_;
    double dy = goal_y_ - current_y_;
    double distance = std::sqrt(dx * dx + dy * dy);

    double kp_linear = this->get_parameter("kp_linear").as_double();
    double kp_angular = this->get_parameter("kp_angular").as_double();
    double tolerance = this->get_parameter("goal_tolerance").as_double();

    geometry_msgs::msg::Twist cmd;

    if (distance < tolerance) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal Reached!");
      return;
    }

    double desired_heading = std::atan2(dy, dx);
    double heading_error = desired_heading - current_theta_;

    // Normalize angle to [-pi, pi]
    while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
    while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

    cmd.linear.x = std::min(kp_linear * distance, 0.5);
    cmd.angular.z = std::clamp(kp_angular * heading_error, -1.0, 1.0);

    cmd_vel_pub_->publish(cmd);
  }

  // ROS 2 Interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<diff_drive_control::srv::NavigateToWaypoint>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Internal State Variables
  double current_x_{0.0}, current_y_{0.0}, current_theta_{0.0};
  double goal_x_{0.0}, goal_y_{0.0};
  bool has_active_goal_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePidController>());
  rclcpp::shutdown();
  return 0;
}