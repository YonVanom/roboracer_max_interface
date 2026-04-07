#ifndef ROBORACER_MAX_INTERFACE__ROBORACER_MAX_INTERFACE_NODE_HPP_
#define ROBORACER_MAX_INTERFACE__ROBORACER_MAX_INTERFACE_NODE_HPP_

#include <deque>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class RoboracerMaxInterfaceNode : public rclcpp::Node
{
public:
  RoboracerMaxInterfaceNode();

private:
  void onControlCmd(const autoware_control_msgs::msg::Control::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Returns the updated moving average after pushing a new sample.
  double updateMovingAverage(std::deque<double> & window, double & sum, double sample) const;

  // Rounds to the given number of decimal places if >= 0, otherwise returns value unchanged.
  static double maybeRound(double value, int decimal_places);

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_status_pub_;

  std::string control_cmd_topic_{"/control/command/control_cmd"};
  std::string odom_topic_{"/ego/odom"};
  std::string drive_topic_{"/ego/drive"};
  std::string steering_status_topic_{"/vehicle/status/steering_status"};
  std::string velocity_status_topic_{"/vehicle/status/velocity_status"};

  int moving_average_window_{10};
  int longitudinal_decimal_places_{-1};  // -1 disables rounding
  int lateral_decimal_places_{-1};
  int heading_rate_decimal_places_{-1};

  // Moving average state: one window + running sum per velocity channel
  std::deque<double> long_vel_window_;
  std::deque<double> lat_vel_window_;
  std::deque<double> heading_rate_window_;
  double long_vel_sum_{0.0};
  double lat_vel_sum_{0.0};
  double heading_rate_sum_{0.0};
};

#endif  // ROBORACER_MAX_INTERFACE__ROBORACER_MAX_INTERFACE_NODE_HPP_
