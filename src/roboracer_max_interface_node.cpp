#include <memory>

#include "roboracer_max_interface/roboracer_max_interface_node.hpp"

RoboracerMaxInterfaceNode::RoboracerMaxInterfaceNode()
: Node("roboracer_max_interface_node")
{
  using std::placeholders::_1;

  control_cmd_topic_ =
    this->declare_parameter<std::string>("control_cmd_topic", control_cmd_topic_);
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", odom_topic_);
  drive_topic_ = this->declare_parameter<std::string>("drive_topic", drive_topic_);
  steering_status_topic_ =
    this->declare_parameter<std::string>("steering_status_topic", steering_status_topic_);
  velocity_status_topic_ =
    this->declare_parameter<std::string>("velocity_status_topic", velocity_status_topic_);

  control_cmd_sub_ = this->create_subscription<autoware_control_msgs::msg::Control>(
    control_cmd_topic_, rclcpp::QoS{1},
    std::bind(&RoboracerMaxInterfaceNode::onControlCmd, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS{1},
    std::bind(&RoboracerMaxInterfaceNode::onOdom, this, _1));

  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    drive_topic_, rclcpp::QoS{1});

  steering_status_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
    steering_status_topic_, rclcpp::QoS{1});

  velocity_status_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
    velocity_status_topic_, rclcpp::QoS{1});
}

void RoboracerMaxInterfaceNode::onControlCmd(
  const autoware_control_msgs::msg::Control::SharedPtr msg)
{
  ackermann_msgs::msg::AckermannDriveStamped drive;
  drive.header.stamp = msg->stamp;
  drive.header.frame_id = "base_link";
  drive.drive.speed = msg->longitudinal.velocity;
  drive.drive.steering_angle = msg->lateral.steering_tire_angle;
  drive_pub_->publish(drive);

  autoware_vehicle_msgs::msg::SteeringReport steering;
  steering.stamp = msg->stamp;
  steering.steering_tire_angle = msg->lateral.steering_tire_angle;
  steering_status_pub_->publish(steering);
}

void RoboracerMaxInterfaceNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  autoware_vehicle_msgs::msg::VelocityReport velocity;
  velocity.header = msg->header;
  velocity.longitudinal_velocity = static_cast<float>(msg->twist.twist.linear.x);
  velocity.lateral_velocity = static_cast<float>(msg->twist.twist.linear.y);
  velocity.heading_rate = static_cast<float>(msg->twist.twist.angular.z);
  velocity_status_pub_->publish(velocity);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoboracerMaxInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
