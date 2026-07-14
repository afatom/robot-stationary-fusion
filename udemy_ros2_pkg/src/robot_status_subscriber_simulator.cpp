#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"


class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("robot_status_subscriber_simulator_node")
  {
    // auto range_topic_callback =
    //   [this](sensor_msgs::msg::Range::UniquePtr msg) -> void {
    //     RCLCPP_INFO(this->get_logger(), "I heard depth: '%f'", msg->range);
    //   };
    auto stationary_topic_callback =
      [this](std_msgs::msg::Bool::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "Is Robot Stationary: '%s'", msg->data==true ? "^^ YES ^^" : "xx NO xx");
      };
    // auto odometer_topic_callback =
    //   [this](nav_msgs::msg::Odometry::UniquePtr msg) -> void {
    //     RCLCPP_INFO(this->get_logger(), "I heard odometer: '%f', x,y,z = ['%f','%f','%f']",
    //     msg->pose.covariance.at(0),
    //     msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    //   };
    // range_subscription_ =
    //   this->create_subscription<sensor_msgs::msg::Range>("/peak_detection/channel_0/range", 10, range_topic_callback);
    // odometer_subscription_ =
    //   this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, odometer_topic_callback);
    
    is_stationary_subscription_ =
      this->create_subscription<std_msgs::msg::Bool>("/robot_state/is_stationary", 10, stationary_topic_callback);
    
  }

private:
  // rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_subscription_;
  // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometer_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_stationary_subscription_;

  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}