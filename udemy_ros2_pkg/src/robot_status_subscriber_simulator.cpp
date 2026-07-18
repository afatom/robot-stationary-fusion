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
    auto stationary_topic_callback =
      [this](std_msgs::msg::Bool::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "Is Robot Stationary: '%s'", msg->data==true ? "^^ YES ^^" : "xx NO xx");
      };

    is_stationary_subscription_ =
      this->create_subscription<std_msgs::msg::Bool>("/robot_state/is_stationary", 10, stationary_topic_callback);
    
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_stationary_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}