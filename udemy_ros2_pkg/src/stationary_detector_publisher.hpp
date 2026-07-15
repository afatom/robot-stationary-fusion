#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "spsc_ring_buffer.hpp"

using namespace std::chrono_literals;

class HardenedSensorFusionNode : public rclcpp::Node {
public:
    HardenedSensorFusionNode(const rclcpp::NodeOptions & options= rclcpp::NodeOptions());

private:
    // Callbacks now strictly execute at O(1) complexity with zero allocations
    void rangeCallback(const sensor_msgs::msg::Range::SharedPtr msg);
    
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void fusionLoopCallback();

private:
    void updateOdometry(double velocity_magnitude, double timestamp);
    void updateRange(double range, double timestamp);
    bool isStationary(double current_timestamp);
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // std::unique_ptr<StaticStationaryDetector> detector_;
    SpscRingBuffer odom_buffer_;
    SpscRingBuffer range_buffer_;
    double window_duration_;
    double velocity_th_;
    double variance_th_;
};
