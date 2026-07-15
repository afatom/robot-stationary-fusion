#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

#include "static_stationary_detector.hpp"
#include "stationary_detector_publisher.hpp"

using namespace std::chrono_literals;


HardenedSensorFusionNode::HardenedSensorFusionNode(const rclcpp::NodeOptions & options) : Node("hardened_sensor_fusion_node", options) {
    // Declare tracking thresholds
    this->declare_parameter<double>("window_duration_sec", 0.5);
    this->declare_parameter<double>("velocity_threshold", 0.02);
    this->declare_parameter<double>("range_variance_threshold", 0.0005);
    this->declare_parameter<double>("fusion_loop_rate_hz", 25.0);
    
    // Handshake expected frequencies to pre-size rings cleanly
    this->declare_parameter<double>("expected_odom_hz", 30.0);
    this->declare_parameter<double>("expected_range_hz", 20.0);

    double duration = this->get_parameter("window_duration_sec").as_double();
    
    // Safety multiplier (3x window requirement size) to comfortably accommodate jitter/burst packet arrivals
    size_t odom_capacity = static_cast<size_t>(this->get_parameter("expected_odom_hz").as_double() * duration * 3.0);
    size_t range_capacity = static_cast<size_t>(this->get_parameter("expected_range_hz").as_double() * duration * 3.0);

    // Instantiation - Dynamic allocation strictly isolated to the initialization sequence
    // detector_ = std::make_unique<StaticStationaryDetector>(
    //     duration,
    //     this->get_parameter("velocity_threshold").as_double(),
    //     this->get_parameter("range_variance_threshold").as_double(),
    //     odom_capacity,
    //     range_capacity
    // );
    odom_buffer_.init(odom_capacity);
    range_buffer_.init(range_capacity);
    window_duration_ = duration;
    velocity_th_ = this->get_parameter("velocity_threshold").as_double();
    variance_th_ = this->get_parameter("range_variance_threshold").as_double();

    range_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/peak_detection/channel_0/range", 10,
        std::bind(&HardenedSensorFusionNode::rangeCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&HardenedSensorFusionNode::odometryCallback, this, std::placeholders::_1));

    status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot_state/is_stationary", 10);

    double loop_hz = this->get_parameter("fusion_loop_rate_hz").as_double();
    auto loop_duration = std::chrono::duration<double>(1.0 / loop_hz);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(loop_duration),
        std::bind(&HardenedSensorFusionNode::fusionLoopCallback, this));

    RCLCPP_INFO(this->get_logger(), "Deterministic Hardened Fusion Node initialized safely.");
}


// Callbacks now strictly execute at O(1) complexity with zero allocations
void HardenedSensorFusionNode::rangeCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
    updateRange(std::abs(msg->range), rclcpp::Time(msg->header.stamp).seconds());
}

void HardenedSensorFusionNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double vz = msg->twist.twist.linear.z;
    double vel = std::sqrt(vx*vx + vy*vy + vz*vz);

    updateOdometry(vel, rclcpp::Time(msg->header.stamp).seconds());
}

void HardenedSensorFusionNode::fusionLoopCallback() {
    bool state = isStationary(this->now().seconds());
    
    auto output_msg = std_msgs::msg::Bool();
    output_msg.data = state;
    status_pub_->publish(output_msg);
}

inline void HardenedSensorFusionNode::updateOdometry(double velocity_magnitude, double timestamp) {
    odom_buffer_.push(velocity_magnitude, timestamp); // O(1) Zero Allocation
}

inline void HardenedSensorFusionNode::updateRange(double range, double timestamp) {
    range_buffer_.push(range, timestamp);             // O(1) Zero Allocation
}

bool HardenedSensorFusionNode::isStationary(double current_timestamp)
{
    double cutoff_time = current_timestamp - window_duration_;

    // --- 1. Evaluate Kinematics (Odometry) ---
    bool odom_indicates_still = true;
    size_t valid_odom_count = 0;

    for (size_t i = 0; i < odom_buffer_.size(); ++i) {
        auto sample = odom_buffer_.get_relative(i);
        // Skip data points older than our time window
        if (sample.timestamp < cutoff_time) continue;

        valid_odom_count++;
        if (sample.value >= velocity_th_) {
            odom_indicates_still = false;
            break; // Break early if moving
        }
    }

    // If no fresh odometry packets exist, fall back to moving/unsafe state
    if (valid_odom_count == 0) return false;


    // --- 2. Evaluate Spatial Profile (Range Variance) ---
    size_t valid_range_count = 0;
    double range_sum = 0.0;

    // Pass 1: Calculate Mean of valid range points in current window
    for (size_t i = 0; i < range_buffer_.size(); ++i) {
        auto sample = range_buffer_.get_relative(i);
        if (sample.timestamp < cutoff_time) continue;

        range_sum += sample.value;
        valid_range_count++;
    }

    // Require a baseline minimum population profile to confidently calculate variance
    if (valid_range_count < 3) return false;
    double range_mean = range_sum / valid_range_count;

    // Pass 2: Calculate Variance
    double variance_sum = 0.0;
    for (size_t i = 0; i < range_buffer_.size(); ++i) {
        auto sample = range_buffer_.get_relative(i);
        if (sample.timestamp < cutoff_time) continue;

        variance_sum += (sample.value - range_mean) * (sample.value - range_mean);
    }
    double range_variance = variance_sum / valid_range_count;
    bool range_indicates_still = (range_variance < variance_th_);


    // --- 3. Strict Fusion Evaluation ---
    return (odom_indicates_still && range_indicates_still);
}