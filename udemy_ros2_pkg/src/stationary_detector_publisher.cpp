#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

#include "static_stationary_detector.hpp"

using namespace std::chrono_literals;

class HardenedSensorFusionNode : public rclcpp::Node {
public:
    explicit HardenedSensorFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("hardened_sensor_fusion_node", options) {
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
        detector_ = std::make_unique<StaticStationaryDetector>(
            duration,
            this->get_parameter("velocity_threshold").as_double(),
            this->get_parameter("range_variance_threshold").as_double(),
            odom_capacity,
            range_capacity
        );

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

private:
    // Callbacks now strictly execute at O(1) complexity with zero allocations
    void rangeCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
        detector_->updateRange(std::abs(msg->range), rclcpp::Time(msg->header.stamp).seconds());
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        double vz = msg->twist.twist.linear.z;
        double vel = std::sqrt(vx*vx + vy*vy + vz*vz);

        detector_->updateOdometry(vel, rclcpp::Time(msg->header.stamp).seconds());
    }

    void fusionLoopCallback() {
        bool state = detector_->isStationary(this->now().seconds());
        
        auto output_msg = std_msgs::msg::Bool();
        output_msg.data = state;
        status_pub_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<StaticStationaryDetector> detector_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardenedSensorFusionNode>());
    rclcpp::shutdown();
    return 0;
}