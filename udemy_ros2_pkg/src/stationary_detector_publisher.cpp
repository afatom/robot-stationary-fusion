#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

#include "udemy_ros2_pkg/static_stationary_detector.hpp"
#include "udemy_ros2_pkg/range_publisher_demo.hpp"
#include "udemy_ros2_pkg/odometer_publisher_demo.hpp"

using namespace std::chrono_literals;

class HardenedSensorFusionNode : public rclcpp::Node {
public:
    explicit HardenedSensorFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("hardened_sensor_fusion_node", options) {
        // Declare tracking thresholds
        this->declare_parameter<double>("window_duration_sec", 0.5);
        this->declare_parameter<double>("velocity_threshold", 0.06);
        this->declare_parameter<double>("range_variance_threshold", 0.0005);
        this->declare_parameter<double>("moving_ratio_threshold", 0.20);
        this->declare_parameter<double>("moving_ratio_hysteresis", 0.05);
        this->declare_parameter<double>("variance_hysteresis_ratio", 0.20);
        this->declare_parameter<double>("fusion_loop_rate_hz", 25.0); //25.0
        
        // Handshake expected frequencies to pre-size rings cleanly
        this->declare_parameter<double>("expected_odom_hz", 30.0);
        this->declare_parameter<double>("expected_range_hz", 20.0);

        double duration = this->get_parameter("window_duration_sec").as_double();
        if (duration <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "window_duration_sec must be > 0. Falling back to 0.5");
            duration = 0.5;
        }
        
        // Safety multiplier (3x window requirement size) to comfortably accommodate jitter/burst packet arrivals
        double expected_odom_hz = this->get_parameter("expected_odom_hz").as_double();
        double expected_range_hz = this->get_parameter("expected_range_hz").as_double();
        if (expected_odom_hz <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "expected_odom_hz must be > 0. Falling back to 30.0");
            expected_odom_hz = 30.0;
        }
        if (expected_range_hz <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "expected_range_hz must be > 0. Falling back to 20.0");
            expected_range_hz = 20.0;
        }

        size_t odom_capacity = static_cast<size_t>(expected_odom_hz * duration * 4.0);
        size_t range_capacity = static_cast<size_t>(expected_range_hz * duration * 4.0);

        // Instantiation - Dynamic allocation strictly isolated to the initialization sequence
        // double __velocity_th = this->get_parameter("velocity_threshold").as_double();
        // double __variance_th = this->get_parameter("range_variance_threshold").as_double();
        
        // RCLCPP_INFO(this->get_logger(), "Static Stationary Detector initialized with velocity threshold %f.", __velocity_th);
        // RCLCPP_INFO(this->get_logger(), "Static Stationary Detector initialized with range variance threshold %f.", __variance_th);
        // RCLCPP_INFO(this->get_logger(), "Static Stationary Detector initialized with window duration %f.", duration);

        const double velocity_threshold = this->get_parameter("velocity_threshold").as_double();
        const double range_variance_threshold = this->get_parameter("range_variance_threshold").as_double();
        const double moving_ratio_threshold = this->get_parameter("moving_ratio_threshold").as_double();
        const double moving_ratio_hysteresis = this->get_parameter("moving_ratio_hysteresis").as_double();
        const double variance_hysteresis_ratio = this->get_parameter("variance_hysteresis_ratio").as_double();

        detector_ = std::make_unique<StaticStationaryDetector>(
            duration,
            velocity_threshold,
            range_variance_threshold,
            moving_ratio_threshold,
            moving_ratio_hysteresis,
            variance_hysteresis_ratio,
            odom_capacity,
            range_capacity
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Detector params: window=%.2fs vel_th=%.4f var_th=%.6f moving_ratio_th=%.3f ratio_hyst=%.3f var_hyst_ratio=%.3f | buffers: odom=%zu range=%zu",
            duration,
            velocity_threshold,
            range_variance_threshold,
            moving_ratio_threshold,
            moving_ratio_hysteresis,
            variance_hysteresis_ratio,
            odom_capacity,
            range_capacity);

        cb_group1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        auto odometer_options = rclcpp::SubscriptionOptions();
        odometer_options.callback_group = cb_group2_;
        auto range_options = rclcpp::SubscriptionOptions();
        range_options.callback_group = cb_group1_;

        range_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/peak_detection/channel_0/range", 10,
            std::bind(&HardenedSensorFusionNode::rangeCallback, this, std::placeholders::_1), range_options);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&HardenedSensorFusionNode::odometryCallback, this, std::placeholders::_1), odometer_options);

        status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot_state/is_stationary", 10);

        double loop_hz = this->get_parameter("fusion_loop_rate_hz").as_double();
        auto loop_duration = std::chrono::duration<double, std::ratio<1>>(1.0 / loop_hz);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(loop_duration),
            std::bind(&HardenedSensorFusionNode::fusionLoopCallback, this), cb_group3_);

        RCLCPP_INFO(this->get_logger(), "Deterministic Hardened Fusion Node initialized safely.");
    }

private:
    // Callbacks now strictly execute at O(1) complexity with zero allocations
    void rangeCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
        bool pushed = detector_->updateRange(std::abs(msg->range), rclcpp::Time(msg->header.stamp).seconds());
        if (!pushed) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Range queue is full: dropping newest range sample. Consider increasing expected_range_hz/window_duration_sec.");
        }
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        double vz = msg->twist.twist.linear.z;
        double vel = std::sqrt(vx*vx + vy*vy + vz*vz);

        bool pushed = detector_->updateOdometry(vel, rclcpp::Time(msg->header.stamp).seconds());
        if (!pushed) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Odometry queue is full: dropping newest odometry sample. Consider increasing expected_odom_hz/window_duration_sec.");
        }
    }

    void fusionLoopCallback() {
        // bool state = detector_->isStationary();
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
    rclcpp::CallbackGroup::SharedPtr cb_group1_;
    rclcpp::CallbackGroup::SharedPtr cb_group2_;
    rclcpp::CallbackGroup::SharedPtr cb_group3_;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto range_pub_node = std::make_shared<AcousticSensorSimulator>();
    auto odom_pub_node = std::make_shared<OdometerSensorSimulator>();
    auto sensor_fusion_node = std::make_shared<HardenedSensorFusionNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(range_pub_node);
    executor.add_node(odom_pub_node);
    executor.add_node(sensor_fusion_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
