#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "csv_telemetry_reader.hpp"

class OdometerSensorSimulator : public rclcpp::Node {
public:
    explicit OdometerSensorSimulator(const rclcpp::NodeOptions & options= rclcpp::NodeOptions());
    ~OdometerSensorSimulator();
private:
    void timerCallback();
    void eulerToQuaternion(double roll, double pitch, double yaw, 
                        geometry_msgs::msg::Quaternion& q);
private:
    std::string csv_path_;
    double sensor_freq_;
    CSVTelemetryReader csv_reader_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Historical tracking parameters for dynamic Twist generation
    PoolRobotTelemetry prior_data_;
    rclcpp::Time last_os_time_;
    bool has_prior_point_;
};