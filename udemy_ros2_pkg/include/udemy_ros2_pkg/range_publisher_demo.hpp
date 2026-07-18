#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "csv_telemetry_reader.hpp"

class AcousticSensorSimulator : public rclcpp::Node {
public:
    explicit AcousticSensorSimulator(const rclcpp::NodeOptions & options= rclcpp::NodeOptions());
    ~AcousticSensorSimulator();
private:
    void timerCallback();
private:
    std::string csv_path_;
    double sensor_freq_;
    CSVTelemetryReader csv_reader_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
