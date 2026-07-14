#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "odometer_publisher_demo.hpp"
#include "sensor_fusion_gconstants.hpp"

using namespace std::chrono_literals;
constexpr int DEFAULT_ODOM_TX_PERIOD_MSEC = 1000;



OdometerSensorSimulator::OdometerSensorSimulator(const rclcpp::NodeOptions & options)
: Node("odometer_sensor_simulator", options) {
    this->declare_parameter<std::string>("csv_path", "ekf.csv");
    // this->declare_parameter<int>("odometer_sensor_tx_period", DEFAULT_ODOM_TX_PERIOD_MSEC);
    this->declare_parameter<double>("odometer_sensor_freq_hz", 30.0);
    sensor_freq_ = this->get_parameter("odometer_sensor_freq_hz").as_double();
    auto sensor_duration = std::chrono::duration<double>(1.0 / sensor_freq_);

    csv_path_ = this->get_parameter("csv_path").as_string();
    //int sensor_tx_period = this->get_parameter("odometer_sensor_tx_period").as_int();
    
    if (!csv_reader_.open(csv_path_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CSV telemetry data file: %s", csv_path_.c_str());
        return;
    }

    if (sensor_freq_ < sensor_fusion_gconstants::ODOMETER_SENSOR_MIN_FREQ_HZ || sensor_freq_ > sensor_fusion_gconstants::ODOMETER_SENSOR_MAX_FREQ_HZ) {
        RCLCPP_ERROR(this->get_logger(), "Sensor frequency %f Hz is out of bounds [%d, %d] Hz. Please adjust the parameter.",
                     sensor_freq_, sensor_fusion_gconstants::ODOMETER_SENSOR_MIN_FREQ_HZ, sensor_fusion_gconstants::ODOMETER_SENSOR_MAX_FREQ_HZ);
        return;
    }

    // Publisher setup as required
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);
    
    // Loop at 30Hz as planned for the Odometry sensor module (~33.3 milliseconds interval)
    // 33.3ms

    timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(sensor_duration),
            std::bind(&OdometerSensorSimulator::timerCallback, this));
    
    last_os_time_ = this->now();
    has_prior_point_ = false;

    RCLCPP_INFO(this->get_logger(), "Odometer Simulator Node online publishing on /odometry/filtered at 30Hz.");
}

OdometerSensorSimulator::~OdometerSensorSimulator() {
    csv_reader_.close();
}

// Helper function converting Euler angles (Roll, Pitch, Yaw) to a ROS2 geometry_quaternion
void OdometerSensorSimulator::eulerToQuaternion(double roll, double pitch, double yaw, 
                                                geometry_msgs::msg::Quaternion& q) 
{
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
}

void OdometerSensorSimulator::timerCallback() {
    auto telemetry_opt = csv_reader_.readNextRow();

    // Loop the file seamlessly if EOF is reached
    if (!telemetry_opt.has_value()) {
        RCLCPP_INFO(this->get_logger(), "Odometry CSV parser hit EOF. Rewinding to start.");
        csv_reader_.rewind();
        telemetry_opt = csv_reader_.readNextRow();
        has_prior_point_ = false; // Reset velocity calculator
        if (!telemetry_opt.has_value()) return;
    }

    const auto& current_data = telemetry_opt.value();
    rclcpp::Time current_os_time = this->now();

    auto odom_msg = nav_msgs::msg::Odometry();

    // 1. Fill Header using Live OS Time
    odom_msg.header.stamp = current_os_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // 2. Fill Pose (3D Position coordinates)
    odom_msg.pose.pose.position.x = current_data.pos_x;
    odom_msg.pose.pose.position.y = current_data.pos_y;
    odom_msg.pose.pose.position.z = current_data.pos_z;

    // 3. Convert Euler coordinates to Quaternion configuration
    eulerToQuaternion(current_data.roll, current_data.pitch, current_data.yaw, odom_msg.pose.pose.orientation);

    // 4. Fill Pose Covariance matrix (6x6 row-major flat layout)
    // Indices map to: 0=X, 7=Y, 14=Z, 21=Roll, 28=Pitch, 35=Yaw
    odom_msg.pose.covariance[0]  = current_data.cov_x;
    odom_msg.pose.covariance[7]  = current_data.cov_y;
    odom_msg.pose.covariance[14] = current_data.cov_z;
    odom_msg.pose.covariance[21] = current_data.cov_psi_x;
    odom_msg.pose.covariance[28] = current_data.cov_psi_y;
    odom_msg.pose.covariance[35] = current_data.cov_psi_z;

    // 5. Compute Instataneous Twist Velocity (dx/dt) dynamically via numeric differentials
    if (has_prior_point_) {
        double dt = (current_os_time - last_os_time_).seconds();
        if (dt > 0.001) { // Guard against division by zero
            odom_msg.twist.twist.linear.x = (current_data.pos_x - prior_data_.pos_x) / dt;
            odom_msg.twist.twist.linear.y = (current_data.pos_y - prior_data_.pos_y) / dt;
            odom_msg.twist.twist.linear.z = (current_data.pos_z - prior_data_.pos_z) / dt;
            
            odom_msg.twist.twist.angular.x = (current_data.roll - prior_data_.roll) / dt;
            odom_msg.twist.twist.angular.y = (current_data.pitch - prior_data_.pitch) / dt;
            odom_msg.twist.twist.angular.z = (current_data.yaw - prior_data_.yaw) / dt;
        }
    } else {
        // Initial step baseline or fallback if missing prior frame context
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
    }

    // Twist Covariance mapping (Using fixed baseline approximations since EKF output maps positions)
    odom_msg.twist.covariance[0]  = 0.01; // Velocity tracking error approximations
    odom_msg.twist.covariance[7]  = 0.01;
    odom_msg.twist.covariance[14] = 0.01;

    // Publish the fully populated Odometry message 
    odom_pub_->publish(odom_msg);

    // Cache historical telemetry snapshots for the next loop calculation step
    prior_data_ = current_data;
    last_os_time_ = current_os_time;
    has_prior_point_ = true;
}


