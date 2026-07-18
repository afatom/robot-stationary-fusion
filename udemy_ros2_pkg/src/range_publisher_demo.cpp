#include "udemy_ros2_pkg/range_publisher_demo.hpp"
#include "udemy_ros2_pkg/sensor_fusion_gconstants.hpp"

#include <chrono>
#include <fstream>
#include <string>

using namespace std::chrono_literals;


AcousticSensorSimulator::AcousticSensorSimulator(const rclcpp::NodeOptions & options)
: Node("acoustic_sensor_publisher_simulator", options) {
    this->declare_parameter<std::string>("csv_path", std::string(fusion_consts::EKF_CSV_DEFAULT_PATH));
    this->declare_parameter<double>("range_sensor_freq_hz", 20.0);
    sensor_freq_ = this->get_parameter("range_sensor_freq_hz").as_double();
    auto sensor_duration = std::chrono::duration<double, std::ratio<1>>(1.0 / sensor_freq_);

    csv_path_ = this->get_parameter("csv_path").as_string();

    // Instantiate and open via the decoupled helper object API
    if (!csv_reader_.open(csv_path_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CSV telemetry data file: '%s'", csv_path_.c_str());
        return;
    }

    if (sensor_freq_ < fusion_consts::RANGE_SENSOR_MIN_FREQ_HZ || sensor_freq_ > fusion_consts::RANGE_SENSOR_MAX_FREQ_HZ) {
        RCLCPP_ERROR(this->get_logger(), "Sensor frequency %f Hz is out of bounds [%d, %d] Hz. Please adjust the parameter.",
                     sensor_freq_, fusion_consts::RANGE_SENSOR_MIN_FREQ_HZ, fusion_consts::RANGE_SENSOR_MAX_FREQ_HZ);
        return;
    }

    // Setup the publisher for the range topic
    range_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/peak_detection/channel_0/range", 10);

    // Create a wall timer spinning at ~20Hz (50ms interval) to stream out data rows
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(int64_t(sensor_tx_period)), std::bind(&AcousticSensorSimulator::timerCallback, this));
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(sensor_duration),
        std::bind(&AcousticSensorSimulator::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Acoustic Simulator started reading from %s", csv_path_.c_str());
}

AcousticSensorSimulator::~AcousticSensorSimulator() {
    csv_reader_.close();
}


void AcousticSensorSimulator::timerCallback() {
    auto telemetry_opt = csv_reader_.readNextRow();
    // Automatically rewind the data loop if we hit the end-of-file condition
    if (!telemetry_opt.has_value()) {
        RCLCPP_INFO(this->get_logger(), "Reached EOF or row error. Rewinding file reader back to start.");
        csv_reader_.rewind();
        telemetry_opt = csv_reader_.readNextRow();
        
        if (!telemetry_opt.has_value()) {
            RCLCPP_ERROR(this->get_logger(), "Unable to read tracking files following reader rewind process.");
            return;
        }
    }

    // Unpack data from optional storage block
    const auto& telemetry = telemetry_opt.value();
    // Convert raw coordinate data directly into standard absolute range measurements
    float simulated_range = std::abs(static_cast<float>(telemetry.pos_z));

    auto range_msg = sensor_msgs::msg::Range();
    // --- LIVE OS TIMESTAMP GENERATION ---
    // This pulls the system runtime clock (or simulation time if use_sim_time is active)
    // instead of pulling the legacy tracking floats from the CSV file.
    range_msg.header.stamp = this->now(); 
    range_msg.header.frame_id = "acoustic_sensor_link";

    // Fill standard Range configuration metadata
    range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    range_msg.field_of_view = fusion_consts::RANGE_SENSOR_FIELD_OF_VIEW_RAD;
    range_msg.min_range = fusion_consts::RANGE_SENSOR_MIN_RANGE_M;
    range_msg.max_range = fusion_consts::RANGE_SENSOR_MAX_RANGE_M;
    range_msg.range = simulated_range;

    range_pub_->publish(range_msg);
}


