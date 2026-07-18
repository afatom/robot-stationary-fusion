
#include <memory>
#include <string>
#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "../../udemy_ros2_pkg/include/udemy_ros2_pkg/odometer_publisher_demo.hpp"


// Test fixture for OdometerSensorSimulator
class TestOdometerSensorSimulator : public::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        // 1. Create a NodeOptions container
        rclcpp::NodeOptions options;

        // 2. Programmatically inject parameters using rclcpp::Parameter objects
        std::vector<rclcpp::Parameter> params;
        params.push_back(rclcpp::Parameter("csv_path", std::string("/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf_test.csv")));
        params.push_back(rclcpp::Parameter("odometer_sensor_freq_hz", 30.0));
        options.parameter_overrides(params);

        node_p = std::make_shared<OdometerSensorSimulator>(options);
    }

    void TearDown() override {
        node_p.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<OdometerSensorSimulator> node_p;
};


// ============================================================================
// BASIC NODE INITIALIZATION TESTS
// ============================================================================

TEST_F(TestOdometerSensorSimulator, TestNodeCreation)
{
    EXPECT_EQ(std::string(node_p->get_name()), std::string("odometer_sensor_simulator"));
    auto pub_ep = node_p->get_publishers_info_by_topic("/odometry/filtered");
    EXPECT_EQ(pub_ep.size(), 1U);
}

TEST_F(TestOdometerSensorSimulator, TestNodeParamsValidation)
{
    auto csv_path_param = node_p->get_parameter("csv_path");
    EXPECT_EQ(csv_path_param.as_string(), "/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf_test.csv");

    auto sensor_freq_param = node_p->get_parameter("odometer_sensor_freq_hz");
    EXPECT_DOUBLE_EQ(sensor_freq_param.as_double(), 30.0);
}


// ============================================================================
// PARAMETER BOUNDARY TESTS
// ============================================================================

TEST_F(TestOdometerSensorSimulator, TestDefaultFrequencyParameter)
{
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> params;
    params.push_back(rclcpp::Parameter("csv_path", 
        std::string("/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf_test.csv")));
    // Don't set frequency - should use default
    options.parameter_overrides(params);

    auto node = std::make_shared<OdometerSensorSimulator>(options);
    auto freq_param = node->get_parameter("odometer_sensor_freq_hz");
    
    // Default should be 30.0 Hz
    EXPECT_DOUBLE_EQ(freq_param.as_double(), 30.0);
    node.reset();
}

TEST_F(TestOdometerSensorSimulator, TestLowFrequencyOperation)
{
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> params;
    params.push_back(rclcpp::Parameter("csv_path", 
        std::string("/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf_test.csv")));
    params.push_back(rclcpp::Parameter("odometer_sensor_freq_hz", 5.0)); // Low frequency
    options.parameter_overrides(params);

    auto node = std::make_shared<OdometerSensorSimulator>(options);
    auto freq_param = node->get_parameter("odometer_sensor_freq_hz");
    EXPECT_DOUBLE_EQ(freq_param.as_double(), 5.0);
    node.reset();
}

TEST_F(TestOdometerSensorSimulator, TestHighFrequencyOperation)
{
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> params;
    params.push_back(rclcpp::Parameter("csv_path", 
        std::string("/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf_test.csv")));
    params.push_back(rclcpp::Parameter("odometer_sensor_freq_hz", 100.0)); // High frequency
    options.parameter_overrides(params);

    auto node = std::make_shared<OdometerSensorSimulator>(options);
    auto freq_param = node->get_parameter("odometer_sensor_freq_hz");
    EXPECT_DOUBLE_EQ(freq_param.as_double(), 100.0);
    node.reset();
}


// // ============================================================================
// // MESSAGE CONTENT VALIDATION TESTS
// // ============================================================================

TEST_F(TestOdometerSensorSimulator, TestOdometryMessageStructure)
{
    // Collect published messages via subscription
    std::vector<nav_msgs::msg::Odometry> messages;
    
    auto subscription = node_p->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        [&messages](const nav_msgs::msg::Odometry::SharedPtr msg) {
            messages.push_back(*msg);
        });

    // Spin multiple times to let timer callbacks fire (30 Hz timer = ~33ms per callback)
    // Need to spin repeatedly for ~200ms to ensure messages arrive
    for (int i = 0; i < 20; ++i) {
        rclcpp::spin_some(node_p);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    EXPECT_GT(messages.size(), 0U) << "No messages received";

    if (!messages.empty()) {
        const auto& odom_msg = messages[0];
        
        // Verify header
        EXPECT_EQ(odom_msg.header.frame_id, "odom");
        EXPECT_EQ(odom_msg.child_frame_id, "base_link");
        
        // Verify position fields exist (should not be NaN)
        EXPECT_FALSE(std::isnan(odom_msg.pose.pose.position.x));
        EXPECT_FALSE(std::isnan(odom_msg.pose.pose.position.y));
        EXPECT_FALSE(std::isnan(odom_msg.pose.pose.position.z));
        
        // Verify orientation (quaternion should be normalized)
        double q_norm_sq = odom_msg.pose.pose.orientation.w * odom_msg.pose.pose.orientation.w +
                          odom_msg.pose.pose.orientation.x * odom_msg.pose.pose.orientation.x +
                          odom_msg.pose.pose.orientation.y * odom_msg.pose.pose.orientation.y +
                          odom_msg.pose.pose.orientation.z * odom_msg.pose.pose.orientation.z;
        EXPECT_NEAR(q_norm_sq, 1.0, 0.01) << "Quaternion not normalized";
    }
}

TEST_F(TestOdometerSensorSimulator, TestOdometryPositionProgression)
{
    std::vector<nav_msgs::msg::Odometry> messages;
    
    auto subscription = node_p->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        [&messages](const nav_msgs::msg::Odometry::SharedPtr msg) {
            if (messages.size() < 5) { // Collect first 5 messages
                messages.push_back(*msg);
            }
        });

    // Wait for multiple messages to arrive
    for (int i = 0; i < 20; ++i) {
        rclcpp::spin_some(node_p);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        if (messages.size() >= 5) break;
    }

    EXPECT_GE(messages.size(), 2U) << "Not enough messages collected";

    if (messages.size() >= 2) {
        // Position should change between messages (unless robot is stationary in CSV)
        // We verify that timestamps are different
        EXPECT_NE(messages[0].header.stamp.nanosec, messages[1].header.stamp.nanosec);
        EXPECT_EQ(messages[0].header.stamp.sec, messages[1].header.stamp.sec);
    }
}

TEST_F(TestOdometerSensorSimulator, TestOdometryCovarianceMatrixPopulation)
{
    std::vector<nav_msgs::msg::Odometry> messages;
    
    auto subscription = node_p->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        [&messages](const nav_msgs::msg::Odometry::SharedPtr msg) {
            messages.push_back(*msg);
        });

    for (int i = 0; i < 50; ++i) {
        rclcpp::spin_some(node_p);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    EXPECT_GT(messages.size(), 20U);

    if (!messages.empty()) {
        const auto& odom_msg = messages.back();

        // Pose covariance should have 36 elements (6x6 matrix)
        EXPECT_EQ(odom_msg.pose.covariance.size(), 36U);
        
        // Specific covariance elements should be populated (diagonal elements)
        // [0]=X, [7]=Y, [14]=Z, [21]=Roll, [28]=Pitch, [35]=Yaw
        EXPECT_GT(odom_msg.pose.covariance[0], 0.0) << "X covariance not set";
        EXPECT_GT(odom_msg.pose.covariance[7], 0.0) << "Y covariance not set";
        EXPECT_GT(odom_msg.pose.covariance[14], 0.0) << "Z covariance not set";
        
        // Twist covariance should also be populated
        EXPECT_EQ(odom_msg.twist.covariance.size(), 36U);
        EXPECT_GT(odom_msg.twist.covariance[0], 0.0) << "Twist X covariance not set";
        EXPECT_GT(odom_msg.twist.covariance[7], 0.0) << "Twist Y covariance not set";
    }
}

TEST_F(TestOdometerSensorSimulator, TestNodeRobustnessWithRapidSpinning)
{
    std::vector<nav_msgs::msg::Odometry> messages;
    
    auto subscription = node_p->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 100,
        [&messages](const nav_msgs::msg::Odometry::SharedPtr msg) {
            messages.push_back(*msg);
        });

    // Rapid spinning should not crash the node
    for (int i = 0; i < 200; ++i) {
        rclcpp::spin_some(node_p);
        // No sleep - stress test
    }

    // Should still be operating
    EXPECT_TRUE(node_p) << "Node crashed during rapid spinning";
}

int main(int argc, char * argv[]) {
    testing::InitGoogleTest(&argc, argv);
    int tests_stats = RUN_ALL_TESTS();
    return tests_stats;
}
