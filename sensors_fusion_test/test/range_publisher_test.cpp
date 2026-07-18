
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "../../udemy_ros2_pkg/include/udemy_ros2_pkg/range_publisher_demo.hpp"
#include "../../udemy_ros2_pkg/include/udemy_ros2_pkg/sensor_fusion_gconstants.hpp"


// Test fixture for AcousticSensorSimulator
class TestAcousticSensorSimulator : public::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        // 1. Create a NodeOptions container
        rclcpp::NodeOptions options;

        // 2. Programmatically inject parameters using rclcpp::Parameter objects
        std::vector<rclcpp::Parameter> params;
        params.push_back(rclcpp::Parameter("csv_path", std::string("/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf_test.csv")));
        params.push_back(rclcpp::Parameter("range_sensor_freq_hz", 15.0));
        options.parameter_overrides(params);

        node_p = std::make_shared<AcousticSensorSimulator>(options);
    }

    void TearDown() override {
        node_p.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<AcousticSensorSimulator> node_p;
};

class TestAcousticSensorSimulatorB : public::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }

    void TearDown() override {
        rclcpp::shutdown();
    }

    std::shared_ptr<AcousticSensorSimulator> node_p;
};

TEST_F(TestAcousticSensorSimulator, TestNodeCreation)
{
    EXPECT_EQ(std::string(node_p->get_name()), std::string("acoustic_sensor_publisher_simulator"));
    auto pub_ep = node_p->get_publishers_info_by_topic("/peak_detection/channel_0/range");
    // expected num of publishers 1
    EXPECT_EQ(pub_ep.size(), 1U);
}

TEST_F(TestAcousticSensorSimulator, TestNodeParamsValidation)
{
    // Validate the parameters
    auto csv_path_param = node_p->get_parameter("csv_path");
    EXPECT_EQ(csv_path_param.as_string(), "/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf_test.csv");

    auto sensor_freq_param = node_p->get_parameter("range_sensor_freq_hz");
    EXPECT_DOUBLE_EQ(sensor_freq_param.as_double(), 15.0);
}

TEST_F(TestAcousticSensorSimulator, TestDefaultFrequencyParameter)
{
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> params;
    params.push_back(rclcpp::Parameter("csv_path", std::string("/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf_test.csv")));
    options.parameter_overrides(params);

    auto node = std::make_shared<AcousticSensorSimulator>(options);
    auto freq_param = node->get_parameter("range_sensor_freq_hz");
    EXPECT_DOUBLE_EQ(freq_param.as_double(), 20.0);
    node.reset();
}

TEST_F(TestAcousticSensorSimulator, TestLowFrequencyOperation)
{
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> params;
    params.push_back(rclcpp::Parameter("csv_path", std::string("/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf_test.csv")));
    params.push_back(rclcpp::Parameter("range_sensor_freq_hz", 5.0));
    options.parameter_overrides(params);

    auto node = std::make_shared<AcousticSensorSimulator>(options);
    auto freq_param = node->get_parameter("range_sensor_freq_hz");
    EXPECT_DOUBLE_EQ(freq_param.as_double(), 5.0);
    node.reset();
}

TEST_F(TestAcousticSensorSimulator, TestHighFrequencyOperation)
{
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> params;
    params.push_back(rclcpp::Parameter("csv_path", std::string("/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf_test.csv")));
    params.push_back(rclcpp::Parameter("range_sensor_freq_hz", 100.0));
    options.parameter_overrides(params);

    auto node = std::make_shared<AcousticSensorSimulator>(options);
    auto freq_param = node->get_parameter("range_sensor_freq_hz");
    EXPECT_DOUBLE_EQ(freq_param.as_double(), 100.0);
    node.reset();
}

TEST_F(TestAcousticSensorSimulator, TestRangeMessageStructure)
{
    std::vector<sensor_msgs::msg::Range> messages;

    auto subscription = node_p->create_subscription<sensor_msgs::msg::Range>(
        "/peak_detection/channel_0/range", 10,
        [&messages](const sensor_msgs::msg::Range::SharedPtr msg) {
            messages.push_back(*msg);
        });

    for (int i = 0; i < 20; ++i) {
        rclcpp::spin_some(node_p);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (!messages.empty()) {
            break;
        }
    }

    ASSERT_FALSE(messages.empty()) << "No range messages received";
    const auto &msg = messages.front();

    EXPECT_EQ(msg.header.frame_id, "acoustic_sensor_link");
    EXPECT_EQ(msg.radiation_type, sensor_msgs::msg::Range::ULTRASOUND);
    EXPECT_FLOAT_EQ(msg.field_of_view, fusion_consts::RANGE_SENSOR_FIELD_OF_VIEW_RAD);
    EXPECT_FLOAT_EQ(msg.min_range, fusion_consts::RANGE_SENSOR_MIN_RANGE_M);
    EXPECT_FLOAT_EQ(msg.max_range, fusion_consts::RANGE_SENSOR_MAX_RANGE_M);
    EXPECT_TRUE(std::isfinite(msg.range));
    EXPECT_GT(msg.range, 0.0f);
    EXPECT_LE(msg.range, 10.0f);
}

TEST_F(TestAcousticSensorSimulator, TestRangeMessagesArePublishedOverTime)
{
    std::vector<sensor_msgs::msg::Range> messages;

    auto subscription = node_p->create_subscription<sensor_msgs::msg::Range>(
        "/peak_detection/channel_0/range", 10,
        [&messages](const sensor_msgs::msg::Range::SharedPtr msg) {
            messages.push_back(*msg);
        });

    for (int i = 0; i < 20; ++i) {
        rclcpp::spin_some(node_p);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (messages.size() >= 3) {
            break;
        }
    }

    EXPECT_GE(messages.size(), 3U) << "Expected multiple range messages over time";
}

TEST_F(TestAcousticSensorSimulator, TestRangeValuesStayWithinConfiguredBounds)
{
    std::vector<sensor_msgs::msg::Range> messages;

    auto subscription = node_p->create_subscription<sensor_msgs::msg::Range>(
        "/peak_detection/channel_0/range", 10,
        [&messages](const sensor_msgs::msg::Range::SharedPtr msg) {
            messages.push_back(*msg);
        });

    for (int i = 0; i < 20; ++i) {
        rclcpp::spin_some(node_p);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (messages.size() >= 5) {
            break;
        }
    }

    ASSERT_GE(messages.size(), 1U);
    for (const auto &msg : messages) {
        EXPECT_TRUE(std::isfinite(msg.range));
        EXPECT_GE(msg.range, 0.1);
        EXPECT_LE(msg.range, 10.0);
    }
}

TEST_F(TestAcousticSensorSimulator, TestNodeRobustnessWithRapidSpinning)
{
    std::vector<sensor_msgs::msg::Range> messages;

    auto subscription = node_p->create_subscription<sensor_msgs::msg::Range>(
        "/peak_detection/channel_0/range", 100,
        [&messages](const sensor_msgs::msg::Range::SharedPtr msg) {
            messages.push_back(*msg);
        });

    for (int i = 0; i < 200; ++i) {
        rclcpp::spin_some(node_p);
    }

    EXPECT_TRUE(node_p) << "Node crashed during rapid spinning";
}


int main(int argc, char * argv[]) {
    testing::InitGoogleTest(&argc, argv);
    int tests_stats = RUN_ALL_TESTS();
    return tests_stats;
}
