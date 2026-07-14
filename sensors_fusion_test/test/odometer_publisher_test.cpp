
#include <memory>
#include <string>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"

#include "../../udemy_ros2_pkg/src/odometer_publisher_demo.hpp"


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
        params.push_back(rclcpp::Parameter("odometer_sensor_freq_hz", 80.0));
        options.parameter_overrides(params);

        node_p = std::make_shared<OdometerSensorSimulator>(options);
    }

    void TearDown() override {
        node_p.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<OdometerSensorSimulator> node_p;
};


TEST_F(TestOdometerSensorSimulator, TestNodeCreation)
{
    EXPECT_EQ(std::string(node_p->get_name()), std::string("odometer_sensor_publisher_simulator"));
    auto pub_ep = node_p->get_publishers_info_by_topic("/peak_detection/channel_0/odometry");
    // expected num of publishers 1
    EXPECT_EQ(pub_ep.size(), 1U);
}

TEST_F(TestOdometerSensorSimulator, TestNodeParamsValidation)
{
    // Validate the parameters
    auto csv_path_param = node_p->get_parameter("csv_path");
    EXPECT_EQ(csv_path_param.as_string(), "/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf_test.csv");

    auto sensor_freq_param = node_p->get_parameter("odometer_sensor_freq_hz");
    EXPECT_DOUBLE_EQ(sensor_freq_param.as_double(), 80.0);
}


int main(int argc, char * argv[]) {
    testing::InitGoogleTest(&argc, argv);
    int tests_stats = RUN_ALL_TESTS();
    return tests_stats;
}
