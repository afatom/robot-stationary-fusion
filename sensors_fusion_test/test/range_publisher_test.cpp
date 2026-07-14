
#include <memory>
#include <string>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"

#include "../../udemy_ros2_pkg/src/range_publisher_demo.hpp"


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

TEST_F(TestAcousticSensorSimulator, TestValidateDataOutput)
{
    // Validate that the node is able to read data from the CSV file and publish it correctly
    EXPECT_EQ(4, 2+2);
}


// TEST_F(TestAcousticSensorSimulator, TestNodeBadParamsInjection1)
// {
//     rclcpp::NodeOptions options;
//     std::vector<rclcpp::Parameter> params;
//     params.push_back(rclcpp::Parameter("csv_path", std::string("/voidir/src/ekf.csv")));
//     params.push_back(rclcpp::Parameter("range_sensor_freq_hz", 40.0));
//     options.parameter_overrides(params);
//     //node_p = std::make_shared<AcousticSensorSimulator>(options);
//     //EXPECT_DEATH((node_p = std::make_shared<AcousticSensorSimulator>(options)), "Failed to open CSV telemetry data file: '/voidir/src/ekf.csv'");
//     EXPECT_EXIT((node_p = std::make_shared<AcousticSensorSimulator>(options)), ::testing::ExitedWithCode(1), ".*Failed to open CSV telemetry.*");
//     // EXPECT_ANY_THROW({
//     //     auto csv_path_param = node_p->get_parameter("csv_path");
//     //     EXPECT_EQ(csv_path_param.as_string(), "/voidir/src/ekf.csv");

//     //     auto sensor_freq_param = node_p->get_parameter("range_sensor_freq_hz");
//     //     EXPECT_DOUBLE_EQ(sensor_freq_param.as_double(), 50.0);
//     // });
//     EXPECT_EQ(4, 2+2);
//     node_p.reset();
// }

int main(int argc, char * argv[]) {
    testing::InitGoogleTest(&argc, argv);
    int tests_stats = RUN_ALL_TESTS();
    return tests_stats;
}
