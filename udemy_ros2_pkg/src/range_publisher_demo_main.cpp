#include "rclcpp/rclcpp.hpp"
#include "range_publisher_demo.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AcousticSensorSimulator>());
    rclcpp::shutdown();
    return 0;
}