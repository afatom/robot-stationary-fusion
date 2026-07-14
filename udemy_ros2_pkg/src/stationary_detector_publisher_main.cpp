#include "stationary_detector_publisher.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardenedSensorFusionNode>());
    rclcpp::shutdown();
    return 0;
}