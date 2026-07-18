from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="udemy_ros2_pkg",
            executable="stationary_detector_publisher",
            name="hardened_sensor_fusion_node",
            parameters=[
                {"window_duration_sec": 1.5}, # 0.5
                {"velocity_threshold" : 0.1}, # 1.0 always yes # 0.06 always no
                {"range_variance_threshold" : 0.1}, # 1.0 always yes # 0.0005 always no
                {"moving_ratio_threshold" : 0.20},
                {"moving_ratio_hysteresis" : 0.05},
                {"variance_hysteresis_ratio" : 0.20},
                {"fusion_loop_rate_hz" : 10.0}, # 25.0
                {"expected_odom_hz" : 30.0},
                {"expected_range_hz" : 25.0},
            ]
        ),
        Node(
            package="udemy_ros2_pkg",
            executable="robot_status_subscriber_simulator",
            name="robot_status_subscriber_simulator_node"
        ),
    ])
