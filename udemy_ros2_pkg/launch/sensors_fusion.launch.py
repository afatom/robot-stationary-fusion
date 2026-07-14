from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="udemy_ros2_pkg",
            executable="range_publisher_demo",
            name="acoustic_sensor_publisher_simulator",
            parameters=[
                {"csv_path": "/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf.csv"},
                {"range_sensor_freq_hz" : 20.0}
            ]
        ),
        Node(
            package="udemy_ros2_pkg",
            executable="odometer_publisher_demo",
            name="odometer_sensor_simulator",
            parameters=[
                {"csv_path": "/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf.csv"},
                {"odometer_sensor_freq_hz" : 30.0} # msec
            ]
        ),
        Node(
            package="udemy_ros2_pkg",
            executable="stationary_detector_publisher",
            name="hardened_sensor_fusion_node",
            parameters=[
                {"window_duration_sec": 0.5},
                {"velocity_threshold" : 0.02},
                {"range_variance_threshold" : 0.0005},
                {"fusion_loop_rate_hz" : 25.0},
                {"expected_odom_hz" : 30.0},
                {"expected_range_hz" : 20.0},
            ]
        ),
        Node(
            package="udemy_ros2_pkg",
            executable="robot_status_subscriber_simulator",
            name="robot_status_subscriber_simulator_node"
        )
        # Node(
        #     package="udemy_ros2_pkg",
        #     executable="motion_detector_demo",
        #     name="motion_detector_publisher"
        # )
        # Node(
        #     package="udemy_ros2_pkg",
        #     executable="odometer_publisher_demo",
        #     name="odometer_sensor_publisher_simulator",
        #     parameters=[
        #         {"csv_path": "/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf.csv"},
        #         {"odometer_sensor_transmit_freq" : 10}
        #     ]
        # ),
    ])
