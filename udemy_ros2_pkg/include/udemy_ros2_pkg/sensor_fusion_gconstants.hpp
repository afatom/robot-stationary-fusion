#pragma once

#include <string_view> //min c++ ver c++17

namespace fusion_consts
{
    constexpr std::string_view EKF_CSV_DEFAULT_PATH = "/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf.csv";
    constexpr std::string_view EKF_TEST_CSV_DEFAULT_PATH = "/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf_test.csv";
    constexpr int RANGE_SENSOR_MIN_FREQ_HZ = 1;
    constexpr int RANGE_SENSOR_MAX_FREQ_HZ = 100;
    constexpr int ODOMETER_SENSOR_MIN_FREQ_HZ = 1;
    constexpr int ODOMETER_SENSOR_MAX_FREQ_HZ = 100;

    constexpr unsigned CSV_TIME_COLUMN_IDX = 0;
    constexpr unsigned CSV_POS_Z_COLUMN_IDX = 3;
    constexpr unsigned CSV_DIST_FROM_SURFACE_COLUMN_IDX = 3;

    //Range sensor parameters
    constexpr float RANGE_SENSOR_MIN_RANGE_M = 0.1;
    constexpr float RANGE_SENSOR_MAX_RANGE_M = 10.0;
    constexpr float RANGE_SENSOR_NOISE_STD_DEV_M = 0.1;
    constexpr float RANGE_SENSOR_FIELD_OF_VIEW_RAD = 0.1; // ~5.7 degrees

    //Odometer sensor parameters
    constexpr float ODOMETER_SENSOR_MIN_RANGE_M = 0.1;
    constexpr float ODOMETER_SENSOR_MAX_RANGE_M = 10.0;
    constexpr float ODOMETER_SENSOR_NOISE_STD_DEV_M = 0.1;

}