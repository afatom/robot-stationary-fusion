#pragma once

#include <string_view> //min c++ ver c++17


namespace sensor_fusion_gconstants
{
    constexpr std::string_view CSV_FP = "/home/adham/ros2_ws/src/udemy_ros2_pkg/src/ekf.csv";
    constexpr int RANGE_SENSOR_MIN_FREQ_HZ = 1;
    constexpr int RANGE_SENSOR_MAX_FREQ_HZ = 100;
    constexpr int ODOMETER_SENSOR_MIN_FREQ_HZ = 1;
    constexpr int ODOMETER_SENSOR_MAX_FREQ_HZ = 100;

    constexpr unsigned CSV_TIME_COLUMN_IDX = 0;
    constexpr unsigned CSV_POS_Z_COLUMN_IDX = 3;
    constexpr unsigned CSV_DIST_FROM_SURFACE_COLUMN_IDX = 3;
}