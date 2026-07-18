cmake_minimum_required(VERSION 3.8)
project(udemy_ros2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)



include_directories(
  include
  # ${rclcpp_INCLUDE_DIRS}
  # ${std_msgs_INCLUDE_DIRS}
  # ${sensor_msgs_INCLUDE_DIRS}
  # ${nav_msgs_INCLUDE_DIRS}
)
# create executables 
# add_executable(publisher src/publisher.cpp)
# add_executable(range_publisher_demo src/range_publisher_demo.cpp)
# add_executable(odometer_publisher_demo src/odometer_publisher_demo.cpp)
add_executable(stationary_detector_publisher src/stationary_detector_publisher.cpp src/odometer_publisher_demo.cpp src/range_publisher_demo.cpp)
add_executable(robot_status_subscriber_simulator src/robot_status_subscriber_simulator.cpp)


# ament_target_dependencies(publisher rclcpp std_msgs)
# add_executable(subscriber src/subscriber.cpp)
# ament_target_dependencies(subscriber rclcpp std_msgs)
# add_executable(rpm_subscriber src/rpm_subscriber.cpp)
# add_executable(rpm_publisher src/rpm_publisher.cpp)
# add_executable(speed_subscriber src/speed_subscriber.cpp)
# ament_target_dependencies(speed_subscriber rclcpp std_msgs)
# ament_target_dependencies(rpm_subscriber rclcpp std_msgs)
# ament_target_dependencies(rpm_publisher rclcpp std_msgs)

# ament_target_dependencies(range_publisher_demo rclcpp sensor_msgs)
# ament_target_dependencies(odometer_publisher_demo rclcpp nav_msgs)
ament_target_dependencies(stationary_detector_publisher rclcpp nav_msgs sensor_msgs std_msgs)
ament_target_dependencies(robot_status_subscriber_simulator rclcpp nav_msgs sensor_msgs std_msgs)


# config for ROS2 cli tools  
install(TARGETS
  stationary_detector_publisher
  robot_status_subscriber_simulator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # find_package(ament_lint_auto REQUIRED) !!
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE) !!
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE) !!
  # ament_lint_auto_find_test_dependencies() !!
  # ament_add_gtest(test_range_sensor_minimal ./test/gtest/test_range_sensor_minimal.cpp)
  # ament_target_dependencies(test_range_sensor_minimal rclcpp sensor_msgs)
endif()

ament_package()
