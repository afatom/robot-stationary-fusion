cmake_minimum_required(VERSION 3.8)
project(sensors_fusion_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_range_publisher_test ./test/range_publisher_test.cpp)
  ament_add_gtest(${PROJECT_NAME}_odometer_publisher_test ./test/odometer_publisher_test.cpp)
  target_include_directories(${PROJECT_NAME}_range_publisher_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../udemy_ros2_pkg/src>
    $<INSTALL_INTERFACE:include>
  )
  # Compile the implementation directly into the test
  target_sources(${PROJECT_NAME}_range_publisher_test PRIVATE ../../udemy_ros2_pkg/src/range_publisher_demo.cpp)
  target_sources(${PROJECT_NAME}_odometer_publisher_test PRIVATE ../../udemy_ros2_pkg/src/odometer_publisher_demo.cpp)
  ament_target_dependencies(${PROJECT_NAME}_range_publisher_test rclcpp sensor_msgs nav_msgs std_msgs)
  ament_target_dependencies(${PROJECT_NAME}_odometer_publisher_test rclcpp sensor_msgs nav_msgs std_msgs)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  
  # set(ament_cmake_copyright_FOUND TRUE) !!----//
  
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  
  # set(ament_cmake_cpplint_FOUND TRUE) !!----//
  
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
