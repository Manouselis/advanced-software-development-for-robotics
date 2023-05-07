cmake_minimum_required(VERSION 3.8)
project(assignment1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(asdfr_interfaces REQUIRED)
add_executable(brightness_node src/brightness_node.cpp)
ament_target_dependencies(brightness_node rclcpp std_msgs sensor_msgs cv_bridge OpenCV)
add_executable(light_position_node src/light_position_node.cpp)
ament_target_dependencies(light_position_node rclcpp std_msgs sensor_msgs cv_bridge OpenCV asdfr_interfaces)
add_executable(jiwy_unit_test src/jiwy_unit_test.cpp)
ament_target_dependencies(jiwy_unit_test rclcpp std_msgs asdfr_interfaces)
add_executable(jiwy_simulator src/jiwy_simulator.cpp)
ament_target_dependencies(jiwy_simulator rclcpp sensor_msgs std_msgs asdfr_interfaces)
add_executable(jiwy_sequence_controller src/jiwy_sequence_controller.cpp)
ament_target_dependencies(jiwy_sequence_controller rclcpp sensor_msgs std_msgs asdfr_interfaces)

install(TARGETS
   brightness_node
   light_position_node
   jiwy_unit_test
   jiwy_simulator
   jiwy_sequence_controller
   DESTINATION lib/${PROJECT_NAME}
 )
 
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

