cmake_minimum_required(VERSION 3.8)
project(gazebo_navstack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(fyp_interfaces REQUIRED) 
find_package(example_interfaces REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_executable(dualshock_node src/dualshock_node.cpp)
add_executable(motors_twist_odom_node src/motors_twist_odom_node.cpp)
ament_target_dependencies(motors_twist_odom_node rclcpp geometry_msgs fyp_interfaces)
ament_target_dependencies(dualshock_node rclcpp geometry_msgs sensor_msgs)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/scan_data_collect.py
  scripts/odom_data_collect.py
  scripts/adjusted_scan_pub.py
  DESTINATION lib/${PROJECT_NAME} )

install(DIRECTORY
  launch
  params
  maps
  rviz2
  description
  config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  dualshock_node
  motors_twist_odom_node
  DESTINATION lib/${PROJECT_NAME})


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
