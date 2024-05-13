make_minimum_required(VERSION 3.8)
project(lifty_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find_library(WIRINGPI_LIB wiringPi)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(fyp_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(wiringPi REQUIRED)
#find_package(example_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


#ament_python_install_package(lifty_nodes)
include_directories(include)
#include_directories(/usr/local/include)

add_executable(hardware_interface_node src/hardware_interface_node.cpp)
add_executable(motor_controller_node src/motor_controller_node.cpp src/Motor.cp>
ament_target_dependencies(hardware_interface_node rclcpp fyp_interfaces)
ament_target_dependencies(motor_controller_node rclcpp fyp_interfaces geometry_>

install(TARGETS
  hardware_interface_node
  motor_controller_node
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
        scripts/lifty_services_node.py
        DESTINATION lib/${PROJECT_NAME} )

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

