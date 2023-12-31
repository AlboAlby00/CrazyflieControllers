cmake_minimum_required(VERSION 3.5)
project(crazyflie_controllers)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(crazyflie_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ACADO REQUIRED)

include_directories(
        include
        /usr/local/include
        /usr/local/include/acado
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(attitude_pid_controller src/attitude_pid_controller.cpp src/control_utils/pid.cpp)
ament_target_dependencies(attitude_pid_controller rclcpp crazyflie_msgs geometry_msgs sensor_msgs)

add_executable(position_mpc_controller src/position_mpc_controller.cpp)
ament_target_dependencies(position_mpc_controller rclcpp crazyflie_msgs geometry_msgs sensor_msgs ACADO)

add_executable(position_pid_controller src/position_pid_controller.cpp src/control_utils/pid.cpp)
ament_target_dependencies(position_pid_controller rclcpp crazyflie_msgs geometry_msgs sensor_msgs)

#add_executable(position_mpc_controller src/position_mpc_controller.cpp)
#target_link_libraries(position_mpc_controller acado_toolkit_s)
#ament_target_dependencies(position_mpc_controller rclcpp std_msgs crazyflie_msgs geometry_msgs sensor_msgs)
target_link_libraries(position_mpc_controller ${ACADO_SHARED_LIBRARIES})

install(
  TARGETS
  attitude_pid_controller
  position_pid_controller
  position_mpc_controller
  DESTINATION lib/${PROJECT_NAME}
)

#ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
  scripts/test_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
