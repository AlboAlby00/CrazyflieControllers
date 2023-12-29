cmake_minimum_required(VERSION 3.5)
project(agent)
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

include_directories(include)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


install(
  TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

#ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
  scripts/agent.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
