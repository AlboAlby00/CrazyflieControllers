cmake_minimum_required(VERSION 3.8)
project(crazyflie_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

set(VO_DEPENDENCIES rclcpp sensor_msgs geometry_msgs cv_bridge visualization_msgs tf2 tf2_geometry_msgs)
file(GLOB VO_SOURCES src/visual_odometry_node.cpp src/my_visual_odometry/*.cpp src/my_geometry/*.cpp src/my_utils/*.cpp src/my_datastructures/*.cpp)
add_executable(visual_odometry_node ${VO_SOURCES})
ament_target_dependencies(visual_odometry_node ${VO_DEPENDENCIES})
target_link_libraries(visual_odometry_node ${OpenCV_LIBS})

set(TF_DEPENDENCIES rclcpp std_msgs tf2 geometry_msgs tf2_ros)
add_executable(tf_broadcaster_node src/tf_broadcaster_node.cpp)
ament_target_dependencies(tf_broadcaster_node ${TF_DEPENDENCIES})


install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}/)


install(TARGETS visual_odometry_node tf_broadcaster_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  crazyflie_localization/camera_aligner.py
  crazyflie_localization/ekf_wrapper.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
