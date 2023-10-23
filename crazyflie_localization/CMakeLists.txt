cmake_minimum_required(VERSION 3.8)
project(crazyflie_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include)


add_executable(visual_odomerty_node src/visual_odomerty_node.cpp)
ament_target_dependencies(visual_odomerty_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(visual_odomerty_node ${OpenCV_LIBS})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS visual_odomerty_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
