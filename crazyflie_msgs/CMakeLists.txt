cmake_minimum_required(VERSION 3.8)
project(crazyflie_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorVel.msg"
  "msg/AttitudeCommand.msg"
  "msg/PositionCommand.msg"
  "msg/PidTuner.msg"
  "msg/EulerAngle.msg"
  "msg/TargetTransformationVector.msg"
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
