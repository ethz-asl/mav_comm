cmake_minimum_required(VERSION 3.14.4)
project(mav_system_msgs)

cmake_policy(SET CMP0148 OLD)

find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CpuInfo.msg
  msg/ProcessInfo.msg
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
