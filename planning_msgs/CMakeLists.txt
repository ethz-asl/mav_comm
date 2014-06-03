cmake_minimum_required(VERSION 2.8.3)
project(planning_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs message_generation octomap_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  OctomapScan.msg
  PlanningResponse.msg
  WayPointArray.msg
  WayPoint.msg
  WaypointType.msg
)

add_service_files(
  FILES
  PlannerService.srv
  Octomap.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs sensor_msgs octomap_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs octomap_msgs
)
