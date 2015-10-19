cmake_minimum_required(VERSION 2.8.3)
project(planning_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs message_generation octomap_msgs trajectory_msgs cmake_modules)
find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

add_message_files(
  FILES
  OctomapScan.msg
  PlanningResponse.msg
  WayPointArray.msg
  WayPoint.msg
  WaypointType.msg
  GPSWayPoint.msg
  GPSWayPointArray.msg
)

add_service_files(
  FILES
  PlannerService.srv
  Octomap.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs sensor_msgs octomap_msgs trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs octomap_msgs trajectory_msgs
)
