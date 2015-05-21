cmake_minimum_required(VERSION 2.8.3)
project(mav_msgs)

find_package(catkin REQUIRED cmake_modules message_generation std_msgs geometry_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

add_message_files(
  FILES
  CommandAttitudeThrust.msg
  CommandMotorSpeed.msg
  CommandRateThrust.msg
  CommandRollPitchYawrateThrust.msg
  CommandTrajectoryPositionYaw.msg
  CommandTrajectoryVelocityYawrate.msg
  MotorSpeed.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include ${EIGEN_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
  DEPENDS Eigen
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
