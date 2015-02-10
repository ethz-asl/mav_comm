cmake_minimum_required(VERSION 2.8.3)
project(mav_msgs)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
)

find_package(catkin REQUIRED message_generation std_msgs geometry_msgs)
find_package(Eigen REQUIRED)

add_message_files(
  FILES
  CommandAttitudeThrust.msg
  CommandMotorSpeed.msg
  CommandRateThrust.msg
  CommandRollPitchYawrateThrust.msg
  CommandTrajectory.msg
  CommandVelocityTrajectory.msg
  MotorSpeed.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
  DEPENDS Eigen
)
