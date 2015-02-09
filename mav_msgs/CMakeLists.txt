cmake_minimum_required(VERSION 2.8.3)
project(mav_msgs)

find_package(catkin REQUIRED message_generation std_msgs geometry_msgs)

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

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

