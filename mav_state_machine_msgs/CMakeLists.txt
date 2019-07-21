cmake_minimum_required(VERSION 2.8.3)
project(mav_state_machine_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)
include_directories(include ${catkin_INCLUDE_DIRS})


#add_message_files(
# FILES
#)
add_service_files(
  FILES
  StartTaskService.srv
)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)