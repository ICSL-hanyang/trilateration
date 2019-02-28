cmake_minimum_required(VERSION 2.8.3)
project(trilateration)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  mavros
  mavros_msgs
  mavros_extras
  geometry_msgs
  geographic_msgs
  sensor_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  msgTagLocation.msg
)


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   mavros_msgs
   geometry_msgs  
   geographic_msgs
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trilateration
  CATKIN_DEPENDS geometry_msgs mavros mavros_extras roscpp std_msgs message_runtime
#realtime_tools
 ## DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(trilateration_node src/trilateration.cpp)
target_link_libraries(trilateration_node ${catkin_LIBRARIES})

