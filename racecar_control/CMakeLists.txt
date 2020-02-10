cmake_minimum_required(VERSION 2.8.3)
project(racecar_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs  
  geometry_msgs
  sensor_msgs
  message_generation
  visualization_msgs 
  interactive_markers
  occupancy_grid_utils 
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OMPL REQUIRED)

# catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES racecar_control
#  CATKIN_DEPENDS other_catkin_pkg
#DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs tf occupancy_grid_utils ackermann_msgs
)

###########
## Build ##
###########
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++0x" )
#SET(OMPL_INCLUDE_DIRECS /usr/local/include)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${OMPL_INCLUDE_DIRS}, ${catkin_INCLUDE_DIRS},${occupancy_grid_utils_INCLUDE_DIRS} 

)
message(${OMPL_INCLUDE_DIRS})

## Declare a C++ library
# add_library(racecar_control
#   src/${PROJECT_NAME}/racecar_control.cpp
# )

# add_dependencies(racecar_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(racecar_control_node src/racecar_control_node.cpp)

add_executable(laser_to_estop scripts/laser_to_estop.cpp)
target_link_libraries(laser_to_estop ${catkin_LIBRARIES})

add_executable(goal_marker_server scripts/goal_marker_server.cpp)
target_link_libraries(goal_marker_server ${catkin_LIBRARIES})

#SET(OMPL_LIB /usr/local/lib/libompl.so)
#link_directories(/usr/local/lib/)
add_executable(racecar_planner scripts/racecar_planner.cpp scripts/rrtsharp/RRTsharp.cpp scripts/rrtsharp/RRTXstatic.cpp)
target_link_libraries(racecar_planner ${OMPL_LIBRARIES} ${catkin_LIBRARIES})

#message(${OMPL_LIB})

add_executable(point_tracker scripts/point_tracker.cpp)
target_link_libraries(point_tracker ${catkin_LIBRARIES})

add_executable(servo_command_to_simu scripts/servo_command_to_simu.cpp)
target_link_libraries(servo_command_to_simu ${catkin_LIBRARIES})

add_executable(servo_command_to_racecar scripts/servo_command_to_racecar.cpp)
target_link_libraries(servo_command_to_racecar ${catkin_LIBRARIES})


