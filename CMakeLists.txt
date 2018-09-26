cmake_minimum_required(VERSION 2.8.3)
project(kuka_lpvds_tasks)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    lwr_ros_client
    tf
    roscpp
    std_msgs
    grasp_interface
)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-deprecated-declarations")



# Uncomment if you add some python scripts
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include/planning include/tasks
  LIBRARIES kuka_lpvds_tasks
  CATKIN_DEPENDS lwr_ros_client roscpp std_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########


############ For planning interface node
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/src)

include_directories(include include/tasks ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${SRC_DIR}/planning/simple_actions/linear_cart_action.cpp
                            ${INCLUDE_DIR}/planning/simple_actions/linear_cart_action.h
                            ${SRC_DIR}/tasks/motion_planners/sinkTaskMotionPlanner.cpp
                            ${INCLUDE_DIR}/tasks/sinkTaskMotionPlanner.h)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )

add_executable(planning_console_node  ${SRC_DIR}/planning/client_console_node.cpp)
target_link_libraries(planning_console_node  ${catkin_LIBRARIES}              )


add_executable(planning_action_node ${SRC_DIR}/planning/client_action_node.cpp	)
target_link_libraries(planning_action_node  ${PROJECT_NAME} ${catkin_LIBRARIES})

############ For DS motion planning node
add_executable(sinkTask_motionPlanner_node ${SRC_DIR}/tasks/sinkTask_motionPlanner_node.cpp)
target_link_libraries(sinkTask_motionPlanner_node ${PROJECT_NAME} ${catkin_LIBRARIES})
