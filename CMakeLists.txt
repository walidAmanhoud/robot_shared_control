cmake_minimum_required(VERSION 2.8.3)

project(robot_shared_control)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  std_msgs
  message_generation
  kdl_parser
  dynamic_reconfigure
  svm_grad
  sg_differentiation
  utils
)

find_package(Eigen 3 QUIET) 
if(NOT Eigen_FOUND)
  include(cmake/FindEigen3.cmake)
endif() 

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(cfg/footControl_params.cfg
                                     cfg/falconControl_params.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation kdl_parser dynamic_reconfigure svm_grad sg_differentiation utils
  # DEPENDS Eigen
 )

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(foot_control src/footControl_node.cpp src/FootControl.cpp)
target_link_libraries(foot_control ${catkin_LIBRARIES})
add_dependencies(foot_control ${PROJECT_NAME}_gencpp)

add_executable(falcon_control src/falconControl_node.cpp src/FalconControl.cpp)
target_link_libraries(falcon_control ${catkin_LIBRARIES})
add_dependencies(falcon_control ${PROJECT_NAME}_gencpp)

FILE(GLOB SRCS src/FootControl.cpp src/FalconControl.cpp)
FILE(GLOB INCS include/FootControl.h include/FalconControl.h)

add_library(${PROJECT_NAME} ${SRCS} ${INCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})