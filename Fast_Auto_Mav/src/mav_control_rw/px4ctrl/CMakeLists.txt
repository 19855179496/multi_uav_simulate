cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# set(mavros_msgs_DIR /home/c2214/Projects/mav_ros/devel/share/mavros_msgs/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  quadrotor_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  uav_utils
  mav_msgs
  dynamic_reconfigure
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/fake_rc.cfg
)

# set(mavros_msgs_INCLUDE_DIRS /home/c2214/Projects/mav_ros/devel/include/mavros_msgs)
catkin_package(
)

message(${catkin_INCLUDE_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  include
)


add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)
add_executable(paramchange_node 
  src/paramchange_node.cpp
)
add_dependencies(px4ctrl_node quadrotor_msgs)

target_link_libraries(px4ctrl_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
target_link_libraries(paramchange_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
#target_link_libraries(px4ctrl_node ${lapacke_LIBRARIES})
#target_link_libraries(px4ctrl_node mav_nonlinear_mpc_lib)

#add_dependencies(px4ctrl_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
