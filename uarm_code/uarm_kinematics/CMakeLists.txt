cmake_minimum_required(VERSION 2.8.3)
project(uarm_kinematics)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(orocos_kdl)
find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  rospy
  std_msgs
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS kdl_parser roscpp rospy std_msgs
#  DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(uarm_ik_solver src/hp_chainiksolverpos_nr_jl.cpp src/chainiksolvervel_pinv.cpp src/svd_HH.cpp)
target_link_libraries(uarm_ik_solver ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(uarm_kinematics src/uarm_kinematics.cpp)
target_link_libraries(uarm_kinematics uarm_ik_solver ${catkin_LIBRARIES})

add_executable(uarm_kinematics_pub src/uarm_kinematics_pub.cpp)
target_link_libraries(uarm_kinematics_pub uarm_ik_solver ${catkin_LIBRARIES})

add_executable(uarm_ik_service src/uarm_ik_service.cpp)
target_link_libraries(uarm_ik_service uarm_ik_solver ${catkin_LIBRARIES})


install(
	TARGETS uarm_kinematics uarm_kinematics_pub uarm_ik_service uarm_ik_solver
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


