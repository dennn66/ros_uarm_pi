cmake_minimum_required(VERSION 2.8.3)
project(uarm_msgs)

find_package(catkin REQUIRED genmsg std_msgs)

add_message_files(FILES
                  Joints.msg
	          GripperPosition.msg
		  UarmJointsState.msg
		  UarmPositionState.msg
                 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   GetUarmIKSolver.srv
 )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(DEPENDS std_msgs)

