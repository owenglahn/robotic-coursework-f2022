cmake_minimum_required(VERSION 3.10)
project(hello_action_type)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS    
    actionlib_msgs 	# need this for generating new ROS action types    
)

# Declare ROS action
add_action_files(
	DIRECTORY 
		action
	FILES 
		MoveTo.action		
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
	CATKIN_DEPENDS 
		message_runtime
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
