cmake_minimum_required(VERSION 3.10)
project(simple_action)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    hello_action_type 	# need this for ROS action
    actionlib 		# need this to execute ROS action
)

# Declare ROS action
#add_action_files(#
#	DIRECTORY action#
#	FILES 
#		MoveTo.action		
#)

#generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
	INCLUDE_DIRS    
  	LIBRARIES 
  	CATKIN_DEPENDS
    	roscpp
    	hello_action_type
    	actionlib
  	DEPENDS 
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}	
)


## make the publisher executable
add_executable(simple_action_server src/server.cpp )
target_link_libraries(simple_action_server ${catkin_LIBRARIES} )

add_executable(simple_action_client src/client.cpp )
target_link_libraries(simple_action_client ${catkin_LIBRARIES} )


#############
## Install ##
#############

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
