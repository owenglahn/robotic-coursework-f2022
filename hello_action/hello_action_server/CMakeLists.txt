cmake_minimum_required(VERSION 3.10)
project(hello_action_server)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    gazebo_msgs    
    actionlib 		# need this to execute ROS action
    hello_action_type
)

catkin_package(
	INCLUDE_DIRS    
  	LIBRARIES 
  	CATKIN_DEPENDS
    	roscpp
    	gazebo_msgs
    	actionlib
    	hello_action_type
  	DEPENDS 
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}	
)

#############
## Install ##
#############

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
  src/CubicPolyPlanner.cpp
)
target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_11)

target_link_libraries(${PROJECT_NAME}_core ${catkin_LIBRARIES} )

## make the publisher executable
add_executable(${PROJECT_NAME} src/server.cpp src/ActionServer.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )


