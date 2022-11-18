cmake_minimum_required(VERSION 3.10)
project(hello_action_client)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)



find_package(Eigen3 REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp  
    actionlib 			# need this to execute ROS action
    hello_action_type
)

catkin_package(
	INCLUDE_DIRS    
		include
	 	${EIGEN_INCLUDE_DIRS}
  	LIBRARIES 
  	CATKIN_DEPENDS
    	roscpp   
    	actionlib 
		hello_action_type
  	DEPENDS  		
)

include_directories(
	include
	${EIGEN_INCLUDE_DIRS}	
	${catkin_INCLUDE_DIRS}	
)

add_executable(${PROJECT_NAME}
	src/client.cpp
  	src/ActionClient.cpp  
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )

#############
## Install ##
#############

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)