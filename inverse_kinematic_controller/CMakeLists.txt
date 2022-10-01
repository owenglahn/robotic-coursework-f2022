cmake_minimum_required(VERSION 3.10)
project(robot_model)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#add_definitions(-Wall -Werror) 					# make sure this is not used

#find_package(PkgConfig) 							# find pkg_config
#pkg_search_module(Eigen3 REQUIRED eigen3)			# use pkg_config to find eigen3


## Find catkin macros and libraries
find_package(catkin 
	REQUIRED
	COMPONENTS
		roscpp
		geometry_msgs
)

find_package(Eigen3 REQUIRED)						# find eigen3
find_package(pinocchio REQUIRED)					# find pinocchio
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30") 	# something for pinocchio



catkin_package(
	INCLUDE_DIRS
    #	include  
    	${Eigen3_INCLUDE_DIRS}
    	${PINOCCHIO_INCLUDE_DIRS}
  	LIBRARIES
  		pinocchio    	  		
  	CATKIN_DEPENDS
    	roscpp
    	geometry_msgs    	 	    
  	DEPENDS
  	    	
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	#include
	${catkin_INCLUDE_DIRS}		
	${Eigen3_INCLUDE_DIRS}		
	${PINOCCHIO_INCLUDE_DIRS}
)

add_executable(test_pinocchio
	test/test_pinocchio.cpp	
)


target_link_libraries(test_pinocchio
  	${catkin_LIBRARIES}
  	${PINOCCHIO_LIBRARIES}
)


add_executable(test_eigen
	test/test_eigen.cpp	
)

target_link_libraries(test_eigen
  	${catkin_LIBRARIES}  	
)



add_executable(test_orientations
	test/test_orientations.cpp	
)
target_link_libraries(test_orientations
  	${catkin_LIBRARIES}  	
  	${PINOCCHIO_LIBRARIES}
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
