cmake_minimum_required(VERSION 3.10)
project(inverse_kinematic_controller)

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
		sensor_msgs
		genmsg
		std_msgs
)

find_package(Eigen3 REQUIRED)						# find eigen3
find_package(pinocchio REQUIRED)					# find pinocchio
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30") 	# something for pinocchio

add_service_files(
  FILES
  move_robot.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
	INCLUDE_DIRS
    #	include  
    	${EIGEN3_INCLUDE_DIRS}
    	${PINOCCHIO_INCLUDE_DIRS}
  	LIBRARIES
  		pinocchio    	  		
  	CATKIN_DEPENDS
    	roscpp
    	geometry_msgs    	 	    
		sensor_msgs
		std_msgs
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
	${EIGEN3_INCLUDE_DIRS}		
	${PINOCCHIO_INCLUDE_DIRS}
)

# add_executable(test_pinocchio
# 	test/test_pinocchio.cpp	
# )


# target_link_libraries(test_pinocchio
#   	${catkin_LIBRARIES}
#   	${PINOCCHIO_LIBRARIES}
# )


# add_executable(test_eigen
# 	test/test_eigen.cpp	
# )

# target_link_libraries(test_eigen
#   	${catkin_LIBRARIES}  	
# )



# add_executable(test_orientations
# 	test/test_orientations.cpp	
# )
# target_link_libraries(test_orientations
#   	${catkin_LIBRARIES}  	
#   	${PINOCCHIO_LIBRARIES}
# )


#############
## Install ##
#############

# Mark cpp header files for installation
install(
  	DIRECTORY include/${PROJECT_NAME}/
 	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  	FILES_MATCHING PATTERN "*.hpp"
)

## make the publisher executable
add_executable(${PROJECT_NAME}_server src/main.cpp)
target_link_libraries(${PROJECT_NAME}_server ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES} )
add_dependencies(${PROJECT_NAME}_server cubic_polynomial_planner_gencpp)