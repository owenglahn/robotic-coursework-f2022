cmake_minimum_required(VERSION 3.10)
project(base_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
# add_definitions(-Wall -Werror)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    genmsg
    std_msgs
)

add_service_files(
  FILES
  start.srv
)

generate_messages(
  DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
	# include    
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    genmsg
    std_msgs
  DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
  	${catkin_INCLUDE_DIRS}    
  	${Eigen3_INCLUDE_DIRS}  
)


## Declare cpp executables
add_executable(${PROJECT_NAME}  
  src/BaseController.cpp
  src/${PROJECT_NAME}_node.cpp
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)


add_dependencies(${PROJECT_NAME}
 ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
	TARGETS ${PROJECT_NAME} 
 	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
 	DIRECTORY include/${PROJECT_NAME}/
 	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 	FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
 	DIRECTORY doc
 	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)