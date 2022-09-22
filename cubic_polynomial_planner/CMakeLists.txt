cmake_minimum_required(VERSION 3.10)
project(cubic_polynomial_planner)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    genmsg
)
    
add_service_files(
  FILES
  my_service.srv
)

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS       
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    genmsg
    add_service_files
    generate_messages
    cubic_polynomial_planner_gencpp
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
)

## make the publisher executable
add_executable(hello_publisher src/publisher.cpp )
target_link_libraries(hello_publisher ${catkin_LIBRARIES} )

## make the subscriber executable
add_executable(hello_subscriber src/subscriber.cpp)
target_link_libraries(hello_subscriber ${catkin_LIBRARIES} )

