cmake_minimum_required(VERSION 3.10)
project(inverse_dynamics_controller)

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
    sensor_msgs
    geometry_msgs
    genmsg
    std_msgs
)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

add_service_files(
  FILES
  move_to.srv
)

generate_messages(DEPENDENCIES)

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
    include
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
    ${EIGEN3_INCLUDE_DIR}
    ${PINOCCHIO_INCLUDE_DIRS}
  LIBRARIES
    pinocchio
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    genmsg
    std_msgs
    geometry_msgs
  DEPENDS

  ## find_package(Eigen3) provides a non standard EIGEN3_INCLUDE_DIR instead of Eigen3_INCLUDE_DIRS.
  ## Therefore, the DEPEND does not work as expected and we need to add the directory to the INCLUDE_DIRS
  # Eigen3

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
  # Set manually because Eigen sets a non standard INCLUDE DIR
  ${EIGEN3_INCLUDE_DIR}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
  src/PotentialField.cpp
)
target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_core
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

## Declare cpp executables
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
  src/InverseDynamicsController.cpp
)

## Specify libraries to link executable targets against
target_link_libraries( ${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${PINOCCHIO_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_gencpp
)
