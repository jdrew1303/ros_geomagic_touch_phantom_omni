cmake_minimum_required(VERSION 2.8.3)
project(omni_controller)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif ()

if (CMAKE_BUILD_TYPE MATCHES Release OR CMAKE_BUILD_TYPE MATCHES RELEASE)
    # release mode options
    add_compile_options(-O3)
else ()
    # options for any other modes
endif ()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    geometry_msgs
    std_msgs
    joy)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES omni_controller
   CATKIN_DEPENDS std_msgs geometry_msgs joy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    "include"
    "${catkin_INCLUDE_DIRS}"
    "${EIGEN3_INCLUDE_DIR}")

## Declare a C++ library
# add_library(omni_controller
#   src/${PROJECT_NAME}/omni_controller.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(omni_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Include files that should be shown by the IDE
file(GLOB_RECURSE PROJECT_HEADERS "include/*.h")
add_custom_target(omni_controller_files_for_ide SOURCES ${PROJECT_HEADERS})

## Create robot_kinematics library
file(GLOB_RECURSE ROBOT_LIB_CPP ${PROJECT_SOURCE_DIR} src/RobotBase/*.cpp)
add_library(robot_kinematics STATIC ${ROBOT_LIB_CPP})

## Declare a C++ ros node executable
file(GLOB_RECURSE ROS_NODE_CPP ${PROJECT_SOURCE_DIR} src/node/*.cpp)
add_executable(omni_controller ${ROS_NODE_CPP})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(omni_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(omni_controller ${catkin_LIBRARIES} robot_kinematics)

## Create test program
add_executable(robot_test ${PROJECT_SOURCE_DIR}/src/test/main.cpp ${PROJECT_SOURCE_DIR}/src/node/Omni.cpp)
target_link_libraries(robot_test robot_kinematics)

