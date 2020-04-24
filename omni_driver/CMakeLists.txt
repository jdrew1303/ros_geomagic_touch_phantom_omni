cmake_minimum_required(VERSION 2.8.3)
project(omni_driver)

set (CMAKE_CXX_FLAGS "--std=c++0x ${CMAKE_CXX_FLAGS} -Wall")

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
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  urdf
  moveit_ros_planning
  moveit_ros_move_group
  omni_controller
)

find_package(Boost REQUIRED COMPONENTS thread)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   OmniButtonEvent.msg
   OmniFeedback.msg
   OmniState.msg
   TeleopControl.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs   std_msgs
)

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
#  LIBRARIES raw_omni
  CATKIN_DEPENDS
    geometry_msgs
    omni_description
    roscpp
    std_msgs
    tf
    urdf
    moveit_ros_planning
    moveit_ros_move_group
    omni_controller
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
#  "${EIGEN3_INCLUDE_DIR}"
)

## Declare a cpp library
#add_library()

## Include files that should be shown by the IDE
file(GLOB_RECURSE PROJECT_HEADERS "include/*.h" "launch/*.launch" "src/*.cpp")
add_custom_target(omni_driver_files_for_ide SOURCES
    ${PROJECT_HEADERS}
    "cfg/uncrustify.cfg")

## Declare a cpp executable
add_executable(omni_driver src/omnibase.cpp src/omnifirewire.cpp src/omninode.cpp src/omniethernet.cpp src/raw1394msg.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(omni_driver omni_driver_generate_messages_cpp )

## Specify libraries to link a library or executable target against
target_link_libraries(omni_driver
  raw1394
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  HD
  HDU
)
