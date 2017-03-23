cmake_minimum_required(VERSION 2.8.3)
project(omni_description)

find_package(catkin REQUIRED COMPONENTS urdf)

catkin_package()

file(GLOB_RECURSE OMNI_DESCRIPTION_PROJECT_HEADERS "urdf/*")
add_custom_target(omni_description_files_for_ide SOURCES ${OMNI_DESCRIPTION_PROJECT_HEADERS})
