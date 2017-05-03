cmake_minimum_required(VERSION 2.8.3)
project(cse180final)

find_package( catkin REQUIRED COMPONENTS roscpp )
find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
## add_action_files(DIRECTORY action FILES DoDishes.action)
## generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package()

include_directories( include ${catkin_INCLUDE_DIRS} )


## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

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
#  LIBRARIES cse180final
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


## create make_plan node executables
# add_executable(make_plan make_plan.cpp)
# target_link_libraries(make_plan ${catkin_LIBRARIES})

## create detect post node executables
# add_executable(detectpost detectpost.cpp)
# target_link_libraries(detectpost ${catkin_LIBRARIES})

add_executable(scan_cloud scan_cloud.cpp)
target_link_libraries(scan_cloud ${catkin_LIBRARIES})








