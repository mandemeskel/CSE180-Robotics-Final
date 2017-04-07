cmake_minimum_required(VERSION 2.8.3)
project(simple)

find_package( catkin REQUIRED COMPONENTS roscpp )
find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
## add_action_files(DIRECTORY action FILES DoDishes.action)
## generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package()

include_directories( include ${catkin_INCLUDE_DIRS} )

## create drawsquare executables
## add_executable(drawsquare drawsquare.cpp)
## target_link_libraries(drawsquare ${catkin_LIBRARIES})


## create printcolor executables
## add_executable(printcolor printcolor.cpp)
## target_link_libraries(printcolor ${catkin_LIBRARIES})


## create chatter executables
## add_executable(chatter chatter.cpp)
## target_link_libraries(chatter ${catkin_LIBRARIES})


## create followplan executables
## add_executable(followplan followplan.cpp)
## target_link_libraries(followplan ${catkin_LIBRARIES})


## create moveongrid executables
## add_executable(moveongrid moveongrid.cpp)
## target_link_libraries(moveongrid ${catkin_LIBRARIES})


## create move around node executables
## add_executable(move_around move_around.cpp)
## target_link_libraries(move_around ${catkin_LIBRARIES})


## create navigatepoints node executables
add_executable(navigatepoints navigatepoints.cpp)
target_link_libraries(navigatepoints ${catkin_LIBRARIES})


## create navigatepoints node executables
add_executable(getplan getplan.cpp)
target_link_libraries(getplan ${catkin_LIBRARIES})


## create navigatepoints node executables
add_executable(eigens eigens.cpp)
target_link_libraries(eigens ${catkin_LIBRARIES})