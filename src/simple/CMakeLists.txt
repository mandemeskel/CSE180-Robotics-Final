cmake_minimum_required(VERSION 2.8.3)
project(simple)

find_package( catkin REQUIRED COMPONENTS roscpp )

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
add_executable(followplan followplan.cpp)
target_link_libraries(followplan ${catkin_LIBRARIES})


## create moveongrid executables
add_executable(moveongrid moveongrid.cpp)
target_link_libraries(moveongrid ${catkin_LIBRARIES})


## create move around node executables
add_executable(move_around move_around.cpp)
target_link_libraries(move_around ${catkin_LIBRARIES})
