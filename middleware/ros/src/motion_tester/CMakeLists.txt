cmake_minimum_required(VERSION 2.8.3)
project(motion_tester)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    motion_msgs
    roscpp
)

catkin_package(CATKIN_DEPENDS
    std_msgs
)

include_directories(include
${catkin_INCLUDE_DIRS}
    ../../../../platform/
)

add_executable(motion_tester
    motion_tester.cpp
    ../../../../common/system.c
    ../../../../common/commander.c
)

target_link_libraries(motion_tester ${catkin_LIBRARIES})


