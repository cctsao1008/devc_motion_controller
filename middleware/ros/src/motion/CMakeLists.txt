# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(motion)

find_package(catkin REQUIRED COMPONENTS roscpp)

## Declare a catkin package
catkin_package()

include_directories(include
${catkin_INCLUDE_DIRS}
../../../../platform/)

add_executable(motion
motion.cpp
../../../../common/system.c
../../../../common/commander.c
../../../../common/motion_control.c
../../../../common/pid.c
../../../../common/fuzzy.c
../../../../common/nn.c
../../../../common/ga.c
../../../../common/motor_control.c
../../../../platform/omr10/motor_driver_ros.c
)

target_link_libraries(motion ${catkin_LIBRARIES})

# %EndTag(FULLTEXT)%
