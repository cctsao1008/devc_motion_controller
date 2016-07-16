# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(motion)

find_package(catkin REQUIRED COMPONENTS roscpp)

## Declare a catkin package
catkin_package()

add_executable(motion motion.cpp)
target_link_libraries(motion ${catkin_LIBRARIES})

# %EndTag(FULLTEXT)%
