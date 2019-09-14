#
# MazeLib/CMakeLists.txt
#
cmake_minimum_required(VERSION 3.1)
project("AccelDesigner" CXX)

#
# build option for compiler
#
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wextra")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

add_subdirectory(test_main)
add_subdirectory(test_traj)
add_subdirectory(test_slalom)
