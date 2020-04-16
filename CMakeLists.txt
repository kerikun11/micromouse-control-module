## author: Ryotaro Onuki @kerikun11
cmake_minimum_required(VERSION 3.1)
project("AccelDesigner" CXX)

## build option for compiler
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

## more warnnings
add_compile_options(-Wfatal-errors -Wall -Wextra)
add_compile_options(-fconcepts)

## subdirectory
add_subdirectory(test_gtest)
add_subdirectory(test_main)
add_subdirectory(test_shape)
add_subdirectory(test_slalom)
add_subdirectory(test_traj)
