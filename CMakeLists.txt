## file: CMakeLists.txt
## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2020.05.13
cmake_minimum_required(VERSION 3.12)
project("MicroMouse Control Module" CXX)

## Build Type
set(CMAKE_BUILD_TYPE Release)

## build option for compiler
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11
add_compile_options(-fdiagnostics-color=always) # colorized output for gcc
add_compile_options(-D_USE_MATH_DEFINES) # for use of M_PI in <cmath>

## additional options
link_libraries(stdc++fs) # link filesystem library in gcc --version < 9

## more warnings
add_compile_options(
  -Wall
  -Wextra
  -fno-common
  -Wdouble-promotion
)

## documentation
add_subdirectory(docs)
## examples
add_subdirectory(examples)
## test
add_subdirectory(test)
## pybind11
add_subdirectory(pybind11)
