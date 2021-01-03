## file: CMakeLists.txt
## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2021.01.03
cmake_minimum_required(VERSION 3.13)
project("MicroMouse Control Module" CXX)

## global build options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11
add_compile_options(-fdiagnostics-color=always) # colorized output for gcc

## make a interface library (header only library)
set(MICROMOUSE_CONTROL_MODULE "mmcm")
add_library(${MICROMOUSE_CONTROL_MODULE} INTERFACE)
target_include_directories(${MICROMOUSE_CONTROL_MODULE} INTERFACE include)
target_compile_definitions(${MICROMOUSE_CONTROL_MODULE}
  INTERFACE _USE_MATH_DEFINES # for use of M_PI in <cmath>
)

## unit test
add_subdirectory(test)
## examples
add_subdirectory(examples)
## pybind11
add_subdirectory(pybind11 EXCLUDE_FROM_ALL)
## documentation
add_subdirectory(docs)
