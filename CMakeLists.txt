## brief: CMake Configuration for MicroMouse Control Module
## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2021.01.03
cmake_minimum_required(VERSION 3.13)
project("MicroMouse Control Module" CXX)

## cmake options
option(BUILD_PYBIND11 "build pybind11" ON)
option(BUILD_DOCS "build documentation" ON)
option(BUILD_TEST "build unit test" ON)
option(BUILD_EXAMPLES "build example projects" ON)

## global build options
set(CMAKE_CXX_STANDARD 17) # enable option -std=c++17
set(CMAKE_CXX_EXTENSIONS OFF) # without compiler extensions like gnu++17

## make a interface library (header only library)
set(MICROMOUSE_CONTROL_MODULE "mmcm")
add_library(${MICROMOUSE_CONTROL_MODULE} INTERFACE)
target_include_directories(${MICROMOUSE_CONTROL_MODULE} INTERFACE include)
target_compile_definitions(${MICROMOUSE_CONTROL_MODULE} INTERFACE
  _USE_MATH_DEFINES # for use of M_PI in <cmath>
)
target_compile_options(${MICROMOUSE_CONTROL_MODULE} INTERFACE
  -fdiagnostics-color=always # colorized output for gcc
)

## pybind11
if(BUILD_PYBIND11)
  add_subdirectory(pybind11)
endif()

## documentation
if(BUILD_DOCS)
  add_subdirectory(docs)
endif()

## unit test
if(BUILD_TEST)
  add_subdirectory(test)
endif()

## examples
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

## cpplint
add_custom_target(cpplint
  COMMAND cpplint --quiet --recursive --exclude=build .
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  USES_TERMINAL
)

## cppcheck
add_custom_target(cppcheck
  COMMAND cppcheck --quiet --enable=all --inline-suppr --suppressions-list=cppcheck-suppressions.txt -i build --platform=unix32 --std=c++17 .
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  USES_TERMINAL
)
