## author: Ryotaro Onuki <kerikun11+github@gmail.com>
cmake_minimum_required(VERSION 3.1)
project("MicroMouse Control Module" CXX)

## build option for compiler
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

## for log stream in C++ src
add_compile_options(-DPROJ_DIR="${CMAKE_HOME_DIRECTORY}")

## for gcc --version < 9
link_libraries(stdc++fs)

## more warnnings
add_compile_options(-Wfatal-errors -Wall -Wextra)
add_compile_options(-fconcepts)

## documentation
add_subdirectory(docs)

## example
add_subdirectory(examples)

## test
add_subdirectory(test)
