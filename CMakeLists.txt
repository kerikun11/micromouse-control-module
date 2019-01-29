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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

add_subdirectory(test_main)
add_subdirectory(test_csv)

# custom target
add_custom_target(main
  COMMAND ${CMAKE_BINARY_DIR}/test_main/test_main
  DEPENDS test_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
add_custom_target(csv
  COMMAND ${CMAKE_BINARY_DIR}/test_csv/test_csv
  DEPENDS test_csv
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
