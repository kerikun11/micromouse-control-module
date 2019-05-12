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

# custom target
add_custom_target(main
  COMMAND ${CMAKE_BINARY_DIR}/test_main/test_main
  DEPENDS test_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
add_custom_target(slalom
  COMMAND ${CMAKE_BINARY_DIR}/test_slalom/test_slalom
  DEPENDS test_slalom
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
add_custom_target(traj
  COMMAND ${CMAKE_BINARY_DIR}/test_traj/test_traj
  DEPENDS test_traj
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
