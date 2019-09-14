include_directories(../inc)
file(GLOB SRCS *.cpp)
add_executable(test_traj ${SRCS})
add_custom_target(traj
  COMMAND ${CMAKE_BINARY_DIR}/test_traj/test_traj
  DEPENDS test_traj
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
