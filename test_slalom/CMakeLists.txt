include_directories(../inc)
file(GLOB SRCS *.cpp)
add_executable(test_slalom ${SRCS})
add_custom_target(slalom
  COMMAND ${CMAKE_BINARY_DIR}/test_slalom/test_slalom
  DEPENDS test_slalom
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
