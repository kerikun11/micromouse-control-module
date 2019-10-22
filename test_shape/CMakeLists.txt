include_directories(../inc)
file(GLOB SRCS *.cpp)
add_executable(test_shape ${SRCS})
add_custom_target(shape
  COMMAND ${CMAKE_BINARY_DIR}/test_shape/test_shape
  DEPENDS test_shape
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
