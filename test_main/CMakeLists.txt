include_directories(../inc)
file(GLOB SRCS *.cpp)
add_executable(test_main ${SRCS})
add_custom_target(main
  COMMAND ${CMAKE_BINARY_DIR}/test_main/test_main
  DEPENDS test_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
