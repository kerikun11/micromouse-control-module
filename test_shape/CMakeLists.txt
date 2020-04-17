include_directories(../inc)
file(GLOB SRCS *.cpp)
add_executable(test_shape ${SRCS})
add_custom_target(shape
  COMMAND ${CMAKE_BINARY_DIR}/test_shape/test_shape
  DEPENDS test_shape
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
add_custom_target(shape_plot
  COMMAND ${CMAKE_BINARY_DIR}/test_shape/test_shape &&
    cd ${CMAKE_HOME_DIRECTORY} &&
    python ${CMAKE_HOME_DIRECTORY}/test_shape/plot.py
  DEPENDS test_shape
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
