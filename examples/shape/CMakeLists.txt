## author: Ryotaro Onuki <kerikun11+github@gmail.com>
file(GLOB SRCS *.cpp)
add_executable(example_shape ${SRCS})
target_include_directories(example_shape PRIVATE ${CMAKE_HOME_DIRECTORY}/include)
## shape
add_custom_target(shape
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/example_shape
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_shape
  USES_TERMINAL
)
add_custom_target(shape_plot python3 ${CMAKE_CURRENT_SOURCE_DIR}/plot.py
  WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}
)
