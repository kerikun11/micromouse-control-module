## author: Ryotaro Onuki <kerikun11+github@gmail.com>
include_directories(${CMAKE_HOME_DIRECTORY}/include)
## accel
file(GLOB SRCS *.cpp)
add_executable(example_accel ${SRCS})
add_custom_target(accel
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/example_accel
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_accel
)
add_custom_target(accel_plot python ${CMAKE_CURRENT_SOURCE_DIR}/plot.py
  WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}
)
