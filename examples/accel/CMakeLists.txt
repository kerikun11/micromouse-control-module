## author: Ryotaro Onuki <kerikun11+github@gmail.com>
file(GLOB SRCS *.cpp)
add_executable(example_accel ${SRCS})
target_include_directories(example_accel PRIVATE ${CMAKE_HOME_DIRECTORY}/include)
## accel
add_custom_target(accel
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/example_accel
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_accel
  USES_TERMINAL
)
## accel_plot
add_custom_target(accel_plot python3 ${CMAKE_CURRENT_SOURCE_DIR}/plot.py
  WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}
)
