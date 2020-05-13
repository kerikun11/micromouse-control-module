## author: Ryotaro Onuki <kerikun11+github@gmail.com>
file(GLOB SRCS *.cpp)
add_executable(example_trajectory ${SRCS})
target_include_directories(example_trajectory PRIVATE ${CMAKE_HOME_DIRECTORY}/include)
## trajectory
add_custom_target(trajectory
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/example_trajectory
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_trajectory
  USES_TERMINAL
)
add_custom_target(trajectory_plot python3 ${CMAKE_CURRENT_SOURCE_DIR}/plot.py
  WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}
)
