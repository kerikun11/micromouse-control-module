## author: Ryotaro Onuki <kerikun11+github@gmail.com>
file(GLOB SRCS *.cpp)
add_executable(example_continuous ${SRCS})
target_include_directories(example_continuous PRIVATE ${CMAKE_HOME_DIRECTORY}/include)
## continuous
add_custom_target(continuous
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/example_continuous
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_continuous
  USES_TERMINAL
)
## continuous_plot
add_custom_target(continuous_plot python3 ${CMAKE_CURRENT_SOURCE_DIR}/plot.py
  WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}
)
