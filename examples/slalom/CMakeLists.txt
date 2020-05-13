## author: Ryotaro Onuki <kerikun11+github@gmail.com>
file(GLOB SRCS *.cpp)
add_executable(example_slalom ${SRCS})
target_include_directories(example_slalom PRIVATE ${CMAKE_HOME_DIRECTORY}/include)
## slalom
add_custom_target(slalom
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/example_slalom
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_slalom
  USES_TERMINAL
)
add_custom_target(slalom_plot python3 ${CMAKE_CURRENT_SOURCE_DIR}/plot.py
  WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}
)
