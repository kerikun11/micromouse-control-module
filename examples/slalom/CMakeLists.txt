## author: Ryotaro Onuki <kerikun11+github@gmail.com>
include_directories(${CMAKE_HOME_DIRECTORY}/include)
## slalom
file(GLOB SRCS *.cpp)
add_executable(example_slalom ${SRCS})
add_custom_target(slalom
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/example_slalom
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_slalom
)
add_custom_target(slalom_plot python ${CMAKE_CURRENT_SOURCE_DIR}/plot.py
  WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}
)
