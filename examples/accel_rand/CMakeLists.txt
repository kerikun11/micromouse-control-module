## author: Ryotaro Onuki <kerikun11+github@gmail.com>
include_directories(${CMAKE_HOME_DIRECTORY}/include)
## accel_rand
file(GLOB SRCS *.cpp)
add_executable(example_accel_rand ${SRCS})
add_custom_target(accel_rand
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/example_accel_rand
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_accel_rand
)
add_custom_target(accel_rand_plot python ${CMAKE_CURRENT_SOURCE_DIR}/plot.py
  WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}
)
