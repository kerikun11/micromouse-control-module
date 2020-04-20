## author: Ryotaro Onuki <kerikun11+github@gmail.com>
include_directories(${CMAKE_HOME_DIRECTORY}/include)
## feedback
file(GLOB SRCS *.cpp)
add_executable(example_feedback ${SRCS})
add_custom_target(feedback
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/example_feedback
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_feedback
)
