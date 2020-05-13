## author: Ryotaro Onuki <kerikun11+github@gmail.com>
file(GLOB SRCS *.cpp)
add_executable(example_feedback ${SRCS})
target_include_directories(example_feedback PRIVATE ${CMAKE_HOME_DIRECTORY}/include)
## feedback
add_custom_target(feedback
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/example_feedback
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_feedback
  USES_TERMINAL
)
