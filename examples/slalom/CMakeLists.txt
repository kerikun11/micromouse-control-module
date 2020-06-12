# author: Ryotaro Onuki <kerikun11+github@gmail.com>
# date: 2020.05.14

# give a name
set(CUSTOM_TARGET_NAME "slalom")
set(TARGET_NAME example_${CUSTOM_TARGET_NAME})
# make a target
file(GLOB SRC_FILES *.cpp)
add_executable(${TARGET_NAME} ${SRC_FILES})
target_include_directories(${TARGET_NAME}
  PRIVATE ${CMAKE_HOME_DIRECTORY}/include
)
# make a custom target to run
add_custom_target(${CUSTOM_TARGET_NAME}
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS ${TARGET_NAME}
  USES_TERMINAL
)
# find python interpreter
find_package(Python3 REQUIRED)
# make a custom target to plot
add_custom_target(${CUSTOM_TARGET_NAME}_plot
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/plot.py
  WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}
  USES_TERMINAL
)
