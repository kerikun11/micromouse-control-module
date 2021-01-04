# author: Ryotaro Onuki <kerikun11+github@gmail.com>
# date: 2020.05.14

# give a name
set(CUSTOM_TARGET_NAME "trajectory")
set(TARGET_NAME example_${CUSTOM_TARGET_NAME})
# make a executable
file(GLOB SRC_FILES *.cpp)
add_executable(${TARGET_NAME} ${SRC_FILES})
target_link_libraries(${TARGET_NAME} PRIVATE ${MICROMOUSE_CONTROL_MODULE})
# make a custom target to run example
add_custom_target(${CUSTOM_TARGET_NAME}
  COMMAND ${TARGET_NAME}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  USES_TERMINAL
)
# find python interpreter
find_package(Python3)
if(NOT Python3_FOUND)
  message(WARNING "Python3 not found in your environment! skipping...")
  RETURN()
endif()
# make a custom target to plot
add_custom_target(${CUSTOM_TARGET_NAME}_plot
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/plot.py
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  USES_TERMINAL
)
