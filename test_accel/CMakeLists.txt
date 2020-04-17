include_directories(../inc)
file(GLOB SRCS *.cpp)
add_executable(test_accel ${SRCS})
add_custom_target(accel
  COMMAND ${CMAKE_BINARY_DIR}/test_accel/test_accel
  DEPENDS test_accel
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
add_custom_target(accel_plot
  COMMAND ${CMAKE_BINARY_DIR}/test_accel/test_accel && cd ${CMAKE_BINARY_DIR} && python ${CMAKE_HOME_DIRECTORY}/test_accel/plot.py --file accel.csv
  DEPENDS test_accel
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
