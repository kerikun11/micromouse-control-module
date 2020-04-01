include_directories(../inc)
file(GLOB SRCS *.cpp)
add_executable(test_main ${SRCS})
add_custom_target(main
  COMMAND ${CMAKE_BINARY_DIR}/test_main/test_main
  DEPENDS test_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
add_custom_target(main_plot
  COMMAND ${CMAKE_BINARY_DIR}/test_main/test_main && cd ${CMAKE_BINARY_DIR} && python ${CMAKE_HOME_DIRECTORY}/test_main/plot.py --file main.csv
  DEPENDS test_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
