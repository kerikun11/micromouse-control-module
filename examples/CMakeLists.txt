## author: Ryotaro Onuki <kerikun11+github@gmail.com>
include_directories(${CMAKE_HOME_DIRECTORY}/include)
## accel
add_executable(accel_main accel_main)
add_custom_target(accel ${CMAKE_CURRENT_BINARY_DIR}/accel_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS accel_main
)
add_custom_target(accel_plot python ${CMAKE_CURRENT_SOURCE_DIR}/accel_plot.py
  WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}
)
## shape
add_executable(shape_main shape_main)
add_custom_target(shape ${CMAKE_CURRENT_BINARY_DIR}/shape_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS shape_main
)
add_custom_target(shape_plot python ${CMAKE_CURRENT_SOURCE_DIR}/shape_plot.py
  WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}
)
## slalom
add_executable(slalom_main slalom_main)
add_custom_target(slalom ${CMAKE_CURRENT_BINARY_DIR}/slalom_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS slalom_main
)
add_custom_target(slalom_plot python ${CMAKE_CURRENT_SOURCE_DIR}/slalom_plot.py
  WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}
)
## feedback
add_executable(feedback_main feedback_main)
add_custom_target(feedback ${CMAKE_CURRENT_BINARY_DIR}/feedback_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS feedback_main
)
## trajectory
add_executable(trajectory_main trajectory_main)
add_custom_target(trajectory ${CMAKE_CURRENT_BINARY_DIR}/trajectory_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS trajectory_main
)
