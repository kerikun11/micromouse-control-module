# author: Ryotaro Onuki <kerikun11+github@gmail.com>
# date: 2020.06.11

# find packages
find_package(pybind11 REQUIRED)

# give a target name
set(CUSTOM_TARGET_NAME "python")
set(TARGET_NAME "ctrl")
# make a module
file(GLOB SRC_FILES *.cpp)
pybind11_add_module(${TARGET_NAME} ${SRC_FILES})
target_include_directories(${TARGET_NAME}
  PRIVATE ${CMAKE_HOME_DIRECTORY}/include
  PRIVATE ${PYBIND11_INCLUDE_DIR}
)

# make a custom target to plot
add_custom_target(${CUSTOM_TARGET_NAME}_plot
  COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/plot.py
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  USES_TERMINAL
)
