# author: Ryotaro Onuki <kerikun11+github@gmail.com>
# date: 2020.06.11

# find packages
find_package(pybind11 REQUIRED)

# make a python module
set(MODULE_NAME "ctrl")
file(GLOB SRC_FILES *.cpp)
pybind11_add_module(${MODULE_NAME} ${SRC_FILES})
target_include_directories(${MODULE_NAME}
  PRIVATE ${CMAKE_HOME_DIRECTORY}/include
  PRIVATE ${PYBIND11_INCLUDE_DIR}
)

# give a name
set(CUSTOM_TARGET_NAME "python")
# find python interpreter
find_package(Python3 REQUIRED)
# make a custom target to plot
add_custom_target(${CUSTOM_TARGET_NAME}_plot
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/plot.py
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  USES_TERMINAL
)
