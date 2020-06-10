# author: Ryotaro Onuki <kerikun11+github@gmail.com>
# date: 2020.05.30

# find packages
find_package(Boost COMPONENTS python38 REQUIRED)
if(NOT Boost_FOUND)
  message(STATUS "!!! Boost NOT FOUND !!!")
  RETURN()
endif()

find_package(PythonLibs 3 REQUIRED)
if(NOT PythonLibs_FOUND)
  message(STATUS "!!! PythonLibs NOT FOUND !!!")
  RETURN()
endif()

# give a target name
set(CUSTOM_TARGET_NAME "python")
set(TARGET_NAME ${CUSTOM_TARGET_NAME})
# make a target
file(GLOB SRC_FILES *.cpp)
add_library(${TARGET_NAME} MODULE ${SRC_FILES})
target_include_directories(${TARGET_NAME}
  PRIVATE ${CMAKE_HOME_DIRECTORY}/include
  PRIVATE ${Boost_INCLUDE_DIR}
  PRIVATE ${PYTHON_INCLUDE_DIRS}
)
target_link_libraries(${TARGET_NAME}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
)
# make a python module named "ctrl"
set(CMAKE_SHARED_MODULE_PREFIX "")
set_target_properties(${TARGET_NAME} PROPERTIES
  LIBRARY_OUTPUT_NAME "ctrl"
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# make a custom target to plot
add_custom_target(${CUSTOM_TARGET_NAME}_plot
  COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/plot.py
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  USES_TERMINAL
)
