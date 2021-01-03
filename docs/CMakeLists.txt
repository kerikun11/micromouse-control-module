# author: Ryotaro Onuki <kerikun11+github@gmail.com>
# date: 2020.06.12

# find Doxygen
find_package(Doxygen)
if(NOT DOXYGEN_FOUND)
  message(WARNING "doxygen not found in your environment! skipping...")
  RETURN()
endif()

# modify doxyfile
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "README.md")
set(DOXYGEN_STRIP_FROM_PATH ${PROJECT_SOURCE_DIR}/include)
set(DOXYGEN_EXTRACT_STATIC YES)
set(DOXYGEN_ENABLE_PREPROCESSING YES)
set(DOXYGEN_MACRO_EXPANSION YES)
set(DOXYGEN_EXPAND_ONLY_PREDEF YES)
set(DOXYGEN_PREDEFINED __attribute__((__packed__)))
set(DOXYGEN_CREATE_SUBDIRS YES) # to resolve src code link

# add target to generate docs
set(TARGET_NAME "docs")
doxygen_add_docs(${TARGET_NAME}
  ${PROJECT_SOURCE_DIR}/docs
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/README.md
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
# copy src files to html directory
add_custom_command(
  TARGET ${TARGET_NAME}
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_BINARY_DIR}/html/include
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/examples ${CMAKE_CURRENT_BINARY_DIR}/html/examples
)
