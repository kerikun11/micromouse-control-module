## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2021.01.03

## find Doxygen
find_package(Doxygen)
if(NOT DOXYGEN_FOUND)
  message(WARNING "doxygen not found in your environment! skipping...")
  RETURN()
endif()

## Doxygen Configuration
# set(DOXYGEN_CALLER_GRAPH YES)
set(DOXYGEN_CALL_GRAPH YES)
set(DOXYGEN_ENABLE_PREPROCESSING YES)
set(DOXYGEN_EXPAND_ONLY_PREDEF YES)
set(DOXYGEN_EXTRACT_ALL YES)
set(DOXYGEN_EXTRACT_STATIC YES)
set(DOXYGEN_INLINE_SOURCES YES)
set(DOXYGEN_MACRO_EXPANSION YES)
set(DOXYGEN_OUTPUT_LANGUAGE Japanese)
set(DOXYGEN_PREDEFINED __attribute__((__packed__)))
set(DOXYGEN_STRIP_FROM_PATH ${PROJECT_SOURCE_DIR}/include)
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "README.md")

## set library version
find_package(Git)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --always --tags --dirty
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_DESCRIBE_VERSION
    RESULT_VARIABLE GIT_DESCRIBE_RESULT
    ERROR_VARIABLE GIT_DESCRIBE_ERROR
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  set(DOXYGEN_PROJECT_NUMBER ${GIT_DESCRIBE_VERSION})
endif()

## make a custom target to generate docs
set(TARGET_NAME "docs")
doxygen_add_docs(${TARGET_NAME}
  ${PROJECT_SOURCE_DIR}/docs
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/README.md
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
