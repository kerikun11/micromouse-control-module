# author: Ryotaro Onuki <kerikun11+github@gmail.com>
# date: 2020.06.12

find_package(Doxygen)
if(NOT DOXYGEN_FOUND)
  message(WARNING "doxygen not found in your environment! skipping...")
  RETURN()
endif()

# modify doxyfile
set(DOXYGEN_DOXYFILE_ENCODING "UTF-8")
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "README.md")
set(DOXYGEN_STRIP_FROM_PATH ${CMAKE_HOME_DIRECTORY}/include)

# add target to generate docs
doxygen_add_docs(docs
  ${CMAKE_HOME_DIRECTORY}/include
  ${CMAKE_HOME_DIRECTORY}/README.md
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
