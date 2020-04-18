find_package(Doxygen)

if(NOT DOXYGEN_FOUND)
  message(WARNING "doxygen binary couldn't be found")
  RETURN()
endif()

set(DOXYGEN_DOXYFILE_ENCODING "UTF-8")
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "README.md")

doxygen_add_docs(docs
  ${CMAKE_HOME_DIRECTORY}/inc ${CMAKE_HOME_DIRECTORY}/README.md
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
