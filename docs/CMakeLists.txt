find_package(Doxygen)

if(NOT DOXYGEN_FOUND)
  message(WARNING "not found doxygen :(")
  message("...skipped")
  RETURN()
endif()

set(DOXYGEN_DOXYFILE_ENCODING "UTF-8")
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "README.md")
set(DOXYGEN_STRIP_FROM_PATH ${CMAKE_HOME_DIRECTORY}/include)

doxygen_add_docs(docs
  ${CMAKE_HOME_DIRECTORY}/include
  ${CMAKE_HOME_DIRECTORY}/README.md
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
