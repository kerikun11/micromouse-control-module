##
## Google Test
##
## Download and unpack googletest at configure time
configure_file(./googletest_download.cmake
               ${CMAKE_BINARY_DIR}/googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . 
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
execute_process(COMMAND ${CMAKE_COMMAND} --build . 
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
## Prevent overriding the parent project's compiler/linker; settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build)

include_directories(${CMAKE_HOME_DIRECTORY}/inc)
include_directories("${gtest_SOURCE_DIR}/include")

file(GLOB SRCS *.cpp)
add_executable(test ${SRCS})
target_link_libraries(test gtest)

add_custom_target(gt
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/test
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS test
)
