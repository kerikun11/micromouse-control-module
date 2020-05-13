## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2020.05.13

## add examples
add_subdirectory(accel)
add_subdirectory(continuous)
add_subdirectory(feedback)
add_subdirectory(shape)
add_subdirectory(slalom)
add_subdirectory(trajectory)

## make warning error
add_compile_options(
  -Werror
  -Wundef
)
## additional options
link_libraries(stdc++fs) # link filesystem library in gcc --version < 9
