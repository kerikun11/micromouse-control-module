## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2020.05.13

## add warnings in examples
add_compile_options(
  -Wall # Enables a big group of common warnings - see https://clang.llvm.org/docs/DiagnosticsReference.html#wall
  -Wextra # Enables key warnings like -Wunused-parameter and -Wsign-compare - see https://clang.llvm.org/docs/DiagnosticsReference.html#wextra
  -Wundef # Warn if an undefined identifier is evaluated in an #if directive. Such identifiers are replaced with zero.
  -Wdouble-promotion # Give a warning when a value of type float is implicitly promoted to double.
)

## add examples
add_subdirectory(accel)
add_subdirectory(continuous)
add_subdirectory(feedback)
add_subdirectory(shape)
add_subdirectory(slalom)
add_subdirectory(trajectory)
