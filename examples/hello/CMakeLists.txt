add_executable(hello EXCLUDE_FROM_ALL main.cc)
target_link_libraries(hello reactor-cpp)
add_dependencies(examples hello)
