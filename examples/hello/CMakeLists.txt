add_executable(hello EXCLUDE_FROM_ALL main.cc)
target_link_libraries(hello dear)
add_dependencies(examples hello)
