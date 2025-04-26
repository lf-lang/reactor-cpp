add_executable(ports EXCLUDE_FROM_ALL main.cc)
target_link_libraries(ports reactor-cpp)
add_dependencies(examples ports)
