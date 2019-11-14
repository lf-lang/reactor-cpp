add_executable(count EXCLUDE_FROM_ALL main.cc)
target_link_libraries(count reactor-cpp)
add_dependencies(examples count)
