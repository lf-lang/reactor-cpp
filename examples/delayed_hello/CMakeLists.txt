add_executable(delayed_hello EXCLUDE_FROM_ALL main.cc)
target_link_libraries(delayed_hello reactor-cpp)
add_dependencies(examples delayed_hello)
