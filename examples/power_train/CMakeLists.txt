add_executable(power_train EXCLUDE_FROM_ALL main.cc)
target_link_libraries(power_train dear)
add_dependencies(examples power_train)
