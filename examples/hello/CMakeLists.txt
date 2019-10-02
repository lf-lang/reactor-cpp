add_executable(hello main.cc)
target_link_libraries(hello dear ${CMAKE_THREAD_LIBS_INIT})
