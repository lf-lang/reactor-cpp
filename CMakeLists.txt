cmake_minimum_required(VERSION 3.5)

project(dear LANGUAGES CXX VERSION 0.0.1)
enable_testing()

add_subdirectory(lib)
add_subdirectory(test)
add_subdirectory(examples)
