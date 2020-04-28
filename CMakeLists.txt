cmake_minimum_required(VERSION 3.5)
project(reactor-cpp LANGUAGES CXX VERSION 0.0.1)

# require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

find_package (Threads)

set(DEFAULT_BUILD_TYPE "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(REACTOR_CPP_VALIDATE "Enable runtime validation" ON)
if (NOT DEFINED REACTOR_CPP_LOG_LEVEL)
  set(REACTOR_CPP_LOG_LEVEL 3)
endif()

configure_file(include/reactor-cpp/config.hh.in include/reactor-cpp/config.hh @ONLY)

include(GNUInstallDirs)

add_subdirectory(lib)
add_subdirectory(examples)

install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
