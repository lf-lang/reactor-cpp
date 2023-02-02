cmake_minimum_required(VERSION 3.9)

if (DEFINED LF_REACTOR_CPP_SUFFIX)
  # the build is triggered from Lingua Franca
  set(LIB_TARGET "reactor-cpp-${LF_REACTOR_CPP_SUFFIX}")
else()
  set(LIB_TARGET "reactor-cpp")
endif()

project(${LIB_TARGET} LANGUAGES CXX VERSION 0.0.1)

# require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Use thw new CMP0068 policy. This needs to be set explicitly to avoid a warning message
cmake_policy(SET CMP0068 NEW)
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
set(BUILD_WITH_INSTALL_NAME_DIR ON)

if (NOT DEFINED LF_REACTOR_CPP_SUFFIX)
  find_program(CLANG_TIDY clang-tidy)
  if (CLANG_TIDY)
    set(CMAKE_CXX_CLANG_TIDY clang-tidy; -header-filter=reactor-cpp/\(.*\)\\.hh; -warnings-as-errors=*;)
  else ()
    message(WARNING "Please install clang-tidy!")
  endif()
endif()

find_package (Threads)

set(DEFAULT_BUILD_TYPE "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(REACTOR_CPP_TRACE "Enable tracing" OFF)
option(REACTOR_CPP_VALIDATE "Enable runtime validation" ON)
if (NOT DEFINED REACTOR_CPP_LOG_LEVEL)
  set(REACTOR_CPP_LOG_LEVEL 3)
endif()

if(REACTOR_CPP_TRACE)
  find_package(LTTngUST REQUIRED)
endif()

configure_file(include/reactor-cpp/config.hh.in include/reactor-cpp/config.hh @ONLY)

include(GNUInstallDirs)

add_subdirectory(lib)
if(NOT DEFINED LF_REACTOR_CPP_SUFFIX)
  add_subdirectory(examples)
endif()

if (DEFINED LF_REACTOR_CPP_SUFFIX)
  install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIB_TARGET}")
else()
  install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
endif()
