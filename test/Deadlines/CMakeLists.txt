cmake_minimum_required(VERSION 3.9)
project(Deadlines VERSION 0.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20 CACHE STRING "The C++ standard is cached for visibility in external tools." FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)

set(LF_MAIN_TARGET Deadlines)

find_package(reactor-cpp PATHS )
find_package(reactor-sdk PATHS )

add_executable(${LF_MAIN_TARGET}
    main.cc
)

include_directories(${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${LF_MAIN_TARGET} reactor-cpp)
target_link_libraries(${LF_MAIN_TARGET} reactor-sdk)

target_compile_options(${LF_MAIN_TARGET} PRIVATE -Wall -Wextra -pedantic)