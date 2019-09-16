add_subdirectory(Catch2)

include_directories(
  "${PROJECT_SOURCE_DIR}/include"
  "${PROJECT_SOURCE_DIR}/tests/Catch2/single_include/catch2"
  )

add_executable(tests main.cc)
target_link_libraries(tests Catch2::Catch2)

include(CTest)
include(Catch2/contrib/Catch.cmake)
catch_discover_tests(tests)
