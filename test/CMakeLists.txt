add_subdirectory(Catch2)

include_directories(
  "${PROJECT_SOURCE_DIR}/include"
  "${PROJECT_SOURCE_DIR}/tests/Catch2/single_include"
  )

add_executable(tests
  action.cc
  input.cc
  main.cc
  output.cc
  reaction.cc
  reactor.cc
  tag.cc
  time.cc
  value_ptr.cc
  )

target_link_libraries(tests Catch2::Catch2 dear)
target_compile_options(tests PRIVATE -Wall -Wextra)

include(CTest)
include(Catch2/contrib/Catch.cmake)
catch_discover_tests(tests)
