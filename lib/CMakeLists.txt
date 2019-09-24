set(DEAR_SRC
  input.cc
  output.cc
  reactor.cc
  tag.cc
  time.cc
  )

include_directories(
  "${PROJECT_SOURCE_DIR}/include"
  )

add_library(dear SHARED ${DEAR_SRC})
target_compile_options(dear PRIVATE -Wall -Wextra)
