set(DEAR_SRC
  action.cc
  port.cc
  reaction.cc
  reactor.cc
  tag.cc
  time.cc
  )

include_directories(
  "${PROJECT_SOURCE_DIR}/include"
  )

add_library(dear SHARED ${DEAR_SRC})
target_compile_options(dear PRIVATE -Wall -Wextra)
