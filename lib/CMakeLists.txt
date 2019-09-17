set(DEAR_SRC
  tag.cc
  time.cc
  )

include_directories(
  "${PROJECT_SOURCE_DIR}/include"
  )

add_library(dear SHARED ${DEAR_SRC})
