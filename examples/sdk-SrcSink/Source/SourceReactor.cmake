set(INCLUDE_FILES 
    "${CMAKE_CURRENT_LIST_DIR}/SourceReactor.hh"
)

set(SOURCE_FILES 
    "${CMAKE_CURRENT_LIST_DIR}/SourceReactor.cc"
)

foreach(file IN LISTS INCLUDE_FILES)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -include ${file}")
endforeach()

target_sources(${LF_MAIN_TARGET} PRIVATE ${SOURCE_FILES})