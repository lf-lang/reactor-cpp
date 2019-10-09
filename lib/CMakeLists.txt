set(SOURCE_FILES
  action.cc
  environment.cc
  logical_time.cc
  logging.cc
  port.cc
  reaction.cc
  reactor.cc
  scheduler.cc
  time.cc
  )

add_library(dear SHARED ${SOURCE_FILES})
target_include_directories(dear PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)
target_compile_options(dear PRIVATE -Wall -Wextra)
target_link_libraries(dear ${CMAKE_THREAD_LIBS_INIT})

set_target_properties(dear PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1)

install(TARGETS dear EXPORT DearConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})

install(EXPORT DearConfig DESTINATION share/dear/cmake)

export(TARGETS ${PROJECT_NAME} FILE DearConfig.cmake)
