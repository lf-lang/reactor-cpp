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
  validate.cc
  )

add_library(reactor-cpp SHARED ${SOURCE_FILES})
target_include_directories(reactor-cpp PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)
if(MSVC)
  target_compile_options(reactor-cpp PRIVATE /W4 /WX)
else()
  target_compile_options(reactor-cpp PRIVATE -Wall -Wextra -pedantic -Werror)
endif()
target_link_libraries(reactor-cpp ${CMAKE_THREAD_LIBS_INIT})

set_target_properties(reactor-cpp PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1)

install(TARGETS reactor-cpp EXPORT ReactorCppConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})

install(EXPORT ReactorCppConfig DESTINATION share/reactor/cmake)

export(TARGETS ${PROJECT_NAME} FILE ReactorCppConfig.cmake)
