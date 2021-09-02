set(SOURCE_FILES
  action.cc
  assert.cc
  environment.cc
  logical_time.cc
  port.cc
  reaction.cc
  reactor.cc
  scheduler.cc
  time.cc
  )

if(REACTOR_CPP_TRACE)
  set(SOURCE_FILES ${SOURCE_FILES} trace.cc)
endif()

add_library(reactor-cpp SHARED ${SOURCE_FILES})
target_include_directories(reactor-cpp PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include/reactor-cpp>"
  "$<BUILD_INTERFACE:${LTTNGUST_INCLUDE_DIRS}>"
  "$<INSTALL_INTERFACE:include>"
  PRIVATE src)

if(MSVC)
  target_compile_options(reactor-cpp PRIVATE /W4 /WX)
else()
  target_compile_options(reactor-cpp PRIVATE -Wall -Wextra -pedantic -Werror)
endif()

target_link_libraries(reactor-cpp ${CMAKE_THREAD_LIBS_INIT})
if(REACTOR_CPP_TRACE)
  target_link_libraries(reactor-cpp LTTng::UST)
endif()

set_target_properties(reactor-cpp PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1)

install(FILES "${CMAKE_BINARY_DIR}/include/reactor-cpp/config.hh" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/reactor-cpp")
install(TARGETS reactor-cpp EXPORT reactor-cppConfig
    ARCHIVE  DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY  DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME  DESTINATION "${CMAKE_INSTALL_BINDIR}")

install(EXPORT reactor-cppConfig DESTINATION share/reactor-cpp/cmake)

export(TARGETS ${PROJECT_NAME} FILE reactor-cppConfig.cmake)
