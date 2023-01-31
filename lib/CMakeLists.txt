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
    multiport.cc
    )

if(REACTOR_CPP_TRACE)
  set(SOURCE_FILES ${SOURCE_FILES} trace.cc )
endif()

if (DEFINED LF_REACTOR_CPP_SUFFIX)
  set(REACTOR_CPP_INCLUDE "include/${LIB_TARGET}")
else()
  set(REACTOR_CPP_INCLUDE "include")
endif()

add_library(${LIB_TARGET} SHARED ${SOURCE_FILES})
target_include_directories(${LIB_TARGET} PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>"
  "$<BUILD_INTERFACE:${LTTNGUST_INCLUDE_DIRS}>"
  "$<INSTALL_INTERFACE:${REACTOR_CPP_INCLUDE}>"
  PRIVATE src)

if(MSVC)
  target_compile_options(${LIB_TARGET} PRIVATE /W4 /WX)
else()
  target_compile_options(${LIB_TARGET} PRIVATE -Wall -Wextra -pedantic -Werror)
endif()

target_link_libraries(${LIB_TARGET} ${CMAKE_THREAD_LIBS_INIT})
if(REACTOR_CPP_TRACE)
  target_link_libraries(${LIB_TARGET} LTTng::UST)
endif()

# Generates Compilation Database for clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set_target_properties(${LIB_TARGET} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1)

if(DEFINED LF_REACTOR_CPP_SUFFIX)
  install(FILES "${PROJECT_BINARY_DIR}/include/reactor-cpp/config.hh" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIB_TARGET}/reactor-cpp")
else()
  install(FILES "${PROJECT_BINARY_DIR}/include/reactor-cpp/config.hh" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/reactor-cpp")
endif()

install(TARGETS ${LIB_TARGET} EXPORT ${LIB_TARGET}Config
    ARCHIVE  DESTINATION "${CMAKE_INSTALL_LIBDIR}" OPTIONAL
    LIBRARY  DESTINATION "${CMAKE_INSTALL_LIBDIR}" OPTIONAL
    RUNTIME  DESTINATION "${CMAKE_INSTALL_BINDIR}" OPTIONAL)

install(EXPORT ${LIB_TARGET}Config DESTINATION share/${LIB_TARGET}/cmake)

export(TARGETS ${PROJECT_NAME} FILE ${LIB_TARGET}Config.cmake)
