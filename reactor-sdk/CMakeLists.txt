set (ReactorSDK_LIB reactor-sdk)

include(GNUInstallDirs)

find_package(reactor-cpp PATHS )

set(REACTOR_CPP_SDK_INCLUDE "include")

add_library(${ReactorSDK_LIB} SHARED Environment.cc Reactor.cc)

target_link_libraries(${ReactorSDK_LIB} PUBLIC reactor-cpp)

install(TARGETS ${ReactorSDK_LIB} EXPORT ${ReactorSDK_LIB}Config
      ARCHIVE  DESTINATION "${CMAKE_INSTALL_LIBDIR}" OPTIONAL
      LIBRARY  DESTINATION "${CMAKE_INSTALL_LIBDIR}" OPTIONAL
      RUNTIME  DESTINATION "${CMAKE_INSTALL_BINDIR}" OPTIONAL)

install(EXPORT ${ReactorSDK_LIB}Config DESTINATION share/${ReactorSDK_LIB}/cmake)

export(TARGETS ${ReactorSDK_LIB} FILE ${ReactorSDK_LIB}Config.cmake)