include_directories(
  "${PROJECT_SOURCE_DIR}/include"
  )

add_custom_target(examples)
add_subdirectory(count)
add_subdirectory(ports)
add_subdirectory(hello)
add_subdirectory(power_train)
