project(Math-Custom C CXX)

FILE(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(${PROJECT_NAME} STATIC
  ${SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  pico_stdlib
  pico_float
  pico_double
  pico_divider
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)