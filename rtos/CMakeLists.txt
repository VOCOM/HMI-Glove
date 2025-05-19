project(FreeRTOS C)

FILE(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel/*.c)

add_library(${PROJECT_NAME} STATIC
  ${SRC_FILES}
  ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel/portable/GCC/ARM_CM0/port.c
  ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel/portable/MemMang/heap_4.c
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel/include
  ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel/portable/GCC/ARM_CM0
)