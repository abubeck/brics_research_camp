cmake_minimum_required(VERSION 2.4)

set(src ${CMAKE_SOURCE_DIR}/youBotPositionControl.cpp)
set(app youBotPositionControl)

add_executable(${app} ${src})
target_link_libraries(${app} memoryMapAndSemaphore)
