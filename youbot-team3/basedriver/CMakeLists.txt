cmake_minimum_required(VERSION 2.4)

set(src ${CMAKE_SOURCE_DIR}/YouBotOmniBaseDriver.cpp ${CMAKE_SOURCE_DIR}/test.cpp)
set(app YouBotBaseDriver)

add_executable(${app} ${src})
target_link_libraries(${app} memoryMapAndSemaphore)
