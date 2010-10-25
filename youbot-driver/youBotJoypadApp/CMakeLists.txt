cmake_minimum_required(VERSION 2.4)

set(src ${CMAKE_SOURCE_DIR}/youBotJoypadApp.cpp)
set(app ../youBotJoypadApp)

add_executable(${app} ${src})
target_link_libraries(${app} SDL memoryMapAndSemaphore)


