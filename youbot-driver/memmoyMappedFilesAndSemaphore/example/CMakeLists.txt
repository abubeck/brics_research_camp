#ein beispiel, wie ich den youBotDriver benutzen kann,
#wenn sich die SharedLibrary und die Header in dem Systemverzeichnis befinden.

project(memMapApplication)

cmake_minimum_required(VERSION 2.6)

add_executable(../bin/maps maps.cpp )
add_executable(../bin/mapt mapt.cpp )

target_link_libraries(../bin/maps memoryMapAndSemaphore)
target_link_libraries(../bin/mapt memoryMapAndSemaphore)
