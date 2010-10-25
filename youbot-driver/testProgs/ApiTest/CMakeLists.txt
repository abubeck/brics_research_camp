cmake_minimum_required(VERSION 2.4)

set(src ${CMAKE_SOURCE_DIR}/youBotApiTest.cpp)
set(app ../youBotApiTest)

add_executable(${app} ${src})
target_link_libraries(${app} memoryMapAndSemaphore)

set(src ${CMAKE_SOURCE_DIR}/armDriverYouBot3.cpp)
set(app ../youBotArmDriver)

add_executable(${app} ${src})
target_link_libraries(${app} memoryMapAndSemaphore)