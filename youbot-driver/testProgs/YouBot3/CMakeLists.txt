cmake_minimum_required(VERSION 2.4)

set(src ${CMAKE_SOURCE_DIR}/Test.cpp ${CMAKE_SOURCE_DIR}/Manipulator.cpp ${CMAKE_SOURCE_DIR}/YouBotArm.cpp)

target_link_libraries(${app} memoryMapAndSemaphore)

set(app ../YouBot3Test)

add_executable(${app} ${src})
target_link_libraries(${app} memoryMapAndSemaphore)