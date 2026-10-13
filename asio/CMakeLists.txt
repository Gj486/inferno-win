cmake_minimum_required(VERSION 3.16)
project(DanteBridgeASIO CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded")

add_library(dante_asio SHARED dante_asio.cpp)
target_link_libraries(dante_asio ole32 advapi32 winmm)
set_target_properties(dante_asio PROPERTIES
    PREFIX ""
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/stageB"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/stageB")

add_library(dante_asio_stageA SHARED stageA/dante_asio_stageA.cpp)
target_link_libraries(dante_asio_stageA ole32 advapi32 winmm)
set_target_properties(dante_asio_stageA PROPERTIES
    PREFIX ""
    OUTPUT_NAME "dante_asio"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/stageA"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/stageA")
