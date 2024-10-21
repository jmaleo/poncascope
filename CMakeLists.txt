cmake_minimum_required(VERSION 3.24 FATAL_ERROR)
Project(Poncascope
        VERSION 1.0
        DESCRIPTION "Test and visualize point cloud differential quantities estimators."
        LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Source and build directories cannot be the same. Go use the /build directory.")
endif()

#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/recipes)

message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
include(config)

add_subdirectory(src)
