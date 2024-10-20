cmake_minimum_required(VERSION 3.24 FATAL_ERROR)
Project(Poncascope
        VERSION 1.0
        DESCRIPTION "Test and visualize point cloud differential quantities estimators."
        LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/recipes)

message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
include(config)

add_subdirectory(src)
