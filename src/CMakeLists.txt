cmake_minimum_required(VERSION 3.0)
project(interpolation-methods)

set(CMAKE_CXX_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message("CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

option(INTERPOL_BUILD_APP_RUNTIME_EXPERIMENTS "Build Runtime Experiments" ON)
option(INTERPOL_BUILD_VIZ "Build Visualization" ON)

add_subdirectory(libinterpol)

if(INTERPOL_BUILD_APP_RUNTIME_EXPERIMENTS)
    add_subdirectory(app_runtime)
endif()

if(INTERPOL_BUILD_VIZ)
    add_subdirectory(libinterpolviz)
    add_subdirectory(app_visualization)
endif()