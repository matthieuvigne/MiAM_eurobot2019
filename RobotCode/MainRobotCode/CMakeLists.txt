cmake_minimum_required(VERSION 3.0)

#Configure project
project(mainRobotCode)
set(PROJECT_DESCRIPTION "Code for the main robot for Eurobot")

# Set compiler to arm compiler (cross-compile)
set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")
set(CMAKE_CXX_FLAGS_DEBUG "-DDEBUG")

# Set C compiler flags.
set(CMAKE_CXX_FLAGS "-Wall -Wno-psabi -fdiagnostics-color -lpthread")

# Use pkg-config to find external libraries : Glib (compiled for arm) and BBBEurobot.
find_package(PkgConfig REQUIRED)
# Look for external libraries, and link them to the project.
message(STATUS "Found armglib-2.0 v." ${ARMGLIB_VERSION})
unset(MIAMLIB_FOUND CACHE)
pkg_search_module(MIAMLIB REQUIRED MiAMEurobot>=2.0)
message(STATUS "Found MiAMEurobot v." ${MIAMLIB_VERSION})

include_directories(${MIAMLIB_INCLUDE_DIRS})
link_directories(${MIAMLIB_LIBRARY_DIRS})

# Include source files from src directory.
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp")
# Add directory containing the headers.
include_directories(include)

# Set executable target and link libraries.
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME} ${MIAMLIB_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION ".")

