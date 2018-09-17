cmake_minimum_required(VERSION 3.0)

#Configure project
project(MiAMEurobot VERSION 1.1.0)
set(PROJECT_DESCRIPTION "C utility library to use a Beaglebone Black / Raspberry Pi for Eurobot - low-level drivers and some higher level utilities")

# Set compiler to arm compiler (cross-compile)
set(CMAKE_C_COMPILER "arm-linux-gnueabihf-gcc")

# Set C compiler flags.
set(CMAKE_C_FLAGS "-std=gnu99 -Wall -fdiagnostics-color -lpthread -lm")

# Set build-type specific flags
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	message("Build mode: debug.")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ggdb -DDEBUG")
else (CMAKE_BUILD_TYPE STREQUAL "Debug")
	message("Build mode: release build - optimization enabled.")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
endif (CMAKE_BUILD_TYPE STREQUAL "Debug")

# Specify library content :
# Recursively include all .c files
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.c")
# Add directory containing the headers, and put all headers in a variable.
include_directories(include)
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS "include/*.h")

# Setup static library
set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# Use pkg-config to link to external library.
find_package(PkgConfig REQUIRED)
# Link to glib (cross-compiled version).
pkg_search_module(ARMGLIB REQUIRED armglib-2.0>=2.47.0)
message(STATUS "Found armglib-2.0 v." ${ARMGLIB_VERSION})

include_directories(${ARMGLIB_INCLUDE_DIRS})
link_directories(${ARMGLIB_LIBRARY_DIRS})

target_link_libraries(${LIBRARY_NAME} ${ARMGLIB_LIBRARIES})

# Create package config file from template.
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/MiAMEurobotTemplate.pc" "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}.pc")

# Set install rules: copy library and headers.
install(TARGETS ${LIBRARY_NAME} DESTINATION "lib")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/" DESTINATION "include")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}.pc" DESTINATION "lib/pkgconfig/")
