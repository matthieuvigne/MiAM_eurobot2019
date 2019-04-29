cmake_minimum_required(VERSION 3.0)

#Configure project
project(HardwareTests)
set(PROJECT_DESCRIPTION "A set of hardware test (mostly made for wiring testing)")

# Set compiler based on option
option(CROSS_COMPILE "True to cross compile to arm, false to compile on current platform" ON)
if(CROSS_COMPILE)
	set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")
else()
	set(CMAKE_CXX_COMPILER "g++")
endif()
set(CMAKE_CXX_FLAGS_DEBUG "-DDEBUG")

# Set C compiler flags.
set(CMAKE_CXX_FLAGS "-Wall -fdiagnostics-color -lpthread")

# Use pkg-config to find external libraries : Glib (compiled for arm) and BBBEurobot.
find_package(PkgConfig REQUIRED)
# Look for external libraries, and link them to the project.
unset(MIAMLIB_FOUND CACHE)
if(CROSS_COMPILE)
	pkg_search_module(MIAMLIB REQUIRED MiAMEurobot>=2.0)
	pkg_check_modules(RPLIDAR REQUIRED rplidar)
else()
	pkg_search_module(MIAMLIB REQUIRED MiAMEurobot_intel>=2.0)
	pkg_check_modules(RPLIDAR REQUIRED rplidar_intel)
endif()
message(STATUS "Found MiAMEurobot v." ${MIAMLIB_VERSION})
message(STATUS "Found rplidar v." ${RPLIDAR_VERSION})

include_directories(${MIAMLIB_INCLUDE_DIRS} ${RPLIDAR_INCLUDE_DIRS})
link_directories(${MIAMLIB_LIBRARY_DIRS} ${RPLIDAR_INCLUDE_DIRS})

# Define a test target from target executable name and source files
FUNCTION(CREATE_TEST)
	add_executable(${ARGV0} ${ARGV1})
	target_link_libraries(${ARGV0} ${MIAMLIB_LIBRARIES} ${RPLIDAR_LIBRARIES})
	install(TARGETS ${ARGV0} DESTINATION ".")
ENDFUNCTION(CREATE_TEST)


# Compile each test.
CREATE_TEST("test_L6470" "src/L6470Test.cpp")
CREATE_TEST("test_USBLCD" "src/USBLCDTest.cpp")
CREATE_TEST("test_maestro" "src/MaestroTest.cpp")
CREATE_TEST("test_rplidar" "src/RPLidarTest.cpp")
CREATE_TEST("test_VL53L0X" "src/VL53L0XTest.cpp")
