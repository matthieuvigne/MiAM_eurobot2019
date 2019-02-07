#
# Project settings
#

# Minimum required version of cmake
CMAKE_MINIMUM_REQUIRED( VERSION 3.10 )

# Project name and programming languages used
PROJECT( CODE_GENERATION_MPC CXX )


# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

# Ignore ignored-attributes (ACADO warnings) & deprecated-declarations (Eigen warnings)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-attributes -Wno-deprecated-declarations")

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

#
# Include directories
#
INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )


#
# Build executables
#
FILE( GLOB TEST_SOURCES src/*.cpp )

FOREACH( TEST_SOURCE_FILE ${TEST_SOURCES} )

	GET_FILENAME_COMPONENT( TEST_NAME ${TEST_SOURCE_FILE} NAME_WE )
	ADD_EXECUTABLE( ${TEST_NAME} ${TEST_SOURCE_FILE} )
	TARGET_LINK_LIBRARIES( ${TEST_NAME} ${ACADO_SHARED_LIBRARIES} )

ENDFOREACH( TEST_SOURCE_FILE ${TEST_SOURCES} )

