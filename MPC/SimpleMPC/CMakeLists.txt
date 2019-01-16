#
# Project settings
#

# Minimum required version of cmake
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project name and programming languages used
PROJECT( SIMPLE_MPC CXX )

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )


#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

#
# Include directories
#
INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )

#
# Build an executable
#
file( GLOB SOURCE_FILES src/*.cpp )
ADD_EXECUTABLE(        SimpleMPC ${SOURCE_FILES} )
# Linux and OS X
TARGET_LINK_LIBRARIES( SimpleMPC ${ACADO_SHARED_LIBRARIES} )
# Windows
#TARGET_LINK_LIBRARIES( <EXEC_NAME> ${ACADO_STATIC_LIBRARIES} )
SET_TARGET_PROPERTIES( SimpleMPC
                       PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build )

