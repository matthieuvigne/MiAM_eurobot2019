#
# Project settings
#

# Minimum required version of cmake
CMAKE_MINIMUM_REQUIRED( VERSION 3.10 )

# Project name and programming languages used
PROJECT( MPC_TESTS C CXX )


# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

# Ignore ignored-attributes (ACADO warnings) & deprecated-declarations (Eigen warnings)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-attributes -Wno-deprecated-declarations")

#
# Prerequisites
#


include_directories(
    ${MIAMLIB_INCLUDE_DIRS} 
    ../MainRobotCode/custom_solver 
    ../MainRobotCode/qpoases 
    ../MainRobotCode/qpoases/INCLUDE 
    ../MainRobotCode/qpoases/SRC 
    ../MainRobotCode/include
)

# QPoases
FILE( GLOB LIB_SOURCES ../MainRobotCode/qpoases/SRC/*.cpp )
FILE( GLOB LIB_HEADERS ../MainRobotCode/qpoases/INCLUDE/*.hpp )
ADD_LIBRARY( QPoases ${LIB_SOURCES} ${LIB_HEADERS} )



#
# Set library up
#
FILE( GLOB LIB_SOURCES 
    src/*.cpp 
    ../MiAMEurobot/src/trajectory/*.cpp 
    ../MiAMEurobot/src/io/*.cpp 
    ../MiAMEurobot/src/io/*.cpp 
    )
FILE( GLOB LIB_HEADERS include/*.hpp )
ADD_LIBRARY( Trajlib ${LIB_SOURCES} ${LIB_HEADERS} )



# CustomSolver
#~ FILE( GLOB LIB_SOURCES ../MainRobotCode/custom_solver/*.cpp ../MainRobotCode/custom_solver/*.c )
#~ FILE( GLOB LIB_HEADERS ../MainRobotCode/custom_solver/*.hpp ../MainRobotCode/custom_solver/*.h )
#~ ADD_LIBRARY( CustomSolver ${LIB_SOURCES} ${LIB_HEADERS} )
#~ TARGET_LINK_LIBRARIES( CustomSolver QPoases )


# MPC
FILE( GLOB LIB_SOURCES ../MainRobotCode/custom_solver/*.cpp ../MainRobotCode/custom_solver/*.c ../MainRobotCode/src/MPC.cpp )
FILE( GLOB LIB_HEADERS ../MainRobotCode/custom_solver/*.hpp ../MainRobotCode/custom_solver/*.h ../MainRobotCode/include/MPC.hpp )
ADD_LIBRARY( MPClib ${LIB_SOURCES} ${LIB_HEADERS} )
TARGET_LINK_LIBRARIES( MPClib QPoases Trajlib )


#
# Include directories
#
INCLUDE_DIRECTORIES( include .. )
FILE( GLOB_RECURSE ${PROJECT_NAME}_HEADERS include/*.hpp )

#
# Build executables
#
FILE( GLOB TEST_SOURCES tests/*.cpp )

FOREACH( TEST_SOURCE_FILE ${TEST_SOURCES} )

	GET_FILENAME_COMPONENT( TEST_NAME ${TEST_SOURCE_FILE} NAME_WE )
	ADD_EXECUTABLE( ${TEST_NAME} ${TEST_SOURCE_FILE} )
    #~ TARGET_LINK_LIBRARIES( ${TEST_NAME} Trajlib QPoases CustomSolver MPClib )
    TARGET_LINK_LIBRARIES( ${TEST_NAME} Trajlib QPoases MPClib )

ENDFOREACH( TEST_SOURCE_FILE ${TEST_SOURCES} )

