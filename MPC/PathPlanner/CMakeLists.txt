#
# Project settings
#

# Minimum required version of cmake
CMAKE_MINIMUM_REQUIRED( VERSION 3.10 )

# Project name and programming languages used
PROJECT( PATH_PLANNER_MPC C CXX )


# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

# Ignore ignored-attributes (ACADO warnings) & deprecated-declarations (Eigen warnings)
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -finline-functions")
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-attributes -Wno-deprecated-declarations")

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

#
# Include directories
#
INCLUDE_DIRECTORIES( 
	${ACADO_INCLUDE_DIRS} 
	include 
	../../RobotCode/MiAMEurobot/include 
	./qpoases 
	./qpoases/INCLUDE 
	./qpoases/SRC
	./custom_solver/include
	./custom_solver/src
)

#
# Build QPOASES
#
FILE( GLOB QPOASES_HEADERS qpoases/INCLUDE/*.hpp qpoases/SRC/*.ipp )
FILE( GLOB QPOASES_SOURCES qpoases/SRC/*.cpp )
ADD_LIBRARY( qpOASES ${QPOASES_HEADERS} ${QPOASES_SOURCES} )

#
# Build custom solver
#
FILE( GLOB CUSTOM_SOLVER_HEADERS custom_solver/include/*.h custom_solver/include/*.hpp )
FILE( GLOB CUSTOM_SOLVER_SOURCES custom_solver/src/*.c custom_solver/src/*.cpp )
ADD_LIBRARY( customSolver ${CUSTOM_SOLVER_HEADERS} ${CUSTOM_SOLVER_SOURCES} )

#
# Set library up
#
FILE( GLOB LIB_SOURCES src/*.cpp ../../RobotCode/MiAMEurobot/src/trajectory/*.cpp )
FILE( GLOB LIB_HEADERS include/*.hpp )
ADD_LIBRARY( PathPlanner ${LIB_SOURCES} ${LIB_HEADERS} )

#
# Build executables
#
FILE( GLOB TEST_SOURCES tests/*.cpp tests/*.c )

FOREACH( TEST_SOURCE_FILE ${TEST_SOURCES} )

	GET_FILENAME_COMPONENT( TEST_NAME ${TEST_SOURCE_FILE} NAME_WE )
	ADD_EXECUTABLE( ${TEST_NAME} ${TEST_SOURCE_FILE} )
	TARGET_LINK_LIBRARIES( ${TEST_NAME} PathPlanner customSolver qpOASES ${ACADO_SHARED_LIBRARIES} )

ENDFOREACH( TEST_SOURCE_FILE ${TEST_SOURCES} )

