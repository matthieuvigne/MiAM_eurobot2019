/*
 *      */


#ifndef QPOASES_HEADER
#define QPOASES_HEADER

#ifdef PC_DEBUG
#include <stdio.h>
#endif /* PC_DEBUG */

#include <math.h>

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

/*
 * A set of options for qpOASES
 */

/** Maximum number of optimization variables. */
#define QPOASES_NVMAX      42
/** Maximum number of constraints. */
#define QPOASES_NCMAX      42
/** Maximum number of working set recalculations. */
#define QPOASES_NWSRMAX    252
/** Print level for qpOASES. */
#define QPOASES_PRINTLEVEL PL_NONE
/** The value of EPS */
#define QPOASES_EPS        2.221e-16
/** Internally used floating point type */
typedef double real_t;

/*
 * Forward function declarations
 */

/** A function that calls the QP solver */
EXTERNC int acado_solve( void );

/** Get the number of active set changes */
EXTERNC int acado_getNWSR( void );

/** Get the error string. */
const char* acado_getErrorString( int error );

#endif /* QPOASES_HEADER */
