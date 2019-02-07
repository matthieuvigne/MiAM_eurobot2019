/*
 *      */


extern "C"
{
#include "acado_common.h"
}

#include "INCLUDE/QProblem.hpp"

#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1
#include "INCLUDE/EXTRAS/SolutionAnalysis.hpp"
#endif /* ACADO_COMPUTE_COVARIANCE_MATRIX */

static int acado_nWSR;



#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1
static SolutionAnalysis acado_sa;
#endif /* ACADO_COMPUTE_COVARIANCE_MATRIX */

int acado_solve( void )
{
	acado_nWSR = QPOASES_NWSRMAX;

	QProblem qp(42, 42);
	
	returnValue retVal = qp.init(acadoWorkspace.H, acadoWorkspace.g, acadoWorkspace.A, acadoWorkspace.lb, acadoWorkspace.ub, acadoWorkspace.lbA, acadoWorkspace.ubA, acado_nWSR, acadoWorkspace.y);

    qp.getPrimalSolution( acadoWorkspace.x );
    qp.getDualSolution( acadoWorkspace.y );
	
#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1

	if (retVal != SUCCESSFUL_RETURN)
		return (int)retVal;
		
	retVal = acado_sa.getHessianInverse( &qp,var );

#endif /* ACADO_COMPUTE_COVARIANCE_MATRIX */

	return (int)retVal;
}

int acado_getNWSR( void )
{
	return acado_nWSR;
}

const char* acado_getErrorString( int error )
{
	return MessageHandling::getErrorString( error );
}
