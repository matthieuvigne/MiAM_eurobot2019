/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
acadoWorkspace.state[0] = acadoVariables.x[0];
acadoWorkspace.state[1] = acadoVariables.x[1];
acadoWorkspace.state[2] = acadoVariables.x[2];
acadoWorkspace.state[3] = acadoVariables.x[3];
acadoWorkspace.state[4] = acadoVariables.x[4];
acadoWorkspace.state[40] = acadoVariables.u[0];
acadoWorkspace.state[41] = acadoVariables.u[1];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{

acadoWorkspace.state[40] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[41] = acadoVariables.u[lRun1 * 2 + 1];

ret = acado_integrate(acadoWorkspace.state, lRun1 == 0);

acadoVariables.x[lRun1 * 5 + 5] = acadoWorkspace.state[0];
acadoVariables.x[lRun1 * 5 + 6] = acadoWorkspace.state[1];
acadoVariables.x[lRun1 * 5 + 7] = acadoWorkspace.state[2];
acadoVariables.x[lRun1 * 5 + 8] = acadoWorkspace.state[3];
acadoVariables.x[lRun1 * 5 + 9] = acadoWorkspace.state[4];

acadoWorkspace.evGx[lRun1 * 25] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 25 + 1] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 25 + 2] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 25 + 3] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 25 + 4] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 25 + 5] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 25 + 6] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 25 + 7] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 25 + 8] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 25 + 9] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 25 + 10] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 25 + 11] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 25 + 12] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 25 + 13] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 25 + 14] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 25 + 15] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 25 + 16] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 25 + 17] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 25 + 18] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 25 + 19] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 25 + 20] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 25 + 21] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 25 + 22] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 25 + 23] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 25 + 24] = acadoWorkspace.state[29];

acadoWorkspace.evGu[lRun1 * 10] = acadoWorkspace.state[30];
acadoWorkspace.evGu[lRun1 * 10 + 1] = acadoWorkspace.state[31];
acadoWorkspace.evGu[lRun1 * 10 + 2] = acadoWorkspace.state[32];
acadoWorkspace.evGu[lRun1 * 10 + 3] = acadoWorkspace.state[33];
acadoWorkspace.evGu[lRun1 * 10 + 4] = acadoWorkspace.state[34];
acadoWorkspace.evGu[lRun1 * 10 + 5] = acadoWorkspace.state[35];
acadoWorkspace.evGu[lRun1 * 10 + 6] = acadoWorkspace.state[36];
acadoWorkspace.evGu[lRun1 * 10 + 7] = acadoWorkspace.state[37];
acadoWorkspace.evGu[lRun1 * 10 + 8] = acadoWorkspace.state[38];
acadoWorkspace.evGu[lRun1 * 10 + 9] = acadoWorkspace.state[39];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 5];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 5 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 5 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 5 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 5 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 5] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 5 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 5 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 5 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 5 + 4] = acadoWorkspace.objValueOut[4];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[50];
acadoWorkspace.objValueIn[1] = acadoVariables.x[51];
acadoWorkspace.objValueIn[2] = acadoVariables.x[52];
acadoWorkspace.objValueIn[3] = acadoVariables.x[53];
acadoWorkspace.objValueIn[4] = acadoVariables.x[54];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4];
dNew[1] += + Gx1[5]*dOld[0] + Gx1[6]*dOld[1] + Gx1[7]*dOld[2] + Gx1[8]*dOld[3] + Gx1[9]*dOld[4];
dNew[2] += + Gx1[10]*dOld[0] + Gx1[11]*dOld[1] + Gx1[12]*dOld[2] + Gx1[13]*dOld[3] + Gx1[14]*dOld[4];
dNew[3] += + Gx1[15]*dOld[0] + Gx1[16]*dOld[1] + Gx1[17]*dOld[2] + Gx1[18]*dOld[3] + Gx1[19]*dOld[4];
dNew[4] += + Gx1[20]*dOld[0] + Gx1[21]*dOld[1] + Gx1[22]*dOld[2] + Gx1[23]*dOld[3] + Gx1[24]*dOld[4];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[15] + Gx1[4]*Gx2[20];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[16] + Gx1[4]*Gx2[21];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[12] + Gx1[3]*Gx2[17] + Gx1[4]*Gx2[22];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[13] + Gx1[3]*Gx2[18] + Gx1[4]*Gx2[23];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[14] + Gx1[3]*Gx2[19] + Gx1[4]*Gx2[24];
Gx3[5] = + Gx1[5]*Gx2[0] + Gx1[6]*Gx2[5] + Gx1[7]*Gx2[10] + Gx1[8]*Gx2[15] + Gx1[9]*Gx2[20];
Gx3[6] = + Gx1[5]*Gx2[1] + Gx1[6]*Gx2[6] + Gx1[7]*Gx2[11] + Gx1[8]*Gx2[16] + Gx1[9]*Gx2[21];
Gx3[7] = + Gx1[5]*Gx2[2] + Gx1[6]*Gx2[7] + Gx1[7]*Gx2[12] + Gx1[8]*Gx2[17] + Gx1[9]*Gx2[22];
Gx3[8] = + Gx1[5]*Gx2[3] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[13] + Gx1[8]*Gx2[18] + Gx1[9]*Gx2[23];
Gx3[9] = + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[14] + Gx1[8]*Gx2[19] + Gx1[9]*Gx2[24];
Gx3[10] = + Gx1[10]*Gx2[0] + Gx1[11]*Gx2[5] + Gx1[12]*Gx2[10] + Gx1[13]*Gx2[15] + Gx1[14]*Gx2[20];
Gx3[11] = + Gx1[10]*Gx2[1] + Gx1[11]*Gx2[6] + Gx1[12]*Gx2[11] + Gx1[13]*Gx2[16] + Gx1[14]*Gx2[21];
Gx3[12] = + Gx1[10]*Gx2[2] + Gx1[11]*Gx2[7] + Gx1[12]*Gx2[12] + Gx1[13]*Gx2[17] + Gx1[14]*Gx2[22];
Gx3[13] = + Gx1[10]*Gx2[3] + Gx1[11]*Gx2[8] + Gx1[12]*Gx2[13] + Gx1[13]*Gx2[18] + Gx1[14]*Gx2[23];
Gx3[14] = + Gx1[10]*Gx2[4] + Gx1[11]*Gx2[9] + Gx1[12]*Gx2[14] + Gx1[13]*Gx2[19] + Gx1[14]*Gx2[24];
Gx3[15] = + Gx1[15]*Gx2[0] + Gx1[16]*Gx2[5] + Gx1[17]*Gx2[10] + Gx1[18]*Gx2[15] + Gx1[19]*Gx2[20];
Gx3[16] = + Gx1[15]*Gx2[1] + Gx1[16]*Gx2[6] + Gx1[17]*Gx2[11] + Gx1[18]*Gx2[16] + Gx1[19]*Gx2[21];
Gx3[17] = + Gx1[15]*Gx2[2] + Gx1[16]*Gx2[7] + Gx1[17]*Gx2[12] + Gx1[18]*Gx2[17] + Gx1[19]*Gx2[22];
Gx3[18] = + Gx1[15]*Gx2[3] + Gx1[16]*Gx2[8] + Gx1[17]*Gx2[13] + Gx1[18]*Gx2[18] + Gx1[19]*Gx2[23];
Gx3[19] = + Gx1[15]*Gx2[4] + Gx1[16]*Gx2[9] + Gx1[17]*Gx2[14] + Gx1[18]*Gx2[19] + Gx1[19]*Gx2[24];
Gx3[20] = + Gx1[20]*Gx2[0] + Gx1[21]*Gx2[5] + Gx1[22]*Gx2[10] + Gx1[23]*Gx2[15] + Gx1[24]*Gx2[20];
Gx3[21] = + Gx1[20]*Gx2[1] + Gx1[21]*Gx2[6] + Gx1[22]*Gx2[11] + Gx1[23]*Gx2[16] + Gx1[24]*Gx2[21];
Gx3[22] = + Gx1[20]*Gx2[2] + Gx1[21]*Gx2[7] + Gx1[22]*Gx2[12] + Gx1[23]*Gx2[17] + Gx1[24]*Gx2[22];
Gx3[23] = + Gx1[20]*Gx2[3] + Gx1[21]*Gx2[8] + Gx1[22]*Gx2[13] + Gx1[23]*Gx2[18] + Gx1[24]*Gx2[23];
Gx3[24] = + Gx1[20]*Gx2[4] + Gx1[21]*Gx2[9] + Gx1[22]*Gx2[14] + Gx1[23]*Gx2[19] + Gx1[24]*Gx2[24];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6] + Gx1[4]*Gu1[8];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7] + Gx1[4]*Gu1[9];
Gu2[2] = + Gx1[5]*Gu1[0] + Gx1[6]*Gu1[2] + Gx1[7]*Gu1[4] + Gx1[8]*Gu1[6] + Gx1[9]*Gu1[8];
Gu2[3] = + Gx1[5]*Gu1[1] + Gx1[6]*Gu1[3] + Gx1[7]*Gu1[5] + Gx1[8]*Gu1[7] + Gx1[9]*Gu1[9];
Gu2[4] = + Gx1[10]*Gu1[0] + Gx1[11]*Gu1[2] + Gx1[12]*Gu1[4] + Gx1[13]*Gu1[6] + Gx1[14]*Gu1[8];
Gu2[5] = + Gx1[10]*Gu1[1] + Gx1[11]*Gu1[3] + Gx1[12]*Gu1[5] + Gx1[13]*Gu1[7] + Gx1[14]*Gu1[9];
Gu2[6] = + Gx1[15]*Gu1[0] + Gx1[16]*Gu1[2] + Gx1[17]*Gu1[4] + Gx1[18]*Gu1[6] + Gx1[19]*Gu1[8];
Gu2[7] = + Gx1[15]*Gu1[1] + Gx1[16]*Gu1[3] + Gx1[17]*Gu1[5] + Gx1[18]*Gu1[7] + Gx1[19]*Gu1[9];
Gu2[8] = + Gx1[20]*Gu1[0] + Gx1[21]*Gu1[2] + Gx1[22]*Gu1[4] + Gx1[23]*Gu1[6] + Gx1[24]*Gu1[8];
Gu2[9] = + Gx1[20]*Gu1[1] + Gx1[21]*Gu1[3] + Gx1[22]*Gu1[5] + Gx1[23]*Gu1[7] + Gx1[24]*Gu1[9];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = 0.0;
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = 0.0;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = 0.0;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = + (real_t)1.0000000000000000e+03*dOld[0];
dNew[1] = + (real_t)1.0000000000000000e+03*dOld[1];
dNew[2] = + (real_t)1.0000000000000000e+01*dOld[2];
dNew[3] = +dOld[3];
dNew[4] = +dOld[4];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = 0.0;
;
RDy1[1] = 0.0;
;
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + (real_t)1.0000000000000000e+03*Dy1[0];
QDy1[1] = + (real_t)1.0000000000000000e+03*Dy1[1];
QDy1[2] = + (real_t)1.0000000000000000e+01*Dy1[2];
QDy1[3] = +Dy1[3];
QDy1[4] = +Dy1[4];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2] + E1[6]*QDy1[3] + E1[8]*QDy1[4];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2] + E1[7]*QDy1[3] + E1[9]*QDy1[4];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[5] + E1[4]*Gx1[10] + E1[6]*Gx1[15] + E1[8]*Gx1[20];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[6] + E1[4]*Gx1[11] + E1[6]*Gx1[16] + E1[8]*Gx1[21];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[7] + E1[4]*Gx1[12] + E1[6]*Gx1[17] + E1[8]*Gx1[22];
H101[3] += + E1[0]*Gx1[3] + E1[2]*Gx1[8] + E1[4]*Gx1[13] + E1[6]*Gx1[18] + E1[8]*Gx1[23];
H101[4] += + E1[0]*Gx1[4] + E1[2]*Gx1[9] + E1[4]*Gx1[14] + E1[6]*Gx1[19] + E1[8]*Gx1[24];
H101[5] += + E1[1]*Gx1[0] + E1[3]*Gx1[5] + E1[5]*Gx1[10] + E1[7]*Gx1[15] + E1[9]*Gx1[20];
H101[6] += + E1[1]*Gx1[1] + E1[3]*Gx1[6] + E1[5]*Gx1[11] + E1[7]*Gx1[16] + E1[9]*Gx1[21];
H101[7] += + E1[1]*Gx1[2] + E1[3]*Gx1[7] + E1[5]*Gx1[12] + E1[7]*Gx1[17] + E1[9]*Gx1[22];
H101[8] += + E1[1]*Gx1[3] + E1[3]*Gx1[8] + E1[5]*Gx1[13] + E1[7]*Gx1[18] + E1[9]*Gx1[23];
H101[9] += + E1[1]*Gx1[4] + E1[3]*Gx1[9] + E1[5]*Gx1[14] + E1[7]*Gx1[19] + E1[9]*Gx1[24];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 10; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
dNew[3] += + E1[6]*U1[0] + E1[7]*U1[1];
dNew[4] += + E1[8]*U1[0] + E1[9]*U1[1];
}

void acado_multQ1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)1.0000000000000000e+03*Gx1[0];
Gx2[1] = + (real_t)1.0000000000000000e+03*Gx1[1];
Gx2[2] = + (real_t)1.0000000000000000e+03*Gx1[2];
Gx2[3] = + (real_t)1.0000000000000000e+03*Gx1[3];
Gx2[4] = + (real_t)1.0000000000000000e+03*Gx1[4];
Gx2[5] = + (real_t)1.0000000000000000e+03*Gx1[5];
Gx2[6] = + (real_t)1.0000000000000000e+03*Gx1[6];
Gx2[7] = + (real_t)1.0000000000000000e+03*Gx1[7];
Gx2[8] = + (real_t)1.0000000000000000e+03*Gx1[8];
Gx2[9] = + (real_t)1.0000000000000000e+03*Gx1[9];
Gx2[10] = + (real_t)1.0000000000000000e+01*Gx1[10];
Gx2[11] = + (real_t)1.0000000000000000e+01*Gx1[11];
Gx2[12] = + (real_t)1.0000000000000000e+01*Gx1[12];
Gx2[13] = + (real_t)1.0000000000000000e+01*Gx1[13];
Gx2[14] = + (real_t)1.0000000000000000e+01*Gx1[14];
Gx2[15] = +Gx1[15];
Gx2[16] = +Gx1[16];
Gx2[17] = +Gx1[17];
Gx2[18] = +Gx1[18];
Gx2[19] = +Gx1[19];
Gx2[20] = +Gx1[20];
Gx2[21] = +Gx1[21];
Gx2[22] = +Gx1[22];
Gx2[23] = +Gx1[23];
Gx2[24] = +Gx1[24];
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)5.0000000000000000e+03*Gx1[0];
Gx2[1] = + (real_t)5.0000000000000000e+03*Gx1[1];
Gx2[2] = + (real_t)5.0000000000000000e+03*Gx1[2];
Gx2[3] = + (real_t)5.0000000000000000e+03*Gx1[3];
Gx2[4] = + (real_t)5.0000000000000000e+03*Gx1[4];
Gx2[5] = + (real_t)5.0000000000000000e+03*Gx1[5];
Gx2[6] = + (real_t)5.0000000000000000e+03*Gx1[6];
Gx2[7] = + (real_t)5.0000000000000000e+03*Gx1[7];
Gx2[8] = + (real_t)5.0000000000000000e+03*Gx1[8];
Gx2[9] = + (real_t)5.0000000000000000e+03*Gx1[9];
Gx2[10] = + (real_t)5.0000000000000000e+01*Gx1[10];
Gx2[11] = + (real_t)5.0000000000000000e+01*Gx1[11];
Gx2[12] = + (real_t)5.0000000000000000e+01*Gx1[12];
Gx2[13] = + (real_t)5.0000000000000000e+01*Gx1[13];
Gx2[14] = + (real_t)5.0000000000000000e+01*Gx1[14];
Gx2[15] = 0.0;
;
Gx2[16] = 0.0;
;
Gx2[17] = 0.0;
;
Gx2[18] = 0.0;
;
Gx2[19] = 0.0;
;
Gx2[20] = 0.0;
;
Gx2[21] = 0.0;
;
Gx2[22] = 0.0;
;
Gx2[23] = 0.0;
;
Gx2[24] = 0.0;
;
}

void acado_multQ1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)1.0000000000000000e+03*Gu1[0];
Gu2[1] = + (real_t)1.0000000000000000e+03*Gu1[1];
Gu2[2] = + (real_t)1.0000000000000000e+03*Gu1[2];
Gu2[3] = + (real_t)1.0000000000000000e+03*Gu1[3];
Gu2[4] = + (real_t)1.0000000000000000e+01*Gu1[4];
Gu2[5] = + (real_t)1.0000000000000000e+01*Gu1[5];
Gu2[6] = +Gu1[6];
Gu2[7] = +Gu1[7];
Gu2[8] = +Gu1[8];
Gu2[9] = +Gu1[9];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)5.0000000000000000e+03*Gu1[0];
Gu2[1] = + (real_t)5.0000000000000000e+03*Gu1[1];
Gu2[2] = + (real_t)5.0000000000000000e+03*Gu1[2];
Gu2[3] = + (real_t)5.0000000000000000e+03*Gu1[3];
Gu2[4] = + (real_t)5.0000000000000000e+01*Gu1[4];
Gu2[5] = + (real_t)5.0000000000000000e+01*Gu1[5];
Gu2[6] = 0.0;
;
Gu2[7] = 0.0;
;
Gu2[8] = 0.0;
;
Gu2[9] = 0.0;
;
}

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[5] + Hx[2]*Gx[10] + Hx[3]*Gx[15] + Hx[4]*Gx[20];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[6] + Hx[2]*Gx[11] + Hx[3]*Gx[16] + Hx[4]*Gx[21];
A01[2] = + Hx[0]*Gx[2] + Hx[1]*Gx[7] + Hx[2]*Gx[12] + Hx[3]*Gx[17] + Hx[4]*Gx[22];
A01[3] = + Hx[0]*Gx[3] + Hx[1]*Gx[8] + Hx[2]*Gx[13] + Hx[3]*Gx[18] + Hx[4]*Gx[23];
A01[4] = + Hx[0]*Gx[4] + Hx[1]*Gx[9] + Hx[2]*Gx[14] + Hx[3]*Gx[19] + Hx[4]*Gx[24];
A01[5] = + Hx[5]*Gx[0] + Hx[6]*Gx[5] + Hx[7]*Gx[10] + Hx[8]*Gx[15] + Hx[9]*Gx[20];
A01[6] = + Hx[5]*Gx[1] + Hx[6]*Gx[6] + Hx[7]*Gx[11] + Hx[8]*Gx[16] + Hx[9]*Gx[21];
A01[7] = + Hx[5]*Gx[2] + Hx[6]*Gx[7] + Hx[7]*Gx[12] + Hx[8]*Gx[17] + Hx[9]*Gx[22];
A01[8] = + Hx[5]*Gx[3] + Hx[6]*Gx[8] + Hx[7]*Gx[13] + Hx[8]*Gx[18] + Hx[9]*Gx[23];
A01[9] = + Hx[5]*Gx[4] + Hx[6]*Gx[9] + Hx[7]*Gx[14] + Hx[8]*Gx[19] + Hx[9]*Gx[24];
A01[10] = + Hx[10]*Gx[0] + Hx[11]*Gx[5] + Hx[12]*Gx[10] + Hx[13]*Gx[15] + Hx[14]*Gx[20];
A01[11] = + Hx[10]*Gx[1] + Hx[11]*Gx[6] + Hx[12]*Gx[11] + Hx[13]*Gx[16] + Hx[14]*Gx[21];
A01[12] = + Hx[10]*Gx[2] + Hx[11]*Gx[7] + Hx[12]*Gx[12] + Hx[13]*Gx[17] + Hx[14]*Gx[22];
A01[13] = + Hx[10]*Gx[3] + Hx[11]*Gx[8] + Hx[12]*Gx[13] + Hx[13]*Gx[18] + Hx[14]*Gx[23];
A01[14] = + Hx[10]*Gx[4] + Hx[11]*Gx[9] + Hx[12]*Gx[14] + Hx[13]*Gx[19] + Hx[14]*Gx[24];
A01[15] = + Hx[15]*Gx[0] + Hx[16]*Gx[5] + Hx[17]*Gx[10] + Hx[18]*Gx[15] + Hx[19]*Gx[20];
A01[16] = + Hx[15]*Gx[1] + Hx[16]*Gx[6] + Hx[17]*Gx[11] + Hx[18]*Gx[16] + Hx[19]*Gx[21];
A01[17] = + Hx[15]*Gx[2] + Hx[16]*Gx[7] + Hx[17]*Gx[12] + Hx[18]*Gx[17] + Hx[19]*Gx[22];
A01[18] = + Hx[15]*Gx[3] + Hx[16]*Gx[8] + Hx[17]*Gx[13] + Hx[18]*Gx[18] + Hx[19]*Gx[23];
A01[19] = + Hx[15]*Gx[4] + Hx[16]*Gx[9] + Hx[17]*Gx[14] + Hx[18]*Gx[19] + Hx[19]*Gx[24];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 80) + (col * 2)] = + Hx[0]*E[0] + Hx[1]*E[2] + Hx[2]*E[4] + Hx[3]*E[6] + Hx[4]*E[8];
acadoWorkspace.A[(row * 80) + (col * 2 + 1)] = + Hx[0]*E[1] + Hx[1]*E[3] + Hx[2]*E[5] + Hx[3]*E[7] + Hx[4]*E[9];
acadoWorkspace.A[(row * 80 + 20) + (col * 2)] = + Hx[5]*E[0] + Hx[6]*E[2] + Hx[7]*E[4] + Hx[8]*E[6] + Hx[9]*E[8];
acadoWorkspace.A[(row * 80 + 20) + (col * 2 + 1)] = + Hx[5]*E[1] + Hx[6]*E[3] + Hx[7]*E[5] + Hx[8]*E[7] + Hx[9]*E[9];
acadoWorkspace.A[(row * 80 + 40) + (col * 2)] = + Hx[10]*E[0] + Hx[11]*E[2] + Hx[12]*E[4] + Hx[13]*E[6] + Hx[14]*E[8];
acadoWorkspace.A[(row * 80 + 40) + (col * 2 + 1)] = + Hx[10]*E[1] + Hx[11]*E[3] + Hx[12]*E[5] + Hx[13]*E[7] + Hx[14]*E[9];
acadoWorkspace.A[(row * 80 + 60) + (col * 2)] = + Hx[15]*E[0] + Hx[16]*E[2] + Hx[17]*E[4] + Hx[18]*E[6] + Hx[19]*E[8];
acadoWorkspace.A[(row * 80 + 60) + (col * 2 + 1)] = + Hx[15]*E[1] + Hx[16]*E[3] + Hx[17]*E[5] + Hx[18]*E[7] + Hx[19]*E[9];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 5;
/* Vector of auxiliary variables; number of elements: 32. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (real_t)(0.0000000000000000e+00);
a[1] = (real_t)(0.0000000000000000e+00);
a[2] = (real_t)(0.0000000000000000e+00);
a[3] = (real_t)(1.0000000000000000e+00);
a[4] = (real_t)(1.0050000000000001e-01);
a[5] = (real_t)(0.0000000000000000e+00);
a[6] = (real_t)(0.0000000000000000e+00);
a[7] = (real_t)(0.0000000000000000e+00);
a[8] = (real_t)(1.0000000000000000e+00);
a[9] = (real_t)(1.0050000000000001e-01);
a[10] = (real_t)(-1.0000000000000000e+00);
a[11] = (a[9]*a[10]);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = (real_t)(0.0000000000000000e+00);
a[14] = (real_t)(0.0000000000000000e+00);
a[15] = (real_t)(0.0000000000000000e+00);
a[16] = (real_t)(0.0000000000000000e+00);
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (real_t)(0.0000000000000000e+00);
a[24] = (real_t)(0.0000000000000000e+00);
a[25] = (real_t)(0.0000000000000000e+00);
a[26] = (real_t)(1.0000000000000000e+00);
a[27] = (real_t)(1.0050000000000001e-01);
a[28] = (real_t)(1.0000000000000000e+00);
a[29] = (real_t)(1.0050000000000001e-01);
a[30] = (real_t)(-1.0000000000000000e+00);
a[31] = (a[29]*a[30]);

/* Compute outputs: */
out[0] = (xd[3]+((real_t)(1.0050000000000001e-01)*xd[4]));
out[1] = (xd[3]-((real_t)(1.0050000000000001e-01)*xd[4]));
out[2] = (u[0]+((real_t)(1.0050000000000001e-01)*u[1]));
out[3] = (u[0]-((real_t)(1.0050000000000001e-01)*u[1]));
out[4] = a[0];
out[5] = a[1];
out[6] = a[2];
out[7] = a[3];
out[8] = a[4];
out[9] = a[5];
out[10] = a[6];
out[11] = a[7];
out[12] = a[8];
out[13] = a[11];
out[14] = a[12];
out[15] = a[13];
out[16] = a[14];
out[17] = a[15];
out[18] = a[16];
out[19] = a[17];
out[20] = a[18];
out[21] = a[19];
out[22] = a[20];
out[23] = a[21];
out[24] = a[22];
out[25] = a[23];
out[26] = a[24];
out[27] = a[25];
out[28] = a[26];
out[29] = a[27];
out[30] = a[28];
out[31] = a[31];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 25 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 25 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 10 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 10 ]), &(acadoWorkspace.E[ 20 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 50 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.evGx[ 50 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.E[ 30 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.E[ 40 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 20 ]), &(acadoWorkspace.E[ 50 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 75 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.evGx[ 75 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.E[ 60 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.E[ 70 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.E[ 80 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.E[ 90 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.evGx[ 100 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.E[ 100 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.E[ 110 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.E[ 120 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.E[ 130 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 40 ]), &(acadoWorkspace.E[ 140 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 125 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.evGx[ 125 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.E[ 150 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.E[ 160 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.E[ 170 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 130 ]), &(acadoWorkspace.E[ 180 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.E[ 190 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 50 ]), &(acadoWorkspace.E[ 200 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 150 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.evGx[ 150 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.E[ 210 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.E[ 220 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.E[ 230 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.E[ 240 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 190 ]), &(acadoWorkspace.E[ 250 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.E[ 260 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.E[ 270 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 175 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.evGx[ 175 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.E[ 280 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.E[ 290 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.E[ 300 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 310 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 250 ]), &(acadoWorkspace.E[ 320 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 260 ]), &(acadoWorkspace.E[ 330 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.E[ 340 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 70 ]), &(acadoWorkspace.E[ 350 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.evGx[ 200 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.E[ 360 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.E[ 370 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.E[ 380 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.E[ 390 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.E[ 400 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.E[ 410 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 340 ]), &(acadoWorkspace.E[ 420 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 350 ]), &(acadoWorkspace.E[ 430 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.E[ 440 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.evGx[ 225 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.E[ 450 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.E[ 460 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.E[ 470 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.E[ 480 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.E[ 490 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.E[ 500 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.E[ 510 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 430 ]), &(acadoWorkspace.E[ 520 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.E[ 530 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 90 ]), &(acadoWorkspace.E[ 540 ]) );

acado_multQ1Gu( acadoWorkspace.E, acadoWorkspace.QE );
acado_multQ1Gu( &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.QE[ 10 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QE[ 20 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.QE[ 50 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.QE[ 70 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 100 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.QE[ 110 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 130 ]), &(acadoWorkspace.QE[ 130 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.QE[ 170 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 190 ]), &(acadoWorkspace.QE[ 190 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QE[ 220 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.QE[ 230 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 250 ]), &(acadoWorkspace.QE[ 250 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 260 ]), &(acadoWorkspace.QE[ 260 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 280 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 290 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.QE[ 310 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 340 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 350 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 370 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 380 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.QE[ 410 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 430 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 460 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 470 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QE[ 490 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.QE[ 500 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.QE[ 520 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 530 ]), &(acadoWorkspace.QE[ 530 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 10 ]), &(acadoWorkspace.evGx[ 25 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 30 ]), &(acadoWorkspace.evGx[ 50 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.evGx[ 75 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 100 ]), &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 150 ]), &(acadoWorkspace.evGx[ 125 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 210 ]), &(acadoWorkspace.evGx[ 150 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 280 ]), &(acadoWorkspace.evGx[ 175 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 360 ]), &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 450 ]), &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 20 ]), &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 40 ]), &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 70 ]), &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 110 ]), &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 160 ]), &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 220 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 290 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 370 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 460 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 50 ]), &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 80 ]), &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 170 ]), &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 230 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 380 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 470 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 90 ]), &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 130 ]), &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 180 ]), &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 310 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 390 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 140 ]), &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 190 ]), &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 250 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 320 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 400 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 490 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 40 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 50 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 200 ]), &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.H10[ 50 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 260 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.H10[ 50 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 330 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.H10[ 50 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 410 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 50 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 500 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 50 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 270 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 340 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 420 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 510 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 70 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 350 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.H10[ 70 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 430 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 70 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 520 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 70 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 440 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 530 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 90 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 540 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.H10[ 90 ]) );

acado_setBlockH11_R1( 0, 0 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.QE[ 10 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 100 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 280 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 450 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.QE[ 20 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 70 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 110 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 220 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 290 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 370 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 460 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 50 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 170 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 230 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 380 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 470 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 130 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 310 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 480 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 190 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 250 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 490 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 260 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 410 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 500 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 510 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 1, 1 );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QE[ 20 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.QE[ 70 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.QE[ 110 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QE[ 220 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 290 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 370 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 460 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 50 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 170 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QE[ 230 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 380 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 470 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.QE[ 130 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 310 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 480 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 190 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QE[ 250 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 490 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QE[ 260 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 410 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 500 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 510 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 2, 2 );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.QE[ 50 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.QE[ 170 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.QE[ 230 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 380 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 470 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 130 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 310 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 480 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.QE[ 190 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.QE[ 250 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 490 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.QE[ 260 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 410 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 500 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 510 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 3, 3 );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 130 ]), &(acadoWorkspace.QE[ 130 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.QE[ 310 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 130 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 190 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 250 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 490 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 260 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 410 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 500 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 510 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 4, 4 );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 190 ]), &(acadoWorkspace.QE[ 190 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 250 ]), &(acadoWorkspace.QE[ 250 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QE[ 490 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 190 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 250 ]), &(acadoWorkspace.QE[ 260 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 410 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QE[ 500 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 250 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QE[ 510 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 5, 5 );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 260 ]), &(acadoWorkspace.QE[ 260 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.QE[ 410 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.QE[ 500 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 260 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.QE[ 510 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 6, 6 );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 340 ]), &(acadoWorkspace.QE[ 340 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 510 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 340 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 7, 7 );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 350 ]), &(acadoWorkspace.QE[ 350 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 430 ]), &(acadoWorkspace.QE[ 430 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.QE[ 520 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 430 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 8, 8 );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QE[ 440 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 530 ]), &(acadoWorkspace.QE[ 530 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 530 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_setBlockH11_R1( 9, 9 );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );


acado_copyHTH( 1, 0 );
acado_copyHTH( 2, 0 );
acado_copyHTH( 2, 1 );
acado_copyHTH( 3, 0 );
acado_copyHTH( 3, 1 );
acado_copyHTH( 3, 2 );
acado_copyHTH( 4, 0 );
acado_copyHTH( 4, 1 );
acado_copyHTH( 4, 2 );
acado_copyHTH( 4, 3 );
acado_copyHTH( 5, 0 );
acado_copyHTH( 5, 1 );
acado_copyHTH( 5, 2 );
acado_copyHTH( 5, 3 );
acado_copyHTH( 5, 4 );
acado_copyHTH( 6, 0 );
acado_copyHTH( 6, 1 );
acado_copyHTH( 6, 2 );
acado_copyHTH( 6, 3 );
acado_copyHTH( 6, 4 );
acado_copyHTH( 6, 5 );
acado_copyHTH( 7, 0 );
acado_copyHTH( 7, 1 );
acado_copyHTH( 7, 2 );
acado_copyHTH( 7, 3 );
acado_copyHTH( 7, 4 );
acado_copyHTH( 7, 5 );
acado_copyHTH( 7, 6 );
acado_copyHTH( 8, 0 );
acado_copyHTH( 8, 1 );
acado_copyHTH( 8, 2 );
acado_copyHTH( 8, 3 );
acado_copyHTH( 8, 4 );
acado_copyHTH( 8, 5 );
acado_copyHTH( 8, 6 );
acado_copyHTH( 8, 7 );
acado_copyHTH( 9, 0 );
acado_copyHTH( 9, 1 );
acado_copyHTH( 9, 2 );
acado_copyHTH( 9, 3 );
acado_copyHTH( 9, 4 );
acado_copyHTH( 9, 5 );
acado_copyHTH( 9, 6 );
acado_copyHTH( 9, 7 );
acado_copyHTH( 9, 8 );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 10 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 30 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 60 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 100 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 150 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 210 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 280 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 360 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 450 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 20 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 40 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 70 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 110 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 160 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 220 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 290 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 370 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 460 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 50 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 80 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 170 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 230 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 380 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 470 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 90 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 130 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 180 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 310 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 390 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 140 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 190 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 250 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 320 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 400 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 490 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 200 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 260 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 330 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 410 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 500 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 270 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 340 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 420 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 510 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 350 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 430 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 520 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 440 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 530 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 540 ]), &(acadoWorkspace.g[ 18 ]) );
acadoWorkspace.lb[0] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[19];
acadoWorkspace.ub[0] = (real_t)1.0000000000000000e+12 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)1.0000000000000000e+12 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)1.0000000000000000e+12 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)1.0000000000000000e+12 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e+12 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e+12 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)1.0000000000000000e+12 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)1.0000000000000000e+12 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+12 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+12 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)1.0000000000000000e+12 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)1.0000000000000000e+12 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)1.0000000000000000e+12 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)1.0000000000000000e+12 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e+12 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)1.0000000000000000e+12 - acadoVariables.u[19];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.x[lRun1 * 5 + 4];
acadoWorkspace.conValueIn[5] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.conValueIn[6] = acadoVariables.u[lRun1 * 2 + 1];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1 * 4] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun1 * 4 + 1] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evH[lRun1 * 4 + 2] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evH[lRun1 * 4 + 3] = acadoWorkspace.conValueOut[3];

acadoWorkspace.evHx[lRun1 * 20] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 20 + 1] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 20 + 2] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 20 + 3] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun1 * 20 + 4] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 20 + 5] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 20 + 6] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 20 + 7] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHx[lRun1 * 20 + 8] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHx[lRun1 * 20 + 9] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHx[lRun1 * 20 + 10] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHx[lRun1 * 20 + 11] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evHx[lRun1 * 20 + 12] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evHx[lRun1 * 20 + 13] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evHx[lRun1 * 20 + 14] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evHx[lRun1 * 20 + 15] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evHx[lRun1 * 20 + 16] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evHx[lRun1 * 20 + 17] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evHx[lRun1 * 20 + 18] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evHx[lRun1 * 20 + 19] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evHu[lRun1 * 8] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evHu[lRun1 * 8 + 1] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evHu[lRun1 * 8 + 2] = acadoWorkspace.conValueOut[26];
acadoWorkspace.evHu[lRun1 * 8 + 3] = acadoWorkspace.conValueOut[27];
acadoWorkspace.evHu[lRun1 * 8 + 4] = acadoWorkspace.conValueOut[28];
acadoWorkspace.evHu[lRun1 * 8 + 5] = acadoWorkspace.conValueOut[29];
acadoWorkspace.evHu[lRun1 * 8 + 6] = acadoWorkspace.conValueOut[30];
acadoWorkspace.evHu[lRun1 * 8 + 7] = acadoWorkspace.conValueOut[31];
}

acadoWorkspace.A01[0] = acadoWorkspace.evHx[0];
acadoWorkspace.A01[1] = acadoWorkspace.evHx[1];
acadoWorkspace.A01[2] = acadoWorkspace.evHx[2];
acadoWorkspace.A01[3] = acadoWorkspace.evHx[3];
acadoWorkspace.A01[4] = acadoWorkspace.evHx[4];
acadoWorkspace.A01[5] = acadoWorkspace.evHx[5];
acadoWorkspace.A01[6] = acadoWorkspace.evHx[6];
acadoWorkspace.A01[7] = acadoWorkspace.evHx[7];
acadoWorkspace.A01[8] = acadoWorkspace.evHx[8];
acadoWorkspace.A01[9] = acadoWorkspace.evHx[9];
acadoWorkspace.A01[10] = acadoWorkspace.evHx[10];
acadoWorkspace.A01[11] = acadoWorkspace.evHx[11];
acadoWorkspace.A01[12] = acadoWorkspace.evHx[12];
acadoWorkspace.A01[13] = acadoWorkspace.evHx[13];
acadoWorkspace.A01[14] = acadoWorkspace.evHx[14];
acadoWorkspace.A01[15] = acadoWorkspace.evHx[15];
acadoWorkspace.A01[16] = acadoWorkspace.evHx[16];
acadoWorkspace.A01[17] = acadoWorkspace.evHx[17];
acadoWorkspace.A01[18] = acadoWorkspace.evHx[18];
acadoWorkspace.A01[19] = acadoWorkspace.evHx[19];

acado_multHxC( &(acadoWorkspace.evHx[ 20 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 20 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.A01[ 40 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.A01[ 60 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.A01[ 80 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.A01[ 100 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.A01[ 120 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.A01[ 140 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.A01[ 160 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.A01[ 180 ]) );

acado_multHxE( &(acadoWorkspace.evHx[ 20 ]), acadoWorkspace.E, 1, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.E[ 10 ]), 2, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.E[ 20 ]), 2, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 30 ]), 3, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 40 ]), 3, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 50 ]), 3, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 60 ]), 4, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 70 ]), 4, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 80 ]), 4, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 90 ]), 4, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 100 ]), 5, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 110 ]), 5, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 120 ]), 5, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 130 ]), 5, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 140 ]), 5, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 150 ]), 6, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 160 ]), 6, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 170 ]), 6, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 180 ]), 6, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 190 ]), 6, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 200 ]), 6, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 210 ]), 7, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 220 ]), 7, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 230 ]), 7, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 240 ]), 7, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 250 ]), 7, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 260 ]), 7, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 270 ]), 7, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 280 ]), 8, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 290 ]), 8, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 300 ]), 8, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 310 ]), 8, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 320 ]), 8, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 330 ]), 8, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 340 ]), 8, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 350 ]), 8, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 360 ]), 9, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 370 ]), 9, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 380 ]), 9, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 390 ]), 9, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 400 ]), 9, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 410 ]), 9, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 420 ]), 9, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 430 ]), 9, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 440 ]), 9, 8 );

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[20] = acadoWorkspace.evHu[2];
acadoWorkspace.A[21] = acadoWorkspace.evHu[3];
acadoWorkspace.A[40] = acadoWorkspace.evHu[4];
acadoWorkspace.A[41] = acadoWorkspace.evHu[5];
acadoWorkspace.A[60] = acadoWorkspace.evHu[6];
acadoWorkspace.A[61] = acadoWorkspace.evHu[7];
acadoWorkspace.A[82] = acadoWorkspace.evHu[8];
acadoWorkspace.A[83] = acadoWorkspace.evHu[9];
acadoWorkspace.A[102] = acadoWorkspace.evHu[10];
acadoWorkspace.A[103] = acadoWorkspace.evHu[11];
acadoWorkspace.A[122] = acadoWorkspace.evHu[12];
acadoWorkspace.A[123] = acadoWorkspace.evHu[13];
acadoWorkspace.A[142] = acadoWorkspace.evHu[14];
acadoWorkspace.A[143] = acadoWorkspace.evHu[15];
acadoWorkspace.A[164] = acadoWorkspace.evHu[16];
acadoWorkspace.A[165] = acadoWorkspace.evHu[17];
acadoWorkspace.A[184] = acadoWorkspace.evHu[18];
acadoWorkspace.A[185] = acadoWorkspace.evHu[19];
acadoWorkspace.A[204] = acadoWorkspace.evHu[20];
acadoWorkspace.A[205] = acadoWorkspace.evHu[21];
acadoWorkspace.A[224] = acadoWorkspace.evHu[22];
acadoWorkspace.A[225] = acadoWorkspace.evHu[23];
acadoWorkspace.A[246] = acadoWorkspace.evHu[24];
acadoWorkspace.A[247] = acadoWorkspace.evHu[25];
acadoWorkspace.A[266] = acadoWorkspace.evHu[26];
acadoWorkspace.A[267] = acadoWorkspace.evHu[27];
acadoWorkspace.A[286] = acadoWorkspace.evHu[28];
acadoWorkspace.A[287] = acadoWorkspace.evHu[29];
acadoWorkspace.A[306] = acadoWorkspace.evHu[30];
acadoWorkspace.A[307] = acadoWorkspace.evHu[31];
acadoWorkspace.A[328] = acadoWorkspace.evHu[32];
acadoWorkspace.A[329] = acadoWorkspace.evHu[33];
acadoWorkspace.A[348] = acadoWorkspace.evHu[34];
acadoWorkspace.A[349] = acadoWorkspace.evHu[35];
acadoWorkspace.A[368] = acadoWorkspace.evHu[36];
acadoWorkspace.A[369] = acadoWorkspace.evHu[37];
acadoWorkspace.A[388] = acadoWorkspace.evHu[38];
acadoWorkspace.A[389] = acadoWorkspace.evHu[39];
acadoWorkspace.A[410] = acadoWorkspace.evHu[40];
acadoWorkspace.A[411] = acadoWorkspace.evHu[41];
acadoWorkspace.A[430] = acadoWorkspace.evHu[42];
acadoWorkspace.A[431] = acadoWorkspace.evHu[43];
acadoWorkspace.A[450] = acadoWorkspace.evHu[44];
acadoWorkspace.A[451] = acadoWorkspace.evHu[45];
acadoWorkspace.A[470] = acadoWorkspace.evHu[46];
acadoWorkspace.A[471] = acadoWorkspace.evHu[47];
acadoWorkspace.A[492] = acadoWorkspace.evHu[48];
acadoWorkspace.A[493] = acadoWorkspace.evHu[49];
acadoWorkspace.A[512] = acadoWorkspace.evHu[50];
acadoWorkspace.A[513] = acadoWorkspace.evHu[51];
acadoWorkspace.A[532] = acadoWorkspace.evHu[52];
acadoWorkspace.A[533] = acadoWorkspace.evHu[53];
acadoWorkspace.A[552] = acadoWorkspace.evHu[54];
acadoWorkspace.A[553] = acadoWorkspace.evHu[55];
acadoWorkspace.A[574] = acadoWorkspace.evHu[56];
acadoWorkspace.A[575] = acadoWorkspace.evHu[57];
acadoWorkspace.A[594] = acadoWorkspace.evHu[58];
acadoWorkspace.A[595] = acadoWorkspace.evHu[59];
acadoWorkspace.A[614] = acadoWorkspace.evHu[60];
acadoWorkspace.A[615] = acadoWorkspace.evHu[61];
acadoWorkspace.A[634] = acadoWorkspace.evHu[62];
acadoWorkspace.A[635] = acadoWorkspace.evHu[63];
acadoWorkspace.A[656] = acadoWorkspace.evHu[64];
acadoWorkspace.A[657] = acadoWorkspace.evHu[65];
acadoWorkspace.A[676] = acadoWorkspace.evHu[66];
acadoWorkspace.A[677] = acadoWorkspace.evHu[67];
acadoWorkspace.A[696] = acadoWorkspace.evHu[68];
acadoWorkspace.A[697] = acadoWorkspace.evHu[69];
acadoWorkspace.A[716] = acadoWorkspace.evHu[70];
acadoWorkspace.A[717] = acadoWorkspace.evHu[71];
acadoWorkspace.A[738] = acadoWorkspace.evHu[72];
acadoWorkspace.A[739] = acadoWorkspace.evHu[73];
acadoWorkspace.A[758] = acadoWorkspace.evHu[74];
acadoWorkspace.A[759] = acadoWorkspace.evHu[75];
acadoWorkspace.A[778] = acadoWorkspace.evHu[76];
acadoWorkspace.A[779] = acadoWorkspace.evHu[77];
acadoWorkspace.A[798] = acadoWorkspace.evHu[78];
acadoWorkspace.A[799] = acadoWorkspace.evHu[79];
acadoWorkspace.lbA[0] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[4];
acadoWorkspace.lbA[5] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[5];
acadoWorkspace.lbA[6] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[6];
acadoWorkspace.lbA[7] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[7];
acadoWorkspace.lbA[8] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[8];
acadoWorkspace.lbA[9] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[9];
acadoWorkspace.lbA[10] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[10];
acadoWorkspace.lbA[11] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[11];
acadoWorkspace.lbA[12] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[12];
acadoWorkspace.lbA[13] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[13];
acadoWorkspace.lbA[14] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[14];
acadoWorkspace.lbA[15] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[15];
acadoWorkspace.lbA[16] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[16];
acadoWorkspace.lbA[17] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[17];
acadoWorkspace.lbA[18] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[18];
acadoWorkspace.lbA[19] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[19];
acadoWorkspace.lbA[20] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[20];
acadoWorkspace.lbA[21] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[21];
acadoWorkspace.lbA[22] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[22];
acadoWorkspace.lbA[23] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[23];
acadoWorkspace.lbA[24] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[24];
acadoWorkspace.lbA[25] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[25];
acadoWorkspace.lbA[26] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[26];
acadoWorkspace.lbA[27] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[27];
acadoWorkspace.lbA[28] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[28];
acadoWorkspace.lbA[29] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[29];
acadoWorkspace.lbA[30] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[30];
acadoWorkspace.lbA[31] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[31];
acadoWorkspace.lbA[32] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[32];
acadoWorkspace.lbA[33] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[33];
acadoWorkspace.lbA[34] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[34];
acadoWorkspace.lbA[35] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[35];
acadoWorkspace.lbA[36] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[36];
acadoWorkspace.lbA[37] = (real_t)-7.1999999999999997e-01 - acadoWorkspace.evH[37];
acadoWorkspace.lbA[38] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[38];
acadoWorkspace.lbA[39] = (real_t)-9.5999999999999996e-01 - acadoWorkspace.evH[39];

acadoWorkspace.ubA[0] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[4];
acadoWorkspace.ubA[5] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[5];
acadoWorkspace.ubA[6] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[6];
acadoWorkspace.ubA[7] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[7];
acadoWorkspace.ubA[8] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[8];
acadoWorkspace.ubA[9] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[9];
acadoWorkspace.ubA[10] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[10];
acadoWorkspace.ubA[11] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[11];
acadoWorkspace.ubA[12] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[12];
acadoWorkspace.ubA[13] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[13];
acadoWorkspace.ubA[14] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[14];
acadoWorkspace.ubA[15] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[15];
acadoWorkspace.ubA[16] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[16];
acadoWorkspace.ubA[17] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[17];
acadoWorkspace.ubA[18] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[18];
acadoWorkspace.ubA[19] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[19];
acadoWorkspace.ubA[20] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[20];
acadoWorkspace.ubA[21] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[21];
acadoWorkspace.ubA[22] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[22];
acadoWorkspace.ubA[23] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[23];
acadoWorkspace.ubA[24] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[24];
acadoWorkspace.ubA[25] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[25];
acadoWorkspace.ubA[26] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[26];
acadoWorkspace.ubA[27] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[27];
acadoWorkspace.ubA[28] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[28];
acadoWorkspace.ubA[29] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[29];
acadoWorkspace.ubA[30] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[30];
acadoWorkspace.ubA[31] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[31];
acadoWorkspace.ubA[32] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[32];
acadoWorkspace.ubA[33] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[33];
acadoWorkspace.ubA[34] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[34];
acadoWorkspace.ubA[35] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[35];
acadoWorkspace.ubA[36] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[36];
acadoWorkspace.ubA[37] = (real_t)7.1999999999999997e-01 - acadoWorkspace.evH[37];
acadoWorkspace.ubA[38] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[38];
acadoWorkspace.ubA[39] = (real_t)9.5999999999999996e-01 - acadoWorkspace.evH[39];

}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];

acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 18 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.QDy[ 5 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.QDy[ 25 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 35 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 45 ]) );

acadoWorkspace.QDy[50] = + (real_t)5.0000000000000000e+03*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[51] = + (real_t)5.0000000000000000e+03*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[52] = + (real_t)5.0000000000000000e+01*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[53] = 0.0;
;
acadoWorkspace.QDy[54] = 0.0;
;

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 5 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.QDy[ 10 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QDy[ 15 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QDy[ 25 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QDy[ 35 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QDy[ 50 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QDy[ 10 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.QDy[ 25 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.QDy[ 35 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QDy[ 25 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.QDy[ 35 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 130 ]), &(acadoWorkspace.QDy[ 25 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QDy[ 35 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.QDy[ 25 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 190 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 250 ]), &(acadoWorkspace.QDy[ 35 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 260 ]), &(acadoWorkspace.QDy[ 35 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QDy[ 35 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 340 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 350 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 430 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 530 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QDy[ 50 ]), &(acadoWorkspace.g[ 18 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[1] += + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[2] += + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[3] += + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[4] += + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[5] += + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[6] += + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[7] += + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[8] += + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[9] += + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[10] += + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[11] += + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[12] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[13] += + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[14] += + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[15] += + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[16] += + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[17] += + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[18] += + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[19] += + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[4];

acadoWorkspace.pacA01Dx0[0] = + acadoWorkspace.A01[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[4]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[1] = + acadoWorkspace.A01[5]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[6]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[7]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[8]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[9]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[2] = + acadoWorkspace.A01[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[14]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[3] = + acadoWorkspace.A01[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[18]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[19]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[4] = + acadoWorkspace.A01[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[24]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[5] = + acadoWorkspace.A01[25]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[26]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[27]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[28]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[29]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[6] = + acadoWorkspace.A01[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[34]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[7] = + acadoWorkspace.A01[35]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[36]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[37]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[38]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[39]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[8] = + acadoWorkspace.A01[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[44]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[9] = + acadoWorkspace.A01[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[49]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[10] = + acadoWorkspace.A01[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[54]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[11] = + acadoWorkspace.A01[55]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[56]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[57]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[58]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[59]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[12] = + acadoWorkspace.A01[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[64]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[13] = + acadoWorkspace.A01[65]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[66]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[67]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[68]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[69]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[14] = + acadoWorkspace.A01[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[74]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[15] = + acadoWorkspace.A01[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[78]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[79]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[16] = + acadoWorkspace.A01[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[84]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[17] = + acadoWorkspace.A01[85]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[86]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[87]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[88]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[89]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[18] = + acadoWorkspace.A01[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[94]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[19] = + acadoWorkspace.A01[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[99]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[20] = + acadoWorkspace.A01[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[104]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[21] = + acadoWorkspace.A01[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[108]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[109]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[22] = + acadoWorkspace.A01[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[111]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[112]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[113]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[114]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[23] = + acadoWorkspace.A01[115]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[116]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[117]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[118]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[119]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[24] = + acadoWorkspace.A01[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[124]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[25] = + acadoWorkspace.A01[125]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[126]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[127]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[128]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[129]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[26] = + acadoWorkspace.A01[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[131]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[132]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[133]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[134]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[27] = + acadoWorkspace.A01[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[138]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[139]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[28] = + acadoWorkspace.A01[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[144]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[29] = + acadoWorkspace.A01[145]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[146]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[147]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[148]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[149]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[30] = + acadoWorkspace.A01[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[154]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[31] = + acadoWorkspace.A01[155]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[156]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[157]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[158]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[159]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[32] = + acadoWorkspace.A01[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[164]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[33] = + acadoWorkspace.A01[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[168]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[169]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[34] = + acadoWorkspace.A01[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[174]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[35] = + acadoWorkspace.A01[175]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[176]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[177]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[178]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[179]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[36] = + acadoWorkspace.A01[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[184]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[37] = + acadoWorkspace.A01[185]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[186]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[187]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[188]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[189]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[38] = + acadoWorkspace.A01[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[194]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[39] = + acadoWorkspace.A01[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[198]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[199]*acadoWorkspace.Dx0[4];
acadoWorkspace.lbA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.lbA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.lbA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.lbA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.lbA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.lbA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.lbA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.lbA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.lbA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.lbA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.lbA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.lbA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.lbA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.lbA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.lbA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.lbA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.lbA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.lbA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.lbA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.lbA[19] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.lbA[20] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.lbA[21] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.lbA[22] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.lbA[23] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.lbA[24] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.lbA[25] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.lbA[26] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.lbA[27] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.lbA[28] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.lbA[29] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.lbA[30] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.lbA[31] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.lbA[32] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.lbA[33] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.lbA[34] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.lbA[35] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.lbA[36] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.lbA[37] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.lbA[38] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.lbA[39] -= acadoWorkspace.pacA01Dx0[39];

acadoWorkspace.ubA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.ubA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.ubA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.ubA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.ubA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.ubA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.ubA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.ubA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.ubA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.ubA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.ubA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.ubA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.ubA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.ubA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.ubA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.ubA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.ubA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.ubA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.ubA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.ubA[19] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.ubA[20] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.ubA[21] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.ubA[22] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.ubA[23] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.ubA[24] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.ubA[25] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.ubA[26] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.ubA[27] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.ubA[28] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.ubA[29] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.ubA[30] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.ubA[31] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.ubA[32] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.ubA[33] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.ubA[34] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.ubA[35] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.ubA[36] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.ubA[37] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.ubA[38] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.ubA[39] -= acadoWorkspace.pacA01Dx0[39];

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];
acadoVariables.x[4] += acadoWorkspace.Dx0[4];

acadoVariables.x[5] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[4];
acadoVariables.x[6] += + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[4];
acadoVariables.x[7] += + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[4];
acadoVariables.x[8] += + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[4];
acadoVariables.x[9] += + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[4];
acadoVariables.x[10] += + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[4];
acadoVariables.x[11] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[4];
acadoVariables.x[12] += + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[4];
acadoVariables.x[13] += + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[4];
acadoVariables.x[14] += + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[4];
acadoVariables.x[15] += + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[4];
acadoVariables.x[16] += + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[4];
acadoVariables.x[17] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[4];
acadoVariables.x[18] += + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[4];
acadoVariables.x[19] += + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[4];
acadoVariables.x[20] += + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[4];
acadoVariables.x[21] += + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[4];
acadoVariables.x[22] += + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[4];
acadoVariables.x[23] += + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[4];
acadoVariables.x[24] += + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[4];
acadoVariables.x[25] += + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[4];
acadoVariables.x[26] += + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[4];
acadoVariables.x[27] += + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[4];
acadoVariables.x[28] += + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[4];
acadoVariables.x[29] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[4];
acadoVariables.x[30] += + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[4];
acadoVariables.x[31] += + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[4];
acadoVariables.x[32] += + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[4];
acadoVariables.x[33] += + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[4];
acadoVariables.x[34] += + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[4];
acadoVariables.x[35] += + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[4];
acadoVariables.x[36] += + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[4];
acadoVariables.x[37] += + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[4];
acadoVariables.x[38] += + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[4];
acadoVariables.x[39] += + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[4];
acadoVariables.x[40] += + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[4];
acadoVariables.x[41] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[4];
acadoVariables.x[42] += + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[4];
acadoVariables.x[43] += + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[4];
acadoVariables.x[44] += + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[4];
acadoVariables.x[45] += + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[4];
acadoVariables.x[46] += + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[4];
acadoVariables.x[47] += + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[4];
acadoVariables.x[48] += + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[4];
acadoVariables.x[49] += + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[4];
acadoVariables.x[50] += + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[4];
acadoVariables.x[51] += + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[4];
acadoVariables.x[52] += + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[4];
acadoVariables.x[53] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[4];
acadoVariables.x[54] += + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[4];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 5 ]) );
acado_multEDu( &(acadoWorkspace.E[ 10 ]), acadoWorkspace.x, &(acadoVariables.x[ 10 ]) );
acado_multEDu( &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 10 ]) );
acado_multEDu( &(acadoWorkspace.E[ 30 ]), acadoWorkspace.x, &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 60 ]), acadoWorkspace.x, &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 100 ]), acadoWorkspace.x, &(acadoVariables.x[ 25 ]) );
acado_multEDu( &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 25 ]) );
acado_multEDu( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 25 ]) );
acado_multEDu( &(acadoWorkspace.E[ 130 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 25 ]) );
acado_multEDu( &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 25 ]) );
acado_multEDu( &(acadoWorkspace.E[ 150 ]), acadoWorkspace.x, &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 190 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 210 ]), acadoWorkspace.x, &(acadoVariables.x[ 35 ]) );
acado_multEDu( &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 35 ]) );
acado_multEDu( &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 35 ]) );
acado_multEDu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 35 ]) );
acado_multEDu( &(acadoWorkspace.E[ 250 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 35 ]) );
acado_multEDu( &(acadoWorkspace.E[ 260 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 35 ]) );
acado_multEDu( &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 35 ]) );
acado_multEDu( &(acadoWorkspace.E[ 280 ]), acadoWorkspace.x, &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 340 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 350 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 360 ]), acadoWorkspace.x, &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 430 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 450 ]), acadoWorkspace.x, &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 530 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 50 ]) );
acado_multEDu( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 50 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 5];
acadoWorkspace.state[1] = acadoVariables.x[index * 5 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 5 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 5 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 5 + 4];
acadoWorkspace.state[40] = acadoVariables.u[index * 2];
acadoWorkspace.state[41] = acadoVariables.u[index * 2 + 1];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 5 + 5] = acadoWorkspace.state[0];
acadoVariables.x[index * 5 + 6] = acadoWorkspace.state[1];
acadoVariables.x[index * 5 + 7] = acadoWorkspace.state[2];
acadoVariables.x[index * 5 + 8] = acadoWorkspace.state[3];
acadoVariables.x[index * 5 + 9] = acadoWorkspace.state[4];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 5] = acadoVariables.x[index * 5 + 5];
acadoVariables.x[index * 5 + 1] = acadoVariables.x[index * 5 + 6];
acadoVariables.x[index * 5 + 2] = acadoVariables.x[index * 5 + 7];
acadoVariables.x[index * 5 + 3] = acadoVariables.x[index * 5 + 8];
acadoVariables.x[index * 5 + 4] = acadoVariables.x[index * 5 + 9];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[50] = xEnd[0];
acadoVariables.x[51] = xEnd[1];
acadoVariables.x[52] = xEnd[2];
acadoVariables.x[53] = xEnd[3];
acadoVariables.x[54] = xEnd[4];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[50];
acadoWorkspace.state[1] = acadoVariables.x[51];
acadoWorkspace.state[2] = acadoVariables.x[52];
acadoWorkspace.state[3] = acadoVariables.x[53];
acadoWorkspace.state[4] = acadoVariables.x[54];
if (uEnd != 0)
{
acadoWorkspace.state[40] = uEnd[0];
acadoWorkspace.state[41] = uEnd[1];
}
else
{
acadoWorkspace.state[40] = acadoVariables.u[18];
acadoWorkspace.state[41] = acadoVariables.u[19];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[50] = acadoWorkspace.state[0];
acadoVariables.x[51] = acadoWorkspace.state[1];
acadoVariables.x[52] = acadoWorkspace.state[2];
acadoVariables.x[53] = acadoWorkspace.state[3];
acadoVariables.x[54] = acadoWorkspace.state[4];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[18] = uEnd[0];
acadoVariables.u[19] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19];
kkt = fabs( kkt );
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 40; ++index)
{
prd = acadoWorkspace.y[index + 20];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 5 */
real_t tmpDy[ 5 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 5 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 5] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 5];
acadoWorkspace.Dy[lRun1 * 5 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 5 + 1];
acadoWorkspace.Dy[lRun1 * 5 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 5 + 2];
acadoWorkspace.Dy[lRun1 * 5 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 5 + 3];
acadoWorkspace.Dy[lRun1 * 5 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 5 + 4];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[50];
acadoWorkspace.objValueIn[1] = acadoVariables.x[51];
acadoWorkspace.objValueIn[2] = acadoVariables.x[52];
acadoWorkspace.objValueIn[3] = acadoVariables.x[53];
acadoWorkspace.objValueIn[4] = acadoVariables.x[54];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 5]*(real_t)1.0000000000000000e+03;
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 5 + 1]*(real_t)1.0000000000000000e+03;
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 5 + 2]*(real_t)1.0000000000000000e+01;
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 5 + 3];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 5 + 4];
objVal += + acadoWorkspace.Dy[lRun1 * 5]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 5 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 5 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 5 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 5 + 4]*tmpDy[4];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)5.0000000000000000e+03;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)5.0000000000000000e+03;
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)5.0000000000000000e+01;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

