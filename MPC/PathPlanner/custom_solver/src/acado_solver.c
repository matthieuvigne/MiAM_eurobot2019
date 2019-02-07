/*
 *      */


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

for (lRun1 = 0; lRun1 < 21; ++lRun1)
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
out[3] = xd[3];
out[4] = xd[4];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 21; ++runObj)
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
acadoWorkspace.objValueIn[0] = acadoVariables.x[105];
acadoWorkspace.objValueIn[1] = acadoVariables.x[106];
acadoWorkspace.objValueIn[2] = acadoVariables.x[107];
acadoWorkspace.objValueIn[3] = acadoVariables.x[108];
acadoWorkspace.objValueIn[4] = acadoVariables.x[109];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];

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
acadoWorkspace.H[(iRow * 84) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8];
acadoWorkspace.H[(iRow * 84) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9];
acadoWorkspace.H[(iRow * 84 + 42) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8];
acadoWorkspace.H[(iRow * 84 + 42) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 84) + (iCol * 2)] = 0.0;
acadoWorkspace.H[(iRow * 84) + (iCol * 2 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 84 + 42) + (iCol * 2)] = 0.0;
acadoWorkspace.H[(iRow * 84 + 42) + (iCol * 2 + 1)] = 0.0;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 84) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 84) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 84 + 42) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 84 + 42) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 84) + (iCol * 2)] = acadoWorkspace.H[(iCol * 84) + (iRow * 2)];
acadoWorkspace.H[(iRow * 84) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 84 + 42) + (iRow * 2)];
acadoWorkspace.H[(iRow * 84 + 42) + (iCol * 2)] = acadoWorkspace.H[(iCol * 84) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 84 + 42) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 84 + 42) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = +dOld[0];
dNew[1] = +dOld[1];
dNew[2] = +dOld[2];
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
QDy1[0] = +Dy1[0];
QDy1[1] = +Dy1[1];
QDy1[2] = +Dy1[2];
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
Gx2[0] = +Gx1[0];
Gx2[1] = +Gx1[1];
Gx2[2] = +Gx1[2];
Gx2[3] = +Gx1[3];
Gx2[4] = +Gx1[4];
Gx2[5] = +Gx1[5];
Gx2[6] = +Gx1[6];
Gx2[7] = +Gx1[7];
Gx2[8] = +Gx1[8];
Gx2[9] = +Gx1[9];
Gx2[10] = +Gx1[10];
Gx2[11] = +Gx1[11];
Gx2[12] = +Gx1[12];
Gx2[13] = +Gx1[13];
Gx2[14] = +Gx1[14];
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
Gx2[0] = + (real_t)5.0000000000000000e+00*Gx1[0];
Gx2[1] = + (real_t)5.0000000000000000e+00*Gx1[1];
Gx2[2] = + (real_t)5.0000000000000000e+00*Gx1[2];
Gx2[3] = + (real_t)5.0000000000000000e+00*Gx1[3];
Gx2[4] = + (real_t)5.0000000000000000e+00*Gx1[4];
Gx2[5] = + (real_t)5.0000000000000000e+00*Gx1[5];
Gx2[6] = + (real_t)5.0000000000000000e+00*Gx1[6];
Gx2[7] = + (real_t)5.0000000000000000e+00*Gx1[7];
Gx2[8] = + (real_t)5.0000000000000000e+00*Gx1[8];
Gx2[9] = + (real_t)5.0000000000000000e+00*Gx1[9];
Gx2[10] = + (real_t)5.0000000000000000e+00*Gx1[10];
Gx2[11] = + (real_t)5.0000000000000000e+00*Gx1[11];
Gx2[12] = + (real_t)5.0000000000000000e+00*Gx1[12];
Gx2[13] = + (real_t)5.0000000000000000e+00*Gx1[13];
Gx2[14] = + (real_t)5.0000000000000000e+00*Gx1[14];
Gx2[15] = + (real_t)5.0000000000000000e+00*Gx1[15];
Gx2[16] = + (real_t)5.0000000000000000e+00*Gx1[16];
Gx2[17] = + (real_t)5.0000000000000000e+00*Gx1[17];
Gx2[18] = + (real_t)5.0000000000000000e+00*Gx1[18];
Gx2[19] = + (real_t)5.0000000000000000e+00*Gx1[19];
Gx2[20] = + (real_t)5.0000000000000000e+00*Gx1[20];
Gx2[21] = + (real_t)5.0000000000000000e+00*Gx1[21];
Gx2[22] = + (real_t)5.0000000000000000e+00*Gx1[22];
Gx2[23] = + (real_t)5.0000000000000000e+00*Gx1[23];
Gx2[24] = + (real_t)5.0000000000000000e+00*Gx1[24];
}

void acado_multQ1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = +Gu1[0];
Gu2[1] = +Gu1[1];
Gu2[2] = +Gu1[2];
Gu2[3] = +Gu1[3];
Gu2[4] = +Gu1[4];
Gu2[5] = +Gu1[5];
Gu2[6] = +Gu1[6];
Gu2[7] = +Gu1[7];
Gu2[8] = +Gu1[8];
Gu2[9] = +Gu1[9];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)5.0000000000000000e+00*Gu1[0];
Gu2[1] = + (real_t)5.0000000000000000e+00*Gu1[1];
Gu2[2] = + (real_t)5.0000000000000000e+00*Gu1[2];
Gu2[3] = + (real_t)5.0000000000000000e+00*Gu1[3];
Gu2[4] = + (real_t)5.0000000000000000e+00*Gu1[4];
Gu2[5] = + (real_t)5.0000000000000000e+00*Gu1[5];
Gu2[6] = + (real_t)5.0000000000000000e+00*Gu1[6];
Gu2[7] = + (real_t)5.0000000000000000e+00*Gu1[7];
Gu2[8] = + (real_t)5.0000000000000000e+00*Gu1[8];
Gu2[9] = + (real_t)5.0000000000000000e+00*Gu1[9];
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
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 42 */
static const int xBoundIndices[ 42 ] = 
{ 8, 9, 13, 14, 18, 19, 23, 24, 28, 29, 33, 34, 38, 39, 43, 44, 48, 49, 53, 54, 58, 59, 63, 64, 68, 69, 73, 74, 78, 79, 83, 84, 88, 89, 93, 94, 98, 99, 103, 104, 108, 109 };
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
for (lRun1 = 1; lRun1 < 21; ++lRun1)
{
acado_moveGxT( &(acadoWorkspace.evGx[ lRun1 * 25 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ lRun1 * 25-25 ]), &(acadoWorkspace.evGx[ lRun1 * 25 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ lRun4 * 10 ]), &(acadoWorkspace.E[ lRun3 * 10 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun1 * 10 ]), &(acadoWorkspace.E[ lRun3 * 10 ]) );
}

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multQ1Gu( &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.QE[ lRun3 * 10 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multQN1Gu( &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.QE[ lRun3 * 10 ]) );
}

for (lRun1 = 0; lRun1 < 21; ++lRun1)
{
acado_zeroBlockH10( &(acadoWorkspace.H10[ lRun1 * 10 ]) );
for (lRun2 = lRun1; lRun2 < 21; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multQETGx( &(acadoWorkspace.QE[ lRun3 * 10 ]), &(acadoWorkspace.evGx[ lRun2 * 25 ]), &(acadoWorkspace.H10[ lRun1 * 10 ]) );
}
}

for (lRun1 = 0; lRun1 < 21; ++lRun1)
{
acado_setBlockH11_R1( lRun1, lRun1 );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 21; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 10 ]), &(acadoWorkspace.QE[ lRun5 * 10 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 21; ++lRun2)
{
acado_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 21; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 10 ]), &(acadoWorkspace.QE[ lRun5 * 10 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 21; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun1, lRun2 );
}
}

for (lRun1 = 0; lRun1 < 21; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 21; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_macETSlu( &(acadoWorkspace.QE[ lRun3 * 10 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}
acadoWorkspace.lb[0] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-8.0000000000000000e+02 - acadoVariables.u[41];
acadoWorkspace.ub[0] = (real_t)8.0000000000000000e+02 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)8.0000000000000000e+02 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)8.0000000000000000e+02 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)8.0000000000000000e+02 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)8.0000000000000000e+02 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)8.0000000000000000e+02 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)8.0000000000000000e+02 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)8.0000000000000000e+02 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)8.0000000000000000e+02 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)8.0000000000000000e+02 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)8.0000000000000000e+02 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)8.0000000000000000e+02 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)8.0000000000000000e+02 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)8.0000000000000000e+02 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)8.0000000000000000e+02 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)8.0000000000000000e+02 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)8.0000000000000000e+02 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)8.0000000000000000e+02 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)8.0000000000000000e+02 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)8.0000000000000000e+02 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)8.0000000000000000e+02 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)8.0000000000000000e+02 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)8.0000000000000000e+02 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)8.0000000000000000e+02 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)8.0000000000000000e+02 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)8.0000000000000000e+02 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)8.0000000000000000e+02 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)8.0000000000000000e+02 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)8.0000000000000000e+02 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)8.0000000000000000e+02 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)8.0000000000000000e+02 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)8.0000000000000000e+02 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)8.0000000000000000e+02 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)8.0000000000000000e+02 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)8.0000000000000000e+02 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)8.0000000000000000e+02 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)8.0000000000000000e+02 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)8.0000000000000000e+02 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)8.0000000000000000e+02 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)8.0000000000000000e+02 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)8.0000000000000000e+02 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)8.0000000000000000e+02 - acadoVariables.u[41];

for (lRun1 = 0; lRun1 < 42; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 5;
lRun4 = ((lRun3) / (5)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = (((((lRun4) * (lRun4-1)) / (2)) + (lRun2)) * (5)) + ((lRun3) % (5));
acadoWorkspace.A[(lRun1 * 42) + (lRun2 * 2)] = acadoWorkspace.E[lRun5 * 2];
acadoWorkspace.A[(lRun1 * 42) + (lRun2 * 2 + 1)] = acadoWorkspace.E[lRun5 * 2 + 1];
}
}

}

void acado_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
real_t tmp;

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
acadoWorkspace.Dy[50] -= acadoVariables.y[50];
acadoWorkspace.Dy[51] -= acadoVariables.y[51];
acadoWorkspace.Dy[52] -= acadoVariables.y[52];
acadoWorkspace.Dy[53] -= acadoVariables.y[53];
acadoWorkspace.Dy[54] -= acadoVariables.y[54];
acadoWorkspace.Dy[55] -= acadoVariables.y[55];
acadoWorkspace.Dy[56] -= acadoVariables.y[56];
acadoWorkspace.Dy[57] -= acadoVariables.y[57];
acadoWorkspace.Dy[58] -= acadoVariables.y[58];
acadoWorkspace.Dy[59] -= acadoVariables.y[59];
acadoWorkspace.Dy[60] -= acadoVariables.y[60];
acadoWorkspace.Dy[61] -= acadoVariables.y[61];
acadoWorkspace.Dy[62] -= acadoVariables.y[62];
acadoWorkspace.Dy[63] -= acadoVariables.y[63];
acadoWorkspace.Dy[64] -= acadoVariables.y[64];
acadoWorkspace.Dy[65] -= acadoVariables.y[65];
acadoWorkspace.Dy[66] -= acadoVariables.y[66];
acadoWorkspace.Dy[67] -= acadoVariables.y[67];
acadoWorkspace.Dy[68] -= acadoVariables.y[68];
acadoWorkspace.Dy[69] -= acadoVariables.y[69];
acadoWorkspace.Dy[70] -= acadoVariables.y[70];
acadoWorkspace.Dy[71] -= acadoVariables.y[71];
acadoWorkspace.Dy[72] -= acadoVariables.y[72];
acadoWorkspace.Dy[73] -= acadoVariables.y[73];
acadoWorkspace.Dy[74] -= acadoVariables.y[74];
acadoWorkspace.Dy[75] -= acadoVariables.y[75];
acadoWorkspace.Dy[76] -= acadoVariables.y[76];
acadoWorkspace.Dy[77] -= acadoVariables.y[77];
acadoWorkspace.Dy[78] -= acadoVariables.y[78];
acadoWorkspace.Dy[79] -= acadoVariables.y[79];
acadoWorkspace.Dy[80] -= acadoVariables.y[80];
acadoWorkspace.Dy[81] -= acadoVariables.y[81];
acadoWorkspace.Dy[82] -= acadoVariables.y[82];
acadoWorkspace.Dy[83] -= acadoVariables.y[83];
acadoWorkspace.Dy[84] -= acadoVariables.y[84];
acadoWorkspace.Dy[85] -= acadoVariables.y[85];
acadoWorkspace.Dy[86] -= acadoVariables.y[86];
acadoWorkspace.Dy[87] -= acadoVariables.y[87];
acadoWorkspace.Dy[88] -= acadoVariables.y[88];
acadoWorkspace.Dy[89] -= acadoVariables.y[89];
acadoWorkspace.Dy[90] -= acadoVariables.y[90];
acadoWorkspace.Dy[91] -= acadoVariables.y[91];
acadoWorkspace.Dy[92] -= acadoVariables.y[92];
acadoWorkspace.Dy[93] -= acadoVariables.y[93];
acadoWorkspace.Dy[94] -= acadoVariables.y[94];
acadoWorkspace.Dy[95] -= acadoVariables.y[95];
acadoWorkspace.Dy[96] -= acadoVariables.y[96];
acadoWorkspace.Dy[97] -= acadoVariables.y[97];
acadoWorkspace.Dy[98] -= acadoVariables.y[98];
acadoWorkspace.Dy[99] -= acadoVariables.y[99];
acadoWorkspace.Dy[100] -= acadoVariables.y[100];
acadoWorkspace.Dy[101] -= acadoVariables.y[101];
acadoWorkspace.Dy[102] -= acadoVariables.y[102];
acadoWorkspace.Dy[103] -= acadoVariables.y[103];
acadoWorkspace.Dy[104] -= acadoVariables.y[104];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];

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
acado_multRDy( &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.g[ 40 ]) );

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
acado_multQDy( &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.QDy[ 50 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.QDy[ 55 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.QDy[ 65 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 70 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.QDy[ 75 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.QDy[ 85 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.QDy[ 95 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.QDy[ 100 ]) );

acadoWorkspace.QDy[105] = + (real_t)5.0000000000000000e+00*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[106] = + (real_t)5.0000000000000000e+00*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[107] = + (real_t)5.0000000000000000e+00*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[108] = + (real_t)5.0000000000000000e+00*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[109] = + (real_t)5.0000000000000000e+00*acadoWorkspace.DyN[4];

for (lRun1 = 0; lRun1 < 21; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 21; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multEQDy( &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.QDy[ lRun2 * 5 + 5 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}

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
acadoWorkspace.g[20] += + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[21] += + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[22] += + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[23] += + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[24] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[25] += + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[26] += + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[27] += + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[28] += + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[29] += + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[30] += + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[31] += + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[32] += + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[33] += + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[34] += + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[35] += + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[36] += + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[37] += + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[38] += + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[39] += + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[40] += + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[41] += + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[4];

tmp = + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[4] + acadoVariables.x[8];
acadoWorkspace.lbA[0] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[0] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[4] + acadoVariables.x[9];
acadoWorkspace.lbA[1] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[1] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[4] + acadoVariables.x[13];
acadoWorkspace.lbA[2] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[2] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[4] + acadoVariables.x[14];
acadoWorkspace.lbA[3] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[3] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[4] + acadoVariables.x[18];
acadoWorkspace.lbA[4] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[4] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[4] + acadoVariables.x[19];
acadoWorkspace.lbA[5] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[5] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[4] + acadoVariables.x[23];
acadoWorkspace.lbA[6] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[6] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[4] + acadoVariables.x[24];
acadoWorkspace.lbA[7] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[7] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[4] + acadoVariables.x[28];
acadoWorkspace.lbA[8] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[8] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[4] + acadoVariables.x[29];
acadoWorkspace.lbA[9] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[9] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[4] + acadoVariables.x[33];
acadoWorkspace.lbA[10] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[10] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[4] + acadoVariables.x[34];
acadoWorkspace.lbA[11] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[11] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[4] + acadoVariables.x[38];
acadoWorkspace.lbA[12] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[12] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[4] + acadoVariables.x[39];
acadoWorkspace.lbA[13] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[13] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[4] + acadoVariables.x[43];
acadoWorkspace.lbA[14] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[14] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[4] + acadoVariables.x[44];
acadoWorkspace.lbA[15] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[15] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[4] + acadoVariables.x[48];
acadoWorkspace.lbA[16] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[16] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[4] + acadoVariables.x[49];
acadoWorkspace.lbA[17] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[17] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[4] + acadoVariables.x[53];
acadoWorkspace.lbA[18] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[18] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[4] + acadoVariables.x[54];
acadoWorkspace.lbA[19] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[19] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[4] + acadoVariables.x[58];
acadoWorkspace.lbA[20] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[20] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[4] + acadoVariables.x[59];
acadoWorkspace.lbA[21] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[21] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[4] + acadoVariables.x[63];
acadoWorkspace.lbA[22] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[22] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[4] + acadoVariables.x[64];
acadoWorkspace.lbA[23] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[23] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[4] + acadoVariables.x[68];
acadoWorkspace.lbA[24] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[24] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[4] + acadoVariables.x[69];
acadoWorkspace.lbA[25] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[25] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[4] + acadoVariables.x[73];
acadoWorkspace.lbA[26] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[26] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[4] + acadoVariables.x[74];
acadoWorkspace.lbA[27] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[27] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[4] + acadoVariables.x[78];
acadoWorkspace.lbA[28] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[28] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[4] + acadoVariables.x[79];
acadoWorkspace.lbA[29] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[29] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[4] + acadoVariables.x[83];
acadoWorkspace.lbA[30] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[30] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[4] + acadoVariables.x[84];
acadoWorkspace.lbA[31] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[31] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[415]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[4] + acadoVariables.x[88];
acadoWorkspace.lbA[32] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[32] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[4] + acadoVariables.x[89];
acadoWorkspace.lbA[33] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[33] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[443]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[444]*acadoWorkspace.Dx0[4] + acadoVariables.x[93];
acadoWorkspace.lbA[34] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[34] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[4] + acadoVariables.x[94];
acadoWorkspace.lbA[35] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[35] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[465]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[466]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[467]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[468]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[469]*acadoWorkspace.Dx0[4] + acadoVariables.x[98];
acadoWorkspace.lbA[36] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[36] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[472]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[473]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[474]*acadoWorkspace.Dx0[4] + acadoVariables.x[99];
acadoWorkspace.lbA[37] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[37] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[490]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[491]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[492]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[493]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[494]*acadoWorkspace.Dx0[4] + acadoVariables.x[103];
acadoWorkspace.lbA[38] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[38] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[495]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[496]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[497]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[498]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[499]*acadoWorkspace.Dx0[4] + acadoVariables.x[104];
acadoWorkspace.lbA[39] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[39] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[515]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[516]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[517]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[518]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[519]*acadoWorkspace.Dx0[4] + acadoVariables.x[108];
acadoWorkspace.lbA[40] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[40] = (real_t)6.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[520]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[521]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[522]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[523]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[524]*acadoWorkspace.Dx0[4] + acadoVariables.x[109];
acadoWorkspace.lbA[41] = (real_t)-6.0000000000000000e+02 - tmp;
acadoWorkspace.ubA[41] = (real_t)6.0000000000000000e+02 - tmp;

}

void acado_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
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
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];

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
acadoVariables.x[55] += + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[4];
acadoVariables.x[56] += + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[4];
acadoVariables.x[57] += + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[4];
acadoVariables.x[58] += + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[4];
acadoVariables.x[59] += + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[4];
acadoVariables.x[60] += + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[4];
acadoVariables.x[61] += + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[4];
acadoVariables.x[62] += + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[4];
acadoVariables.x[63] += + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[4];
acadoVariables.x[64] += + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[4];
acadoVariables.x[65] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[4];
acadoVariables.x[66] += + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[4];
acadoVariables.x[67] += + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[4];
acadoVariables.x[68] += + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[4];
acadoVariables.x[69] += + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[4];
acadoVariables.x[70] += + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[328]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[329]*acadoWorkspace.Dx0[4];
acadoVariables.x[71] += + acadoWorkspace.evGx[330]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[331]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[332]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[333]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[334]*acadoWorkspace.Dx0[4];
acadoVariables.x[72] += + acadoWorkspace.evGx[335]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[4];
acadoVariables.x[73] += + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[4];
acadoVariables.x[74] += + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[4];
acadoVariables.x[75] += + acadoWorkspace.evGx[350]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[351]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[4];
acadoVariables.x[76] += + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[4];
acadoVariables.x[77] += + acadoWorkspace.evGx[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[364]*acadoWorkspace.Dx0[4];
acadoVariables.x[78] += + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[4];
acadoVariables.x[79] += + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[4];
acadoVariables.x[80] += + acadoWorkspace.evGx[375]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[376]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[377]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[378]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[379]*acadoWorkspace.Dx0[4];
acadoVariables.x[81] += + acadoWorkspace.evGx[380]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[381]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[382]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[383]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[384]*acadoWorkspace.Dx0[4];
acadoVariables.x[82] += + acadoWorkspace.evGx[385]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[386]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[387]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[388]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[389]*acadoWorkspace.Dx0[4];
acadoVariables.x[83] += + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[4];
acadoVariables.x[84] += + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[4];
acadoVariables.x[85] += + acadoWorkspace.evGx[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[403]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[404]*acadoWorkspace.Dx0[4];
acadoVariables.x[86] += + acadoWorkspace.evGx[405]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[406]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[407]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[408]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[409]*acadoWorkspace.Dx0[4];
acadoVariables.x[87] += + acadoWorkspace.evGx[410]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[411]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[412]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[413]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[414]*acadoWorkspace.Dx0[4];
acadoVariables.x[88] += + acadoWorkspace.evGx[415]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[4];
acadoVariables.x[89] += + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[4];
acadoVariables.x[90] += + acadoWorkspace.evGx[425]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[426]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[427]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[428]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[429]*acadoWorkspace.Dx0[4];
acadoVariables.x[91] += + acadoWorkspace.evGx[430]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[431]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[432]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[433]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[434]*acadoWorkspace.Dx0[4];
acadoVariables.x[92] += + acadoWorkspace.evGx[435]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[436]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[437]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[438]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[439]*acadoWorkspace.Dx0[4];
acadoVariables.x[93] += + acadoWorkspace.evGx[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[443]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[444]*acadoWorkspace.Dx0[4];
acadoVariables.x[94] += + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[4];
acadoVariables.x[95] += + acadoWorkspace.evGx[450]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[451]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[452]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[453]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[454]*acadoWorkspace.Dx0[4];
acadoVariables.x[96] += + acadoWorkspace.evGx[455]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[456]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[457]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[458]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[459]*acadoWorkspace.Dx0[4];
acadoVariables.x[97] += + acadoWorkspace.evGx[460]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[461]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[462]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[463]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[464]*acadoWorkspace.Dx0[4];
acadoVariables.x[98] += + acadoWorkspace.evGx[465]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[466]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[467]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[468]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[469]*acadoWorkspace.Dx0[4];
acadoVariables.x[99] += + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[472]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[473]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[474]*acadoWorkspace.Dx0[4];
acadoVariables.x[100] += + acadoWorkspace.evGx[475]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[476]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[477]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[478]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[479]*acadoWorkspace.Dx0[4];
acadoVariables.x[101] += + acadoWorkspace.evGx[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[482]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[483]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[484]*acadoWorkspace.Dx0[4];
acadoVariables.x[102] += + acadoWorkspace.evGx[485]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[486]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[487]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[488]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[489]*acadoWorkspace.Dx0[4];
acadoVariables.x[103] += + acadoWorkspace.evGx[490]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[491]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[492]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[493]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[494]*acadoWorkspace.Dx0[4];
acadoVariables.x[104] += + acadoWorkspace.evGx[495]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[496]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[497]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[498]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[499]*acadoWorkspace.Dx0[4];
acadoVariables.x[105] += + acadoWorkspace.evGx[500]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[501]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[502]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[503]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[504]*acadoWorkspace.Dx0[4];
acadoVariables.x[106] += + acadoWorkspace.evGx[505]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[506]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[507]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[508]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[509]*acadoWorkspace.Dx0[4];
acadoVariables.x[107] += + acadoWorkspace.evGx[510]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[511]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[512]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[513]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[514]*acadoWorkspace.Dx0[4];
acadoVariables.x[108] += + acadoWorkspace.evGx[515]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[516]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[517]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[518]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[519]*acadoWorkspace.Dx0[4];
acadoVariables.x[109] += + acadoWorkspace.evGx[520]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[521]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[522]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[523]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[524]*acadoWorkspace.Dx0[4];

for (lRun1 = 0; lRun1 < 21; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multEDu( &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.x[ lRun2 * 2 ]), &(acadoVariables.x[ lRun1 * 5 + 5 ]) );
}
}
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
for (index = 0; index < 21; ++index)
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
for (index = 0; index < 21; ++index)
{
acadoVariables.x[index * 5] = acadoVariables.x[index * 5 + 5];
acadoVariables.x[index * 5 + 1] = acadoVariables.x[index * 5 + 6];
acadoVariables.x[index * 5 + 2] = acadoVariables.x[index * 5 + 7];
acadoVariables.x[index * 5 + 3] = acadoVariables.x[index * 5 + 8];
acadoVariables.x[index * 5 + 4] = acadoVariables.x[index * 5 + 9];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[105] = xEnd[0];
acadoVariables.x[106] = xEnd[1];
acadoVariables.x[107] = xEnd[2];
acadoVariables.x[108] = xEnd[3];
acadoVariables.x[109] = xEnd[4];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[105];
acadoWorkspace.state[1] = acadoVariables.x[106];
acadoWorkspace.state[2] = acadoVariables.x[107];
acadoWorkspace.state[3] = acadoVariables.x[108];
acadoWorkspace.state[4] = acadoVariables.x[109];
if (uEnd != 0)
{
acadoWorkspace.state[40] = uEnd[0];
acadoWorkspace.state[41] = uEnd[1];
}
else
{
acadoWorkspace.state[40] = acadoVariables.u[40];
acadoWorkspace.state[41] = acadoVariables.u[41];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[105] = acadoWorkspace.state[0];
acadoVariables.x[106] = acadoWorkspace.state[1];
acadoVariables.x[107] = acadoWorkspace.state[2];
acadoVariables.x[108] = acadoWorkspace.state[3];
acadoVariables.x[109] = acadoWorkspace.state[4];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[40] = uEnd[0];
acadoVariables.u[41] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41];
kkt = fabs( kkt );
for (index = 0; index < 42; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 42; ++index)
{
prd = acadoWorkspace.y[index + 42];
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

/** Row vector of size: 5 */
real_t tmpDyN[ 5 ];

for (lRun1 = 0; lRun1 < 21; ++lRun1)
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
acadoWorkspace.objValueIn[0] = acadoVariables.x[105];
acadoWorkspace.objValueIn[1] = acadoVariables.x[106];
acadoWorkspace.objValueIn[2] = acadoVariables.x[107];
acadoWorkspace.objValueIn[3] = acadoVariables.x[108];
acadoWorkspace.objValueIn[4] = acadoVariables.x[109];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 21; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 5];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 5 + 1];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 5 + 2];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 5 + 3];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 5 + 4];
objVal += + acadoWorkspace.Dy[lRun1 * 5]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 5 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 5 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 5 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 5 + 4]*tmpDy[4];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)5.0000000000000000e+00;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)5.0000000000000000e+00;
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)5.0000000000000000e+00;
tmpDyN[3] = + acadoWorkspace.DyN[3]*(real_t)5.0000000000000000e+00;
tmpDyN[4] = + acadoWorkspace.DyN[4]*(real_t)5.0000000000000000e+00;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4];

objVal *= 0.5;
return objVal;
}

