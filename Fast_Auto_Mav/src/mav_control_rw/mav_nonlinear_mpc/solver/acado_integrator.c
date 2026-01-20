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


real_t rk_dim18_swap;

/** Column vector of size: 18 */
real_t rk_dim18_bPerm[ 18 ];

/** Column vector of size: 75 */
real_t auxVar[ 75 ];

real_t rk_ttt;

/** Row vector of size: 21 */
real_t rk_xxx[ 21 ];

/** Matrix of size: 9 x 2 (row major format) */
real_t rk_kkk[ 18 ];

/** Matrix of size: 18 x 18 (row major format) */
real_t rk_A[ 324 ];

/** Column vector of size: 18 */
real_t rk_b[ 18 ];

/** Row vector of size: 18 */
int rk_dim18_perm[ 18 ];

/** Column vector of size: 9 */
real_t rk_rhsTemp[ 9 ];

/** Matrix of size: 2 x 108 (row major format) */
real_t rk_diffsTemp2[ 216 ];

/** Matrix of size: 9 x 2 (row major format) */
real_t rk_diffK[ 18 ];

/** Matrix of size: 9 x 12 (row major format) */
real_t rk_diffsPrev2[ 108 ];

/** Matrix of size: 9 x 12 (row major format) */
real_t rk_diffsNew2[ 108 ];

#pragma omp threadprivate( auxVar, rk_ttt, rk_xxx, rk_kkk, rk_diffK, rk_rhsTemp, rk_dim18_perm, rk_A, rk_b, rk_diffsPrev2, rk_diffsNew2, rk_diffsTemp2, rk_dim18_swap, rk_dim18_bPerm )

void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 9;
const real_t* od = in + 12;
/* Vector of auxiliary variables; number of elements: 31. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[3]));
a[1] = (cos(xd[5]));
a[2] = (sin(xd[4]));
a[3] = (sin(xd[3]));
a[4] = (sin(xd[5]));
a[5] = (sin(xd[4]));
a[6] = (cos(xd[4]));
a[7] = (cos(xd[5]));
a[8] = (cos(xd[4]));
a[9] = (sin(xd[5]));
a[10] = (((((a[5]*od[4])*u[2])*xd[2])+((((a[6]*a[7])*od[4])*u[2])*xd[0]))-((((a[8]*od[4])*a[9])*u[2])*xd[1]));
a[11] = (cos(xd[3]));
a[12] = (sin(xd[4]));
a[13] = (sin(xd[5]));
a[14] = (cos(xd[5]));
a[15] = (sin(xd[3]));
a[16] = (cos(xd[3]));
a[17] = (sin(xd[5]));
a[18] = (cos(xd[5]));
a[19] = (sin(xd[4]));
a[20] = (sin(xd[3]));
a[21] = (cos(xd[3]));
a[22] = (cos(xd[5]));
a[23] = (sin(xd[4]));
a[24] = (sin(xd[3]));
a[25] = (sin(xd[5]));
a[26] = (cos(xd[4]));
a[27] = (sin(xd[3]));
a[28] = (((((((a[16]*a[17])-((a[18]*a[19])*a[20]))*od[5])*u[2])*xd[0])-(((((a[21]*a[22])+((a[23]*a[24])*a[25]))*od[5])*u[2])*xd[1]))-((((a[26]*od[5])*a[27])*u[2])*xd[2]));
a[29] = (cos(xd[4]));
a[30] = (cos(xd[3]));

/* Compute outputs: */
out[0] = ((((((a[0]*a[1])*a[2])+(a[3]*a[4]))*u[2])-a[10])+od[6]);
out[1] = ((((((a[11]*a[12])*a[13])-(a[14]*a[15]))*u[2])-a[28])+od[7]);
out[2] = (((real_t)(-9.8065999999999995e+00)+((a[29]*a[30])*u[2]))+od[8]);
out[3] = (((od[1]*u[0])-xd[3])/od[0]);
out[4] = (((od[3]*u[1])-xd[4])/od[2]);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = xd[0];
out[7] = xd[1];
out[8] = xd[2];
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 9;
const real_t* od = in + 12;
/* Vector of auxiliary variables; number of elements: 75. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[4]));
a[1] = (cos(xd[5]));
a[2] = (((a[0]*a[1])*od[4])*u[2]);
a[3] = (cos(xd[4]));
a[4] = (sin(xd[5]));
a[5] = ((real_t)(0.0000000000000000e+00)-(((a[3]*od[4])*a[4])*u[2]));
a[6] = (sin(xd[4]));
a[7] = ((a[6]*od[4])*u[2]);
a[8] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[9] = (cos(xd[5]));
a[10] = (sin(xd[4]));
a[11] = (cos(xd[3]));
a[12] = (sin(xd[5]));
a[13] = (cos(xd[3]));
a[14] = (cos(xd[4]));
a[15] = (cos(xd[4]));
a[16] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[17] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[18] = (((((a[15]*od[4])*u[2])*xd[2])+((((a[16]*a[1])*od[4])*u[2])*xd[0]))-((((a[17]*od[4])*a[4])*u[2])*xd[1]));
a[19] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[20] = (sin(xd[3]));
a[21] = (cos(xd[5]));
a[22] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[23] = (cos(xd[5]));
a[24] = (((((a[0]*a[22])*od[4])*u[2])*xd[0])-((((a[3]*od[4])*a[23])*u[2])*xd[1]));
a[25] = ((((a[6]*od[4])*xd[2])+(((a[0]*a[1])*od[4])*xd[0]))-(((a[3]*od[4])*a[4])*xd[1]));
a[26] = (cos(xd[3]));
a[27] = (sin(xd[5]));
a[28] = (cos(xd[5]));
a[29] = (sin(xd[4]));
a[30] = (sin(xd[3]));
a[31] = ((((a[26]*a[27])-((a[28]*a[29])*a[30]))*od[5])*u[2]);
a[32] = (cos(xd[3]));
a[33] = (cos(xd[5]));
a[34] = (sin(xd[4]));
a[35] = (sin(xd[3]));
a[36] = (sin(xd[5]));
a[37] = ((real_t)(0.0000000000000000e+00)-((((a[32]*a[33])+((a[34]*a[35])*a[36]))*od[5])*u[2]));
a[38] = (cos(xd[4]));
a[39] = (sin(xd[3]));
a[40] = ((real_t)(0.0000000000000000e+00)-(((a[38]*od[5])*a[39])*u[2]));
a[41] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[42] = (sin(xd[4]));
a[43] = (sin(xd[5]));
a[44] = (cos(xd[5]));
a[45] = (cos(xd[3]));
a[46] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[47] = (cos(xd[3]));
a[48] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[49] = (cos(xd[3]));
a[50] = (cos(xd[3]));
a[51] = (((((((a[46]*a[27])-((a[28]*a[29])*a[47]))*od[5])*u[2])*xd[0])-(((((a[48]*a[33])+((a[34]*a[49])*a[36]))*od[5])*u[2])*xd[1]))-((((a[38]*od[5])*a[50])*u[2])*xd[2]));
a[52] = (cos(xd[3]));
a[53] = (cos(xd[4]));
a[54] = (cos(xd[4]));
a[55] = (cos(xd[4]));
a[56] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[57] = (((((((real_t)(0.0000000000000000e+00)-((a[28]*a[54])*a[30]))*od[5])*u[2])*xd[0])-(((((a[55]*a[35])*a[36])*od[5])*u[2])*xd[1]))-((((a[56]*od[5])*a[39])*u[2])*xd[2]));
a[58] = (cos(xd[5]));
a[59] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[60] = (sin(xd[3]));
a[61] = (cos(xd[5]));
a[62] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[63] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[64] = (cos(xd[5]));
a[65] = ((((((a[26]*a[61])-((a[62]*a[29])*a[30]))*od[5])*u[2])*xd[0])-(((((a[32]*a[63])+((a[34]*a[35])*a[64]))*od[5])*u[2])*xd[1]));
a[66] = ((((((a[26]*a[27])-((a[28]*a[29])*a[30]))*od[5])*xd[0])-((((a[32]*a[33])+((a[34]*a[35])*a[36]))*od[5])*xd[1]))-(((a[38]*od[5])*a[39])*xd[2]));
a[67] = (cos(xd[4]));
a[68] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[69] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[70] = (cos(xd[3]));
a[71] = ((real_t)(1.0000000000000000e+00)/od[0]);
a[72] = ((real_t)(1.0000000000000000e+00)/od[0]);
a[73] = ((real_t)(1.0000000000000000e+00)/od[2]);
a[74] = ((real_t)(1.0000000000000000e+00)/od[2]);

/* Compute outputs: */
out[0] = ((real_t)(0.0000000000000000e+00)-a[2]);
out[1] = ((real_t)(0.0000000000000000e+00)-a[5]);
out[2] = ((real_t)(0.0000000000000000e+00)-a[7]);
out[3] = ((((a[8]*a[9])*a[10])+(a[11]*a[12]))*u[2]);
out[4] = ((((a[13]*a[9])*a[14])*u[2])-a[18]);
out[5] = (((((a[13]*a[19])*a[10])+(a[20]*a[21]))*u[2])-a[24]);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = ((((a[13]*a[9])*a[10])+(a[20]*a[12]))-a[25]);
out[12] = ((real_t)(0.0000000000000000e+00)-a[31]);
out[13] = ((real_t)(0.0000000000000000e+00)-a[37]);
out[14] = ((real_t)(0.0000000000000000e+00)-a[40]);
out[15] = (((((a[41]*a[42])*a[43])-(a[44]*a[45]))*u[2])-a[51]);
out[16] = ((((a[52]*a[53])*a[43])*u[2])-a[57]);
out[17] = (((((a[52]*a[42])*a[58])-(a[59]*a[60]))*u[2])-a[65]);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = ((((a[52]*a[42])*a[43])-(a[44]*a[60]))-a[66]);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = ((a[67]*a[68])*u[2]);
out[28] = ((a[69]*a[70])*u[2]);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (a[67]*a[70]);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[71]);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (od[1]*a[72]);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[73]);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (od[3]*a[74]);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(1.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(1.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(1.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
}



void acado_solve_dim18_triangular( real_t* const A, real_t* const b )
{

b[17] = b[17]/A[323];
b[16] -= + A[305]*b[17];
b[16] = b[16]/A[304];
b[15] -= + A[287]*b[17];
b[15] -= + A[286]*b[16];
b[15] = b[15]/A[285];
b[14] -= + A[269]*b[17];
b[14] -= + A[268]*b[16];
b[14] -= + A[267]*b[15];
b[14] = b[14]/A[266];
b[13] -= + A[251]*b[17];
b[13] -= + A[250]*b[16];
b[13] -= + A[249]*b[15];
b[13] -= + A[248]*b[14];
b[13] = b[13]/A[247];
b[12] -= + A[233]*b[17];
b[12] -= + A[232]*b[16];
b[12] -= + A[231]*b[15];
b[12] -= + A[230]*b[14];
b[12] -= + A[229]*b[13];
b[12] = b[12]/A[228];
b[11] -= + A[215]*b[17];
b[11] -= + A[214]*b[16];
b[11] -= + A[213]*b[15];
b[11] -= + A[212]*b[14];
b[11] -= + A[211]*b[13];
b[11] -= + A[210]*b[12];
b[11] = b[11]/A[209];
b[10] -= + A[197]*b[17];
b[10] -= + A[196]*b[16];
b[10] -= + A[195]*b[15];
b[10] -= + A[194]*b[14];
b[10] -= + A[193]*b[13];
b[10] -= + A[192]*b[12];
b[10] -= + A[191]*b[11];
b[10] = b[10]/A[190];
b[9] -= + A[179]*b[17];
b[9] -= + A[178]*b[16];
b[9] -= + A[177]*b[15];
b[9] -= + A[176]*b[14];
b[9] -= + A[175]*b[13];
b[9] -= + A[174]*b[12];
b[9] -= + A[173]*b[11];
b[9] -= + A[172]*b[10];
b[9] = b[9]/A[171];
b[8] -= + A[161]*b[17];
b[8] -= + A[160]*b[16];
b[8] -= + A[159]*b[15];
b[8] -= + A[158]*b[14];
b[8] -= + A[157]*b[13];
b[8] -= + A[156]*b[12];
b[8] -= + A[155]*b[11];
b[8] -= + A[154]*b[10];
b[8] -= + A[153]*b[9];
b[8] = b[8]/A[152];
b[7] -= + A[143]*b[17];
b[7] -= + A[142]*b[16];
b[7] -= + A[141]*b[15];
b[7] -= + A[140]*b[14];
b[7] -= + A[139]*b[13];
b[7] -= + A[138]*b[12];
b[7] -= + A[137]*b[11];
b[7] -= + A[136]*b[10];
b[7] -= + A[135]*b[9];
b[7] -= + A[134]*b[8];
b[7] = b[7]/A[133];
b[6] -= + A[125]*b[17];
b[6] -= + A[124]*b[16];
b[6] -= + A[123]*b[15];
b[6] -= + A[122]*b[14];
b[6] -= + A[121]*b[13];
b[6] -= + A[120]*b[12];
b[6] -= + A[119]*b[11];
b[6] -= + A[118]*b[10];
b[6] -= + A[117]*b[9];
b[6] -= + A[116]*b[8];
b[6] -= + A[115]*b[7];
b[6] = b[6]/A[114];
b[5] -= + A[107]*b[17];
b[5] -= + A[106]*b[16];
b[5] -= + A[105]*b[15];
b[5] -= + A[104]*b[14];
b[5] -= + A[103]*b[13];
b[5] -= + A[102]*b[12];
b[5] -= + A[101]*b[11];
b[5] -= + A[100]*b[10];
b[5] -= + A[99]*b[9];
b[5] -= + A[98]*b[8];
b[5] -= + A[97]*b[7];
b[5] -= + A[96]*b[6];
b[5] = b[5]/A[95];
b[4] -= + A[89]*b[17];
b[4] -= + A[88]*b[16];
b[4] -= + A[87]*b[15];
b[4] -= + A[86]*b[14];
b[4] -= + A[85]*b[13];
b[4] -= + A[84]*b[12];
b[4] -= + A[83]*b[11];
b[4] -= + A[82]*b[10];
b[4] -= + A[81]*b[9];
b[4] -= + A[80]*b[8];
b[4] -= + A[79]*b[7];
b[4] -= + A[78]*b[6];
b[4] -= + A[77]*b[5];
b[4] = b[4]/A[76];
b[3] -= + A[71]*b[17];
b[3] -= + A[70]*b[16];
b[3] -= + A[69]*b[15];
b[3] -= + A[68]*b[14];
b[3] -= + A[67]*b[13];
b[3] -= + A[66]*b[12];
b[3] -= + A[65]*b[11];
b[3] -= + A[64]*b[10];
b[3] -= + A[63]*b[9];
b[3] -= + A[62]*b[8];
b[3] -= + A[61]*b[7];
b[3] -= + A[60]*b[6];
b[3] -= + A[59]*b[5];
b[3] -= + A[58]*b[4];
b[3] = b[3]/A[57];
b[2] -= + A[53]*b[17];
b[2] -= + A[52]*b[16];
b[2] -= + A[51]*b[15];
b[2] -= + A[50]*b[14];
b[2] -= + A[49]*b[13];
b[2] -= + A[48]*b[12];
b[2] -= + A[47]*b[11];
b[2] -= + A[46]*b[10];
b[2] -= + A[45]*b[9];
b[2] -= + A[44]*b[8];
b[2] -= + A[43]*b[7];
b[2] -= + A[42]*b[6];
b[2] -= + A[41]*b[5];
b[2] -= + A[40]*b[4];
b[2] -= + A[39]*b[3];
b[2] = b[2]/A[38];
b[1] -= + A[35]*b[17];
b[1] -= + A[34]*b[16];
b[1] -= + A[33]*b[15];
b[1] -= + A[32]*b[14];
b[1] -= + A[31]*b[13];
b[1] -= + A[30]*b[12];
b[1] -= + A[29]*b[11];
b[1] -= + A[28]*b[10];
b[1] -= + A[27]*b[9];
b[1] -= + A[26]*b[8];
b[1] -= + A[25]*b[7];
b[1] -= + A[24]*b[6];
b[1] -= + A[23]*b[5];
b[1] -= + A[22]*b[4];
b[1] -= + A[21]*b[3];
b[1] -= + A[20]*b[2];
b[1] = b[1]/A[19];
b[0] -= + A[17]*b[17];
b[0] -= + A[16]*b[16];
b[0] -= + A[15]*b[15];
b[0] -= + A[14]*b[14];
b[0] -= + A[13]*b[13];
b[0] -= + A[12]*b[12];
b[0] -= + A[11]*b[11];
b[0] -= + A[10]*b[10];
b[0] -= + A[9]*b[9];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim18_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 18; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (17); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*18+i]);
	for( j=(i+1); j < 18; j++ ) {
		temp = fabs(A[j*18+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 18; ++k)
{
	rk_dim18_swap = A[i*18+k];
	A[i*18+k] = A[indexMax*18+k];
	A[indexMax*18+k] = rk_dim18_swap;
}
	rk_dim18_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = rk_dim18_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*18+i];
	for( j=i+1; j < 18; j++ ) {
		A[j*18+i] = -A[j*18+i]/A[i*18+i];
		for( k=i+1; k < 18; k++ ) {
			A[j*18+k] += A[j*18+i] * A[i*18+k];
		}
		b[j] += A[j*18+i] * b[i];
	}
}
det *= A[323];
det = fabs(det);
acado_solve_dim18_triangular( A, b );
return det;
}

void acado_solve_dim18_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

rk_dim18_bPerm[0] = b[rk_perm[0]];
rk_dim18_bPerm[1] = b[rk_perm[1]];
rk_dim18_bPerm[2] = b[rk_perm[2]];
rk_dim18_bPerm[3] = b[rk_perm[3]];
rk_dim18_bPerm[4] = b[rk_perm[4]];
rk_dim18_bPerm[5] = b[rk_perm[5]];
rk_dim18_bPerm[6] = b[rk_perm[6]];
rk_dim18_bPerm[7] = b[rk_perm[7]];
rk_dim18_bPerm[8] = b[rk_perm[8]];
rk_dim18_bPerm[9] = b[rk_perm[9]];
rk_dim18_bPerm[10] = b[rk_perm[10]];
rk_dim18_bPerm[11] = b[rk_perm[11]];
rk_dim18_bPerm[12] = b[rk_perm[12]];
rk_dim18_bPerm[13] = b[rk_perm[13]];
rk_dim18_bPerm[14] = b[rk_perm[14]];
rk_dim18_bPerm[15] = b[rk_perm[15]];
rk_dim18_bPerm[16] = b[rk_perm[16]];
rk_dim18_bPerm[17] = b[rk_perm[17]];
rk_dim18_bPerm[1] += A[18]*rk_dim18_bPerm[0];

rk_dim18_bPerm[2] += A[36]*rk_dim18_bPerm[0];
rk_dim18_bPerm[2] += A[37]*rk_dim18_bPerm[1];

rk_dim18_bPerm[3] += A[54]*rk_dim18_bPerm[0];
rk_dim18_bPerm[3] += A[55]*rk_dim18_bPerm[1];
rk_dim18_bPerm[3] += A[56]*rk_dim18_bPerm[2];

rk_dim18_bPerm[4] += A[72]*rk_dim18_bPerm[0];
rk_dim18_bPerm[4] += A[73]*rk_dim18_bPerm[1];
rk_dim18_bPerm[4] += A[74]*rk_dim18_bPerm[2];
rk_dim18_bPerm[4] += A[75]*rk_dim18_bPerm[3];

rk_dim18_bPerm[5] += A[90]*rk_dim18_bPerm[0];
rk_dim18_bPerm[5] += A[91]*rk_dim18_bPerm[1];
rk_dim18_bPerm[5] += A[92]*rk_dim18_bPerm[2];
rk_dim18_bPerm[5] += A[93]*rk_dim18_bPerm[3];
rk_dim18_bPerm[5] += A[94]*rk_dim18_bPerm[4];

rk_dim18_bPerm[6] += A[108]*rk_dim18_bPerm[0];
rk_dim18_bPerm[6] += A[109]*rk_dim18_bPerm[1];
rk_dim18_bPerm[6] += A[110]*rk_dim18_bPerm[2];
rk_dim18_bPerm[6] += A[111]*rk_dim18_bPerm[3];
rk_dim18_bPerm[6] += A[112]*rk_dim18_bPerm[4];
rk_dim18_bPerm[6] += A[113]*rk_dim18_bPerm[5];

rk_dim18_bPerm[7] += A[126]*rk_dim18_bPerm[0];
rk_dim18_bPerm[7] += A[127]*rk_dim18_bPerm[1];
rk_dim18_bPerm[7] += A[128]*rk_dim18_bPerm[2];
rk_dim18_bPerm[7] += A[129]*rk_dim18_bPerm[3];
rk_dim18_bPerm[7] += A[130]*rk_dim18_bPerm[4];
rk_dim18_bPerm[7] += A[131]*rk_dim18_bPerm[5];
rk_dim18_bPerm[7] += A[132]*rk_dim18_bPerm[6];

rk_dim18_bPerm[8] += A[144]*rk_dim18_bPerm[0];
rk_dim18_bPerm[8] += A[145]*rk_dim18_bPerm[1];
rk_dim18_bPerm[8] += A[146]*rk_dim18_bPerm[2];
rk_dim18_bPerm[8] += A[147]*rk_dim18_bPerm[3];
rk_dim18_bPerm[8] += A[148]*rk_dim18_bPerm[4];
rk_dim18_bPerm[8] += A[149]*rk_dim18_bPerm[5];
rk_dim18_bPerm[8] += A[150]*rk_dim18_bPerm[6];
rk_dim18_bPerm[8] += A[151]*rk_dim18_bPerm[7];

rk_dim18_bPerm[9] += A[162]*rk_dim18_bPerm[0];
rk_dim18_bPerm[9] += A[163]*rk_dim18_bPerm[1];
rk_dim18_bPerm[9] += A[164]*rk_dim18_bPerm[2];
rk_dim18_bPerm[9] += A[165]*rk_dim18_bPerm[3];
rk_dim18_bPerm[9] += A[166]*rk_dim18_bPerm[4];
rk_dim18_bPerm[9] += A[167]*rk_dim18_bPerm[5];
rk_dim18_bPerm[9] += A[168]*rk_dim18_bPerm[6];
rk_dim18_bPerm[9] += A[169]*rk_dim18_bPerm[7];
rk_dim18_bPerm[9] += A[170]*rk_dim18_bPerm[8];

rk_dim18_bPerm[10] += A[180]*rk_dim18_bPerm[0];
rk_dim18_bPerm[10] += A[181]*rk_dim18_bPerm[1];
rk_dim18_bPerm[10] += A[182]*rk_dim18_bPerm[2];
rk_dim18_bPerm[10] += A[183]*rk_dim18_bPerm[3];
rk_dim18_bPerm[10] += A[184]*rk_dim18_bPerm[4];
rk_dim18_bPerm[10] += A[185]*rk_dim18_bPerm[5];
rk_dim18_bPerm[10] += A[186]*rk_dim18_bPerm[6];
rk_dim18_bPerm[10] += A[187]*rk_dim18_bPerm[7];
rk_dim18_bPerm[10] += A[188]*rk_dim18_bPerm[8];
rk_dim18_bPerm[10] += A[189]*rk_dim18_bPerm[9];

rk_dim18_bPerm[11] += A[198]*rk_dim18_bPerm[0];
rk_dim18_bPerm[11] += A[199]*rk_dim18_bPerm[1];
rk_dim18_bPerm[11] += A[200]*rk_dim18_bPerm[2];
rk_dim18_bPerm[11] += A[201]*rk_dim18_bPerm[3];
rk_dim18_bPerm[11] += A[202]*rk_dim18_bPerm[4];
rk_dim18_bPerm[11] += A[203]*rk_dim18_bPerm[5];
rk_dim18_bPerm[11] += A[204]*rk_dim18_bPerm[6];
rk_dim18_bPerm[11] += A[205]*rk_dim18_bPerm[7];
rk_dim18_bPerm[11] += A[206]*rk_dim18_bPerm[8];
rk_dim18_bPerm[11] += A[207]*rk_dim18_bPerm[9];
rk_dim18_bPerm[11] += A[208]*rk_dim18_bPerm[10];

rk_dim18_bPerm[12] += A[216]*rk_dim18_bPerm[0];
rk_dim18_bPerm[12] += A[217]*rk_dim18_bPerm[1];
rk_dim18_bPerm[12] += A[218]*rk_dim18_bPerm[2];
rk_dim18_bPerm[12] += A[219]*rk_dim18_bPerm[3];
rk_dim18_bPerm[12] += A[220]*rk_dim18_bPerm[4];
rk_dim18_bPerm[12] += A[221]*rk_dim18_bPerm[5];
rk_dim18_bPerm[12] += A[222]*rk_dim18_bPerm[6];
rk_dim18_bPerm[12] += A[223]*rk_dim18_bPerm[7];
rk_dim18_bPerm[12] += A[224]*rk_dim18_bPerm[8];
rk_dim18_bPerm[12] += A[225]*rk_dim18_bPerm[9];
rk_dim18_bPerm[12] += A[226]*rk_dim18_bPerm[10];
rk_dim18_bPerm[12] += A[227]*rk_dim18_bPerm[11];

rk_dim18_bPerm[13] += A[234]*rk_dim18_bPerm[0];
rk_dim18_bPerm[13] += A[235]*rk_dim18_bPerm[1];
rk_dim18_bPerm[13] += A[236]*rk_dim18_bPerm[2];
rk_dim18_bPerm[13] += A[237]*rk_dim18_bPerm[3];
rk_dim18_bPerm[13] += A[238]*rk_dim18_bPerm[4];
rk_dim18_bPerm[13] += A[239]*rk_dim18_bPerm[5];
rk_dim18_bPerm[13] += A[240]*rk_dim18_bPerm[6];
rk_dim18_bPerm[13] += A[241]*rk_dim18_bPerm[7];
rk_dim18_bPerm[13] += A[242]*rk_dim18_bPerm[8];
rk_dim18_bPerm[13] += A[243]*rk_dim18_bPerm[9];
rk_dim18_bPerm[13] += A[244]*rk_dim18_bPerm[10];
rk_dim18_bPerm[13] += A[245]*rk_dim18_bPerm[11];
rk_dim18_bPerm[13] += A[246]*rk_dim18_bPerm[12];

rk_dim18_bPerm[14] += A[252]*rk_dim18_bPerm[0];
rk_dim18_bPerm[14] += A[253]*rk_dim18_bPerm[1];
rk_dim18_bPerm[14] += A[254]*rk_dim18_bPerm[2];
rk_dim18_bPerm[14] += A[255]*rk_dim18_bPerm[3];
rk_dim18_bPerm[14] += A[256]*rk_dim18_bPerm[4];
rk_dim18_bPerm[14] += A[257]*rk_dim18_bPerm[5];
rk_dim18_bPerm[14] += A[258]*rk_dim18_bPerm[6];
rk_dim18_bPerm[14] += A[259]*rk_dim18_bPerm[7];
rk_dim18_bPerm[14] += A[260]*rk_dim18_bPerm[8];
rk_dim18_bPerm[14] += A[261]*rk_dim18_bPerm[9];
rk_dim18_bPerm[14] += A[262]*rk_dim18_bPerm[10];
rk_dim18_bPerm[14] += A[263]*rk_dim18_bPerm[11];
rk_dim18_bPerm[14] += A[264]*rk_dim18_bPerm[12];
rk_dim18_bPerm[14] += A[265]*rk_dim18_bPerm[13];

rk_dim18_bPerm[15] += A[270]*rk_dim18_bPerm[0];
rk_dim18_bPerm[15] += A[271]*rk_dim18_bPerm[1];
rk_dim18_bPerm[15] += A[272]*rk_dim18_bPerm[2];
rk_dim18_bPerm[15] += A[273]*rk_dim18_bPerm[3];
rk_dim18_bPerm[15] += A[274]*rk_dim18_bPerm[4];
rk_dim18_bPerm[15] += A[275]*rk_dim18_bPerm[5];
rk_dim18_bPerm[15] += A[276]*rk_dim18_bPerm[6];
rk_dim18_bPerm[15] += A[277]*rk_dim18_bPerm[7];
rk_dim18_bPerm[15] += A[278]*rk_dim18_bPerm[8];
rk_dim18_bPerm[15] += A[279]*rk_dim18_bPerm[9];
rk_dim18_bPerm[15] += A[280]*rk_dim18_bPerm[10];
rk_dim18_bPerm[15] += A[281]*rk_dim18_bPerm[11];
rk_dim18_bPerm[15] += A[282]*rk_dim18_bPerm[12];
rk_dim18_bPerm[15] += A[283]*rk_dim18_bPerm[13];
rk_dim18_bPerm[15] += A[284]*rk_dim18_bPerm[14];

rk_dim18_bPerm[16] += A[288]*rk_dim18_bPerm[0];
rk_dim18_bPerm[16] += A[289]*rk_dim18_bPerm[1];
rk_dim18_bPerm[16] += A[290]*rk_dim18_bPerm[2];
rk_dim18_bPerm[16] += A[291]*rk_dim18_bPerm[3];
rk_dim18_bPerm[16] += A[292]*rk_dim18_bPerm[4];
rk_dim18_bPerm[16] += A[293]*rk_dim18_bPerm[5];
rk_dim18_bPerm[16] += A[294]*rk_dim18_bPerm[6];
rk_dim18_bPerm[16] += A[295]*rk_dim18_bPerm[7];
rk_dim18_bPerm[16] += A[296]*rk_dim18_bPerm[8];
rk_dim18_bPerm[16] += A[297]*rk_dim18_bPerm[9];
rk_dim18_bPerm[16] += A[298]*rk_dim18_bPerm[10];
rk_dim18_bPerm[16] += A[299]*rk_dim18_bPerm[11];
rk_dim18_bPerm[16] += A[300]*rk_dim18_bPerm[12];
rk_dim18_bPerm[16] += A[301]*rk_dim18_bPerm[13];
rk_dim18_bPerm[16] += A[302]*rk_dim18_bPerm[14];
rk_dim18_bPerm[16] += A[303]*rk_dim18_bPerm[15];

rk_dim18_bPerm[17] += A[306]*rk_dim18_bPerm[0];
rk_dim18_bPerm[17] += A[307]*rk_dim18_bPerm[1];
rk_dim18_bPerm[17] += A[308]*rk_dim18_bPerm[2];
rk_dim18_bPerm[17] += A[309]*rk_dim18_bPerm[3];
rk_dim18_bPerm[17] += A[310]*rk_dim18_bPerm[4];
rk_dim18_bPerm[17] += A[311]*rk_dim18_bPerm[5];
rk_dim18_bPerm[17] += A[312]*rk_dim18_bPerm[6];
rk_dim18_bPerm[17] += A[313]*rk_dim18_bPerm[7];
rk_dim18_bPerm[17] += A[314]*rk_dim18_bPerm[8];
rk_dim18_bPerm[17] += A[315]*rk_dim18_bPerm[9];
rk_dim18_bPerm[17] += A[316]*rk_dim18_bPerm[10];
rk_dim18_bPerm[17] += A[317]*rk_dim18_bPerm[11];
rk_dim18_bPerm[17] += A[318]*rk_dim18_bPerm[12];
rk_dim18_bPerm[17] += A[319]*rk_dim18_bPerm[13];
rk_dim18_bPerm[17] += A[320]*rk_dim18_bPerm[14];
rk_dim18_bPerm[17] += A[321]*rk_dim18_bPerm[15];
rk_dim18_bPerm[17] += A[322]*rk_dim18_bPerm[16];


acado_solve_dim18_triangular( A, rk_dim18_bPerm );
b[0] = rk_dim18_bPerm[0];
b[1] = rk_dim18_bPerm[1];
b[2] = rk_dim18_bPerm[2];
b[3] = rk_dim18_bPerm[3];
b[4] = rk_dim18_bPerm[4];
b[5] = rk_dim18_bPerm[5];
b[6] = rk_dim18_bPerm[6];
b[7] = rk_dim18_bPerm[7];
b[8] = rk_dim18_bPerm[8];
b[9] = rk_dim18_bPerm[9];
b[10] = rk_dim18_bPerm[10];
b[11] = rk_dim18_bPerm[11];
b[12] = rk_dim18_bPerm[12];
b[13] = rk_dim18_bPerm[13];
b[14] = rk_dim18_bPerm[14];
b[15] = rk_dim18_bPerm[15];
b[16] = rk_dim18_bPerm[16];
b[17] = rk_dim18_bPerm[17];
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t acado_Ah_mat[ 4 ] = 
{ 1.2500000000000001e-02, 2.6933756729740646e-02, 
-1.9337567297406434e-03, 1.2500000000000001e-02 };


/* Fixed step size:0.05 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

rk_ttt = 0.0000000000000000e+00;
rk_xxx[9] = rk_eta[117];
rk_xxx[10] = rk_eta[118];
rk_xxx[11] = rk_eta[119];
rk_xxx[12] = rk_eta[120];
rk_xxx[13] = rk_eta[121];
rk_xxx[14] = rk_eta[122];
rk_xxx[15] = rk_eta[123];
rk_xxx[16] = rk_eta[124];
rk_xxx[17] = rk_eta[125];
rk_xxx[18] = rk_eta[126];
rk_xxx[19] = rk_eta[127];
rk_xxx[20] = rk_eta[128];

for (run = 0; run < 2; ++run)
{
if( run > 0 ) {
for (i = 0; i < 9; ++i)
{
rk_diffsPrev2[i * 12] = rk_eta[i * 9 + 9];
rk_diffsPrev2[i * 12 + 1] = rk_eta[i * 9 + 10];
rk_diffsPrev2[i * 12 + 2] = rk_eta[i * 9 + 11];
rk_diffsPrev2[i * 12 + 3] = rk_eta[i * 9 + 12];
rk_diffsPrev2[i * 12 + 4] = rk_eta[i * 9 + 13];
rk_diffsPrev2[i * 12 + 5] = rk_eta[i * 9 + 14];
rk_diffsPrev2[i * 12 + 6] = rk_eta[i * 9 + 15];
rk_diffsPrev2[i * 12 + 7] = rk_eta[i * 9 + 16];
rk_diffsPrev2[i * 12 + 8] = rk_eta[i * 9 + 17];
rk_diffsPrev2[i * 12 + 9] = rk_eta[i * 3 + 90];
rk_diffsPrev2[i * 12 + 10] = rk_eta[i * 3 + 91];
rk_diffsPrev2[i * 12 + 11] = rk_eta[i * 3 + 92];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 9; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 108 ]) );
for (j = 0; j < 9; ++j)
{
tmp_index1 = (run1 * 9) + (j);
rk_A[tmp_index1 * 18] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12)];
rk_A[tmp_index1 * 18 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 1)];
rk_A[tmp_index1 * 18 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 2)];
rk_A[tmp_index1 * 18 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 3)];
rk_A[tmp_index1 * 18 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 4)];
rk_A[tmp_index1 * 18 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 5)];
rk_A[tmp_index1 * 18 + 6] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 6)];
rk_A[tmp_index1 * 18 + 7] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 7)];
rk_A[tmp_index1 * 18 + 8] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 8)];
if( 0 == run1 ) rk_A[(tmp_index1 * 18) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 18 + 9] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12)];
rk_A[tmp_index1 * 18 + 10] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 1)];
rk_A[tmp_index1 * 18 + 11] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 2)];
rk_A[tmp_index1 * 18 + 12] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 3)];
rk_A[tmp_index1 * 18 + 13] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 4)];
rk_A[tmp_index1 * 18 + 14] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 5)];
rk_A[tmp_index1 * 18 + 15] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 6)];
rk_A[tmp_index1 * 18 + 16] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 7)];
rk_A[tmp_index1 * 18 + 17] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 8)];
if( 1 == run1 ) rk_A[(tmp_index1 * 18) + (j + 9)] -= 1.0000000000000000e+00;
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 9] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 9 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 9 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 9 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 9 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 9 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
rk_b[run1 * 9 + 6] = rk_kkk[run1 + 12] - rk_rhsTemp[6];
rk_b[run1 * 9 + 7] = rk_kkk[run1 + 14] - rk_rhsTemp[7];
rk_b[run1 * 9 + 8] = rk_kkk[run1 + 16] - rk_rhsTemp[8];
}
det = acado_solve_dim18_system( rk_A, rk_b, rk_dim18_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 9];
rk_kkk[j + 2] += rk_b[j * 9 + 1];
rk_kkk[j + 4] += rk_b[j * 9 + 2];
rk_kkk[j + 6] += rk_b[j * 9 + 3];
rk_kkk[j + 8] += rk_b[j * 9 + 4];
rk_kkk[j + 10] += rk_b[j * 9 + 5];
rk_kkk[j + 12] += rk_b[j * 9 + 6];
rk_kkk[j + 14] += rk_b[j * 9 + 7];
rk_kkk[j + 16] += rk_b[j * 9 + 8];
}
}
}
for (i = 0; i < 2; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 9; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 9] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 9 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 9 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 9 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 9 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 9 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
rk_b[run1 * 9 + 6] = rk_kkk[run1 + 12] - rk_rhsTemp[6];
rk_b[run1 * 9 + 7] = rk_kkk[run1 + 14] - rk_rhsTemp[7];
rk_b[run1 * 9 + 8] = rk_kkk[run1 + 16] - rk_rhsTemp[8];
}
acado_solve_dim18_system_reuse( rk_A, rk_b, rk_dim18_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 9];
rk_kkk[j + 2] += rk_b[j * 9 + 1];
rk_kkk[j + 4] += rk_b[j * 9 + 2];
rk_kkk[j + 6] += rk_b[j * 9 + 3];
rk_kkk[j + 8] += rk_b[j * 9 + 4];
rk_kkk[j + 10] += rk_b[j * 9 + 5];
rk_kkk[j + 12] += rk_b[j * 9 + 6];
rk_kkk[j + 14] += rk_b[j * 9 + 7];
rk_kkk[j + 16] += rk_b[j * 9 + 8];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 9; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 108 ]) );
for (j = 0; j < 9; ++j)
{
tmp_index1 = (run1 * 9) + (j);
rk_A[tmp_index1 * 18] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12)];
rk_A[tmp_index1 * 18 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 1)];
rk_A[tmp_index1 * 18 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 2)];
rk_A[tmp_index1 * 18 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 3)];
rk_A[tmp_index1 * 18 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 4)];
rk_A[tmp_index1 * 18 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 5)];
rk_A[tmp_index1 * 18 + 6] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 6)];
rk_A[tmp_index1 * 18 + 7] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 7)];
rk_A[tmp_index1 * 18 + 8] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 8)];
if( 0 == run1 ) rk_A[(tmp_index1 * 18) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 18 + 9] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12)];
rk_A[tmp_index1 * 18 + 10] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 1)];
rk_A[tmp_index1 * 18 + 11] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 2)];
rk_A[tmp_index1 * 18 + 12] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 3)];
rk_A[tmp_index1 * 18 + 13] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 4)];
rk_A[tmp_index1 * 18 + 14] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 5)];
rk_A[tmp_index1 * 18 + 15] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 6)];
rk_A[tmp_index1 * 18 + 16] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 7)];
rk_A[tmp_index1 * 18 + 17] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 108) + (j * 12 + 8)];
if( 1 == run1 ) rk_A[(tmp_index1 * 18) + (j + 9)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 9; ++run1)
{
for (i = 0; i < 2; ++i)
{
rk_b[i * 9] = - rk_diffsTemp2[(i * 108) + (run1)];
rk_b[i * 9 + 1] = - rk_diffsTemp2[(i * 108) + (run1 + 12)];
rk_b[i * 9 + 2] = - rk_diffsTemp2[(i * 108) + (run1 + 24)];
rk_b[i * 9 + 3] = - rk_diffsTemp2[(i * 108) + (run1 + 36)];
rk_b[i * 9 + 4] = - rk_diffsTemp2[(i * 108) + (run1 + 48)];
rk_b[i * 9 + 5] = - rk_diffsTemp2[(i * 108) + (run1 + 60)];
rk_b[i * 9 + 6] = - rk_diffsTemp2[(i * 108) + (run1 + 72)];
rk_b[i * 9 + 7] = - rk_diffsTemp2[(i * 108) + (run1 + 84)];
rk_b[i * 9 + 8] = - rk_diffsTemp2[(i * 108) + (run1 + 96)];
}
if( 0 == run1 ) {
det = acado_solve_dim18_system( rk_A, rk_b, rk_dim18_perm );
}
 else {
acado_solve_dim18_system_reuse( rk_A, rk_b, rk_dim18_perm );
}
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 9];
rk_diffK[i + 2] = rk_b[i * 9 + 1];
rk_diffK[i + 4] = rk_b[i * 9 + 2];
rk_diffK[i + 6] = rk_b[i * 9 + 3];
rk_diffK[i + 8] = rk_b[i * 9 + 4];
rk_diffK[i + 10] = rk_b[i * 9 + 5];
rk_diffK[i + 12] = rk_b[i * 9 + 6];
rk_diffK[i + 14] = rk_b[i * 9 + 7];
rk_diffK[i + 16] = rk_b[i * 9 + 8];
}
for (i = 0; i < 9; ++i)
{
rk_diffsNew2[(i * 12) + (run1)] = (i == run1-0);
rk_diffsNew2[(i * 12) + (run1)] += + rk_diffK[i * 2]*(real_t)2.5000000000000001e-02 + rk_diffK[i * 2 + 1]*(real_t)2.5000000000000001e-02;
}
}
for (run1 = 0; run1 < 3; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 9; ++j)
{
tmp_index1 = (i * 9) + (j);
tmp_index2 = (run1) + (j * 12);
rk_b[tmp_index1] = - rk_diffsTemp2[(i * 108) + (tmp_index2 + 9)];
}
}
acado_solve_dim18_system_reuse( rk_A, rk_b, rk_dim18_perm );
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 9];
rk_diffK[i + 2] = rk_b[i * 9 + 1];
rk_diffK[i + 4] = rk_b[i * 9 + 2];
rk_diffK[i + 6] = rk_b[i * 9 + 3];
rk_diffK[i + 8] = rk_b[i * 9 + 4];
rk_diffK[i + 10] = rk_b[i * 9 + 5];
rk_diffK[i + 12] = rk_b[i * 9 + 6];
rk_diffK[i + 14] = rk_b[i * 9 + 7];
rk_diffK[i + 16] = rk_b[i * 9 + 8];
}
for (i = 0; i < 9; ++i)
{
rk_diffsNew2[(i * 12) + (run1 + 9)] = + rk_diffK[i * 2]*(real_t)2.5000000000000001e-02 + rk_diffK[i * 2 + 1]*(real_t)2.5000000000000001e-02;
}
}
rk_eta[0] += + rk_kkk[0]*(real_t)2.5000000000000001e-02 + rk_kkk[1]*(real_t)2.5000000000000001e-02;
rk_eta[1] += + rk_kkk[2]*(real_t)2.5000000000000001e-02 + rk_kkk[3]*(real_t)2.5000000000000001e-02;
rk_eta[2] += + rk_kkk[4]*(real_t)2.5000000000000001e-02 + rk_kkk[5]*(real_t)2.5000000000000001e-02;
rk_eta[3] += + rk_kkk[6]*(real_t)2.5000000000000001e-02 + rk_kkk[7]*(real_t)2.5000000000000001e-02;
rk_eta[4] += + rk_kkk[8]*(real_t)2.5000000000000001e-02 + rk_kkk[9]*(real_t)2.5000000000000001e-02;
rk_eta[5] += + rk_kkk[10]*(real_t)2.5000000000000001e-02 + rk_kkk[11]*(real_t)2.5000000000000001e-02;
rk_eta[6] += + rk_kkk[12]*(real_t)2.5000000000000001e-02 + rk_kkk[13]*(real_t)2.5000000000000001e-02;
rk_eta[7] += + rk_kkk[14]*(real_t)2.5000000000000001e-02 + rk_kkk[15]*(real_t)2.5000000000000001e-02;
rk_eta[8] += + rk_kkk[16]*(real_t)2.5000000000000001e-02 + rk_kkk[17]*(real_t)2.5000000000000001e-02;
if( run == 0 ) {
for (i = 0; i < 9; ++i)
{
for (j = 0; j < 9; ++j)
{
tmp_index2 = (j) + (i * 9);
rk_eta[tmp_index2 + 9] = rk_diffsNew2[(i * 12) + (j)];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 90] = rk_diffsNew2[(i * 12) + (j + 9)];
}
}
}
else {
for (i = 0; i < 9; ++i)
{
for (j = 0; j < 9; ++j)
{
tmp_index2 = (j) + (i * 9);
rk_eta[tmp_index2 + 9] = + rk_diffsNew2[i * 12]*rk_diffsPrev2[j];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 1]*rk_diffsPrev2[j + 12];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 2]*rk_diffsPrev2[j + 24];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 3]*rk_diffsPrev2[j + 36];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 4]*rk_diffsPrev2[j + 48];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 5]*rk_diffsPrev2[j + 60];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 6]*rk_diffsPrev2[j + 72];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 7]*rk_diffsPrev2[j + 84];
rk_eta[tmp_index2 + 9] += + rk_diffsNew2[i * 12 + 8]*rk_diffsPrev2[j + 96];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 90] = rk_diffsNew2[(i * 12) + (j + 9)];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12]*rk_diffsPrev2[j + 9];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 1]*rk_diffsPrev2[j + 21];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 2]*rk_diffsPrev2[j + 33];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 3]*rk_diffsPrev2[j + 45];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 4]*rk_diffsPrev2[j + 57];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 5]*rk_diffsPrev2[j + 69];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 6]*rk_diffsPrev2[j + 81];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 7]*rk_diffsPrev2[j + 93];
rk_eta[tmp_index2 + 90] += + rk_diffsNew2[i * 12 + 8]*rk_diffsPrev2[j + 105];
}
}
}
resetIntegrator = 0;
rk_ttt += 5.0000000000000000e-01;
}
for (i = 0; i < 9; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



