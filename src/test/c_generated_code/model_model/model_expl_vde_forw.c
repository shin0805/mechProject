/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) model_expl_vde_forw_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[13] = {9, 1, 0, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8};
static const casadi_int casadi_s1[93] = {9, 9, 0, 9, 18, 27, 36, 45, 54, 63, 72, 81, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8};
static const casadi_int casadi_s2[63] = {9, 6, 0, 9, 18, 27, 36, 45, 54, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8};
static const casadi_int casadi_s3[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s4[3] = {0, 0, 0};

/* model_expl_vde_forw:(i0[9],i1[9x9],i2[9x6],i3[6],i4[])->(o0[9],o1[9x9],o2[9x6]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a7, a8, a9;
  a0=arg[0]? arg[0][1] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a0=-1.0000000000000000e-03;
  a1=arg[3]? arg[3][0] : 0;
  a2=(a0*a1);
  a3=-2.0000000000000000e-03;
  a4=1.0700000000000000e-01;
  a5=arg[0]? arg[0][3] : 0;
  a6=sin(a5);
  a6=(a4*a6);
  a6=(a3+a6);
  a7=casadi_sq(a6);
  a8=4.5999999999999999e-02;
  a9=arg[0]? arg[0][4] : 0;
  a10=sin(a9);
  a10=(a4*a10);
  a8=(a8+a10);
  a10=casadi_sq(a8);
  a7=(a7+a10);
  a7=sqrt(a7);
  a2=(a2/a7);
  a10=arg[3]? arg[3][2] : 0;
  a11=(a0*a10);
  a12=arg[0]? arg[0][5] : 0;
  a13=sin(a12);
  a13=(a4*a13);
  a3=(a3+a13);
  a13=casadi_sq(a3);
  a14=-4.5999999999999999e-02;
  a15=arg[0]? arg[0][6] : 0;
  a16=sin(a15);
  a16=(a4*a16);
  a16=(a14+a16);
  a17=casadi_sq(a16);
  a13=(a13+a17);
  a13=sqrt(a13);
  a11=(a11/a13);
  a17=(a2+a11);
  a18=arg[3]? arg[3][4] : 0;
  a19=(a0*a18);
  a20=8.4000000000000005e-02;
  a21=arg[0]? arg[0][7] : 0;
  a22=sin(a21);
  a22=(a4*a22);
  a20=(a20+a22);
  a22=casadi_sq(a20);
  a23=arg[0]? arg[0][8] : 0;
  a24=sin(a23);
  a24=(a4*a24);
  a14=(a14+a24);
  a24=casadi_sq(a14);
  a22=(a22+a24);
  a22=sqrt(a22);
  a19=(a19/a22);
  a17=(a17+a19);
  a24=2.9999999999999999e-01;
  a17=(a17/a24);
  if (res[0]!=0) res[0][1]=a17;
  a17=arg[3]? arg[3][1] : 0;
  a24=(a0*a17);
  a24=(a24/a7);
  a25=(a24*a6);
  a26=(a2*a8);
  a25=(a25-a26);
  a26=arg[3]? arg[3][3] : 0;
  a27=(a0*a26);
  a27=(a27/a13);
  a28=(a27*a3);
  a25=(a25+a28);
  a28=(a11*a16);
  a25=(a25-a28);
  a28=arg[3]? arg[3][5] : 0;
  a29=(a0*a28);
  a29=(a29/a22);
  a30=(a29*a20);
  a25=(a25+a30);
  a30=(a19*a14);
  a25=(a25-a30);
  a30=4.0000000000000001e-03;
  a25=(a25/a30);
  if (res[0]!=0) res[0][2]=a25;
  if (res[0]!=0) res[0][3]=a1;
  if (res[0]!=0) res[0][4]=a17;
  if (res[0]!=0) res[0][5]=a10;
  if (res[0]!=0) res[0][6]=a26;
  if (res[0]!=0) res[0][7]=a18;
  if (res[0]!=0) res[0][8]=a28;
  a28=arg[1]? arg[1][1] : 0;
  if (res[1]!=0) res[1][0]=a28;
  a28=3.3333333333333335e+00;
  a18=(a2/a7);
  a26=(a6+a6);
  a10=cos(a5);
  a17=arg[1]? arg[1][3] : 0;
  a17=(a10*a17);
  a17=(a4*a17);
  a1=(a26*a17);
  a25=(a8+a8);
  a30=cos(a9);
  a31=arg[1]? arg[1][4] : 0;
  a31=(a30*a31);
  a31=(a4*a31);
  a32=(a25*a31);
  a1=(a1+a32);
  a32=(a7+a7);
  a1=(a1/a32);
  a33=(a18*a1);
  a34=(a11/a13);
  a35=(a3+a3);
  a36=cos(a12);
  a37=arg[1]? arg[1][5] : 0;
  a37=(a36*a37);
  a37=(a4*a37);
  a38=(a35*a37);
  a39=(a16+a16);
  a40=cos(a15);
  a41=arg[1]? arg[1][6] : 0;
  a41=(a40*a41);
  a41=(a4*a41);
  a42=(a39*a41);
  a38=(a38+a42);
  a42=(a13+a13);
  a38=(a38/a42);
  a43=(a34*a38);
  a44=(a33+a43);
  a45=(a19/a22);
  a46=(a20+a20);
  a47=cos(a21);
  a48=arg[1]? arg[1][7] : 0;
  a48=(a47*a48);
  a48=(a4*a48);
  a49=(a46*a48);
  a50=(a14+a14);
  a51=cos(a23);
  a52=arg[1]? arg[1][8] : 0;
  a52=(a51*a52);
  a52=(a4*a52);
  a53=(a50*a52);
  a49=(a49+a53);
  a53=(a22+a22);
  a49=(a49/a53);
  a54=(a45*a49);
  a44=(a44+a54);
  a44=(a28*a44);
  a44=(-a44);
  if (res[1]!=0) res[1][1]=a44;
  a44=250.;
  a17=(a24*a17);
  a55=(a24/a7);
  a1=(a55*a1);
  a1=(a6*a1);
  a17=(a17-a1);
  a31=(a2*a31);
  a33=(a8*a33);
  a31=(a31-a33);
  a17=(a17-a31);
  a37=(a27*a37);
  a31=(a27/a13);
  a38=(a31*a38);
  a38=(a3*a38);
  a37=(a37-a38);
  a17=(a17+a37);
  a41=(a11*a41);
  a43=(a16*a43);
  a41=(a41-a43);
  a17=(a17-a41);
  a48=(a29*a48);
  a41=(a29/a22);
  a49=(a41*a49);
  a49=(a20*a49);
  a48=(a48-a49);
  a17=(a17+a48);
  a52=(a19*a52);
  a54=(a14*a54);
  a52=(a52-a54);
  a17=(a17-a52);
  a17=(a44*a17);
  if (res[1]!=0) res[1][2]=a17;
  a17=0.;
  if (res[1]!=0) res[1][3]=a17;
  if (res[1]!=0) res[1][4]=a17;
  if (res[1]!=0) res[1][5]=a17;
  if (res[1]!=0) res[1][6]=a17;
  if (res[1]!=0) res[1][7]=a17;
  if (res[1]!=0) res[1][8]=a17;
  a52=arg[1]? arg[1][10] : 0;
  if (res[1]!=0) res[1][9]=a52;
  a52=arg[1]? arg[1][12] : 0;
  a52=(a10*a52);
  a52=(a4*a52);
  a54=(a26*a52);
  a48=arg[1]? arg[1][13] : 0;
  a48=(a30*a48);
  a48=(a4*a48);
  a49=(a25*a48);
  a54=(a54+a49);
  a54=(a54/a32);
  a49=(a18*a54);
  a43=arg[1]? arg[1][14] : 0;
  a43=(a36*a43);
  a43=(a4*a43);
  a37=(a35*a43);
  a38=arg[1]? arg[1][15] : 0;
  a38=(a40*a38);
  a38=(a4*a38);
  a33=(a39*a38);
  a37=(a37+a33);
  a37=(a37/a42);
  a33=(a34*a37);
  a1=(a49+a33);
  a56=arg[1]? arg[1][16] : 0;
  a56=(a47*a56);
  a56=(a4*a56);
  a57=(a46*a56);
  a58=arg[1]? arg[1][17] : 0;
  a58=(a51*a58);
  a58=(a4*a58);
  a59=(a50*a58);
  a57=(a57+a59);
  a57=(a57/a53);
  a59=(a45*a57);
  a1=(a1+a59);
  a1=(a28*a1);
  a1=(-a1);
  if (res[1]!=0) res[1][10]=a1;
  a52=(a24*a52);
  a54=(a55*a54);
  a54=(a6*a54);
  a52=(a52-a54);
  a48=(a2*a48);
  a49=(a8*a49);
  a48=(a48-a49);
  a52=(a52-a48);
  a43=(a27*a43);
  a37=(a31*a37);
  a37=(a3*a37);
  a43=(a43-a37);
  a52=(a52+a43);
  a38=(a11*a38);
  a33=(a16*a33);
  a38=(a38-a33);
  a52=(a52-a38);
  a56=(a29*a56);
  a57=(a41*a57);
  a57=(a20*a57);
  a56=(a56-a57);
  a52=(a52+a56);
  a58=(a19*a58);
  a59=(a14*a59);
  a58=(a58-a59);
  a52=(a52-a58);
  a52=(a44*a52);
  if (res[1]!=0) res[1][11]=a52;
  if (res[1]!=0) res[1][12]=a17;
  if (res[1]!=0) res[1][13]=a17;
  if (res[1]!=0) res[1][14]=a17;
  if (res[1]!=0) res[1][15]=a17;
  if (res[1]!=0) res[1][16]=a17;
  if (res[1]!=0) res[1][17]=a17;
  a52=arg[1]? arg[1][19] : 0;
  if (res[1]!=0) res[1][18]=a52;
  a52=arg[1]? arg[1][21] : 0;
  a52=(a10*a52);
  a52=(a4*a52);
  a58=(a26*a52);
  a59=arg[1]? arg[1][22] : 0;
  a59=(a30*a59);
  a59=(a4*a59);
  a56=(a25*a59);
  a58=(a58+a56);
  a58=(a58/a32);
  a56=(a18*a58);
  a57=arg[1]? arg[1][23] : 0;
  a57=(a36*a57);
  a57=(a4*a57);
  a38=(a35*a57);
  a33=arg[1]? arg[1][24] : 0;
  a33=(a40*a33);
  a33=(a4*a33);
  a43=(a39*a33);
  a38=(a38+a43);
  a38=(a38/a42);
  a43=(a34*a38);
  a37=(a56+a43);
  a48=arg[1]? arg[1][25] : 0;
  a48=(a47*a48);
  a48=(a4*a48);
  a49=(a46*a48);
  a54=arg[1]? arg[1][26] : 0;
  a54=(a51*a54);
  a54=(a4*a54);
  a1=(a50*a54);
  a49=(a49+a1);
  a49=(a49/a53);
  a1=(a45*a49);
  a37=(a37+a1);
  a37=(a28*a37);
  a37=(-a37);
  if (res[1]!=0) res[1][19]=a37;
  a52=(a24*a52);
  a58=(a55*a58);
  a58=(a6*a58);
  a52=(a52-a58);
  a59=(a2*a59);
  a56=(a8*a56);
  a59=(a59-a56);
  a52=(a52-a59);
  a57=(a27*a57);
  a38=(a31*a38);
  a38=(a3*a38);
  a57=(a57-a38);
  a52=(a52+a57);
  a33=(a11*a33);
  a43=(a16*a43);
  a33=(a33-a43);
  a52=(a52-a33);
  a48=(a29*a48);
  a49=(a41*a49);
  a49=(a20*a49);
  a48=(a48-a49);
  a52=(a52+a48);
  a54=(a19*a54);
  a1=(a14*a1);
  a54=(a54-a1);
  a52=(a52-a54);
  a52=(a44*a52);
  if (res[1]!=0) res[1][20]=a52;
  if (res[1]!=0) res[1][21]=a17;
  if (res[1]!=0) res[1][22]=a17;
  if (res[1]!=0) res[1][23]=a17;
  if (res[1]!=0) res[1][24]=a17;
  if (res[1]!=0) res[1][25]=a17;
  if (res[1]!=0) res[1][26]=a17;
  a52=arg[1]? arg[1][28] : 0;
  if (res[1]!=0) res[1][27]=a52;
  a52=arg[1]? arg[1][30] : 0;
  a52=(a10*a52);
  a52=(a4*a52);
  a54=(a26*a52);
  a1=arg[1]? arg[1][31] : 0;
  a1=(a30*a1);
  a1=(a4*a1);
  a48=(a25*a1);
  a54=(a54+a48);
  a54=(a54/a32);
  a48=(a18*a54);
  a49=arg[1]? arg[1][32] : 0;
  a49=(a36*a49);
  a49=(a4*a49);
  a33=(a35*a49);
  a43=arg[1]? arg[1][33] : 0;
  a43=(a40*a43);
  a43=(a4*a43);
  a57=(a39*a43);
  a33=(a33+a57);
  a33=(a33/a42);
  a57=(a34*a33);
  a38=(a48+a57);
  a59=arg[1]? arg[1][34] : 0;
  a59=(a47*a59);
  a59=(a4*a59);
  a56=(a46*a59);
  a58=arg[1]? arg[1][35] : 0;
  a58=(a51*a58);
  a58=(a4*a58);
  a37=(a50*a58);
  a56=(a56+a37);
  a56=(a56/a53);
  a37=(a45*a56);
  a38=(a38+a37);
  a38=(a28*a38);
  a38=(-a38);
  if (res[1]!=0) res[1][28]=a38;
  a52=(a24*a52);
  a54=(a55*a54);
  a54=(a6*a54);
  a52=(a52-a54);
  a1=(a2*a1);
  a48=(a8*a48);
  a1=(a1-a48);
  a52=(a52-a1);
  a49=(a27*a49);
  a33=(a31*a33);
  a33=(a3*a33);
  a49=(a49-a33);
  a52=(a52+a49);
  a43=(a11*a43);
  a57=(a16*a57);
  a43=(a43-a57);
  a52=(a52-a43);
  a59=(a29*a59);
  a56=(a41*a56);
  a56=(a20*a56);
  a59=(a59-a56);
  a52=(a52+a59);
  a58=(a19*a58);
  a37=(a14*a37);
  a58=(a58-a37);
  a52=(a52-a58);
  a52=(a44*a52);
  if (res[1]!=0) res[1][29]=a52;
  if (res[1]!=0) res[1][30]=a17;
  if (res[1]!=0) res[1][31]=a17;
  if (res[1]!=0) res[1][32]=a17;
  if (res[1]!=0) res[1][33]=a17;
  if (res[1]!=0) res[1][34]=a17;
  if (res[1]!=0) res[1][35]=a17;
  a52=arg[1]? arg[1][37] : 0;
  if (res[1]!=0) res[1][36]=a52;
  a52=arg[1]? arg[1][39] : 0;
  a52=(a10*a52);
  a52=(a4*a52);
  a58=(a26*a52);
  a37=arg[1]? arg[1][40] : 0;
  a37=(a30*a37);
  a37=(a4*a37);
  a59=(a25*a37);
  a58=(a58+a59);
  a58=(a58/a32);
  a59=(a18*a58);
  a56=arg[1]? arg[1][41] : 0;
  a56=(a36*a56);
  a56=(a4*a56);
  a43=(a35*a56);
  a57=arg[1]? arg[1][42] : 0;
  a57=(a40*a57);
  a57=(a4*a57);
  a49=(a39*a57);
  a43=(a43+a49);
  a43=(a43/a42);
  a49=(a34*a43);
  a33=(a59+a49);
  a1=arg[1]? arg[1][43] : 0;
  a1=(a47*a1);
  a1=(a4*a1);
  a48=(a46*a1);
  a54=arg[1]? arg[1][44] : 0;
  a54=(a51*a54);
  a54=(a4*a54);
  a38=(a50*a54);
  a48=(a48+a38);
  a48=(a48/a53);
  a38=(a45*a48);
  a33=(a33+a38);
  a33=(a28*a33);
  a33=(-a33);
  if (res[1]!=0) res[1][37]=a33;
  a52=(a24*a52);
  a58=(a55*a58);
  a58=(a6*a58);
  a52=(a52-a58);
  a37=(a2*a37);
  a59=(a8*a59);
  a37=(a37-a59);
  a52=(a52-a37);
  a56=(a27*a56);
  a43=(a31*a43);
  a43=(a3*a43);
  a56=(a56-a43);
  a52=(a52+a56);
  a57=(a11*a57);
  a49=(a16*a49);
  a57=(a57-a49);
  a52=(a52-a57);
  a1=(a29*a1);
  a48=(a41*a48);
  a48=(a20*a48);
  a1=(a1-a48);
  a52=(a52+a1);
  a54=(a19*a54);
  a38=(a14*a38);
  a54=(a54-a38);
  a52=(a52-a54);
  a52=(a44*a52);
  if (res[1]!=0) res[1][38]=a52;
  if (res[1]!=0) res[1][39]=a17;
  if (res[1]!=0) res[1][40]=a17;
  if (res[1]!=0) res[1][41]=a17;
  if (res[1]!=0) res[1][42]=a17;
  if (res[1]!=0) res[1][43]=a17;
  if (res[1]!=0) res[1][44]=a17;
  a52=arg[1]? arg[1][46] : 0;
  if (res[1]!=0) res[1][45]=a52;
  a52=arg[1]? arg[1][48] : 0;
  a52=(a10*a52);
  a52=(a4*a52);
  a54=(a26*a52);
  a38=arg[1]? arg[1][49] : 0;
  a38=(a30*a38);
  a38=(a4*a38);
  a1=(a25*a38);
  a54=(a54+a1);
  a54=(a54/a32);
  a1=(a18*a54);
  a48=arg[1]? arg[1][50] : 0;
  a48=(a36*a48);
  a48=(a4*a48);
  a57=(a35*a48);
  a49=arg[1]? arg[1][51] : 0;
  a49=(a40*a49);
  a49=(a4*a49);
  a56=(a39*a49);
  a57=(a57+a56);
  a57=(a57/a42);
  a56=(a34*a57);
  a43=(a1+a56);
  a37=arg[1]? arg[1][52] : 0;
  a37=(a47*a37);
  a37=(a4*a37);
  a59=(a46*a37);
  a58=arg[1]? arg[1][53] : 0;
  a58=(a51*a58);
  a58=(a4*a58);
  a33=(a50*a58);
  a59=(a59+a33);
  a59=(a59/a53);
  a33=(a45*a59);
  a43=(a43+a33);
  a43=(a28*a43);
  a43=(-a43);
  if (res[1]!=0) res[1][46]=a43;
  a52=(a24*a52);
  a54=(a55*a54);
  a54=(a6*a54);
  a52=(a52-a54);
  a38=(a2*a38);
  a1=(a8*a1);
  a38=(a38-a1);
  a52=(a52-a38);
  a48=(a27*a48);
  a57=(a31*a57);
  a57=(a3*a57);
  a48=(a48-a57);
  a52=(a52+a48);
  a49=(a11*a49);
  a56=(a16*a56);
  a49=(a49-a56);
  a52=(a52-a49);
  a37=(a29*a37);
  a59=(a41*a59);
  a59=(a20*a59);
  a37=(a37-a59);
  a52=(a52+a37);
  a58=(a19*a58);
  a33=(a14*a33);
  a58=(a58-a33);
  a52=(a52-a58);
  a52=(a44*a52);
  if (res[1]!=0) res[1][47]=a52;
  if (res[1]!=0) res[1][48]=a17;
  if (res[1]!=0) res[1][49]=a17;
  if (res[1]!=0) res[1][50]=a17;
  if (res[1]!=0) res[1][51]=a17;
  if (res[1]!=0) res[1][52]=a17;
  if (res[1]!=0) res[1][53]=a17;
  a52=arg[1]? arg[1][55] : 0;
  if (res[1]!=0) res[1][54]=a52;
  a52=arg[1]? arg[1][57] : 0;
  a52=(a10*a52);
  a52=(a4*a52);
  a58=(a26*a52);
  a33=arg[1]? arg[1][58] : 0;
  a33=(a30*a33);
  a33=(a4*a33);
  a37=(a25*a33);
  a58=(a58+a37);
  a58=(a58/a32);
  a37=(a18*a58);
  a59=arg[1]? arg[1][59] : 0;
  a59=(a36*a59);
  a59=(a4*a59);
  a49=(a35*a59);
  a56=arg[1]? arg[1][60] : 0;
  a56=(a40*a56);
  a56=(a4*a56);
  a48=(a39*a56);
  a49=(a49+a48);
  a49=(a49/a42);
  a48=(a34*a49);
  a57=(a37+a48);
  a38=arg[1]? arg[1][61] : 0;
  a38=(a47*a38);
  a38=(a4*a38);
  a1=(a46*a38);
  a54=arg[1]? arg[1][62] : 0;
  a54=(a51*a54);
  a54=(a4*a54);
  a43=(a50*a54);
  a1=(a1+a43);
  a1=(a1/a53);
  a43=(a45*a1);
  a57=(a57+a43);
  a57=(a28*a57);
  a57=(-a57);
  if (res[1]!=0) res[1][55]=a57;
  a52=(a24*a52);
  a58=(a55*a58);
  a58=(a6*a58);
  a52=(a52-a58);
  a33=(a2*a33);
  a37=(a8*a37);
  a33=(a33-a37);
  a52=(a52-a33);
  a59=(a27*a59);
  a49=(a31*a49);
  a49=(a3*a49);
  a59=(a59-a49);
  a52=(a52+a59);
  a56=(a11*a56);
  a48=(a16*a48);
  a56=(a56-a48);
  a52=(a52-a56);
  a38=(a29*a38);
  a1=(a41*a1);
  a1=(a20*a1);
  a38=(a38-a1);
  a52=(a52+a38);
  a54=(a19*a54);
  a43=(a14*a43);
  a54=(a54-a43);
  a52=(a52-a54);
  a52=(a44*a52);
  if (res[1]!=0) res[1][56]=a52;
  if (res[1]!=0) res[1][57]=a17;
  if (res[1]!=0) res[1][58]=a17;
  if (res[1]!=0) res[1][59]=a17;
  if (res[1]!=0) res[1][60]=a17;
  if (res[1]!=0) res[1][61]=a17;
  if (res[1]!=0) res[1][62]=a17;
  a52=arg[1]? arg[1][64] : 0;
  if (res[1]!=0) res[1][63]=a52;
  a52=arg[1]? arg[1][66] : 0;
  a52=(a10*a52);
  a52=(a4*a52);
  a54=(a26*a52);
  a43=arg[1]? arg[1][67] : 0;
  a43=(a30*a43);
  a43=(a4*a43);
  a38=(a25*a43);
  a54=(a54+a38);
  a54=(a54/a32);
  a38=(a18*a54);
  a1=arg[1]? arg[1][68] : 0;
  a1=(a36*a1);
  a1=(a4*a1);
  a56=(a35*a1);
  a48=arg[1]? arg[1][69] : 0;
  a48=(a40*a48);
  a48=(a4*a48);
  a59=(a39*a48);
  a56=(a56+a59);
  a56=(a56/a42);
  a59=(a34*a56);
  a49=(a38+a59);
  a33=arg[1]? arg[1][70] : 0;
  a33=(a47*a33);
  a33=(a4*a33);
  a37=(a46*a33);
  a58=arg[1]? arg[1][71] : 0;
  a58=(a51*a58);
  a58=(a4*a58);
  a57=(a50*a58);
  a37=(a37+a57);
  a37=(a37/a53);
  a57=(a45*a37);
  a49=(a49+a57);
  a49=(a28*a49);
  a49=(-a49);
  if (res[1]!=0) res[1][64]=a49;
  a52=(a24*a52);
  a54=(a55*a54);
  a54=(a6*a54);
  a52=(a52-a54);
  a43=(a2*a43);
  a38=(a8*a38);
  a43=(a43-a38);
  a52=(a52-a43);
  a1=(a27*a1);
  a56=(a31*a56);
  a56=(a3*a56);
  a1=(a1-a56);
  a52=(a52+a1);
  a48=(a11*a48);
  a59=(a16*a59);
  a48=(a48-a59);
  a52=(a52-a48);
  a33=(a29*a33);
  a37=(a41*a37);
  a37=(a20*a37);
  a33=(a33-a37);
  a52=(a52+a33);
  a58=(a19*a58);
  a57=(a14*a57);
  a58=(a58-a57);
  a52=(a52-a58);
  a52=(a44*a52);
  if (res[1]!=0) res[1][65]=a52;
  if (res[1]!=0) res[1][66]=a17;
  if (res[1]!=0) res[1][67]=a17;
  if (res[1]!=0) res[1][68]=a17;
  if (res[1]!=0) res[1][69]=a17;
  if (res[1]!=0) res[1][70]=a17;
  if (res[1]!=0) res[1][71]=a17;
  a52=arg[1]? arg[1][73] : 0;
  if (res[1]!=0) res[1][72]=a52;
  a52=arg[1]? arg[1][75] : 0;
  a10=(a10*a52);
  a10=(a4*a10);
  a26=(a26*a10);
  a52=arg[1]? arg[1][76] : 0;
  a30=(a30*a52);
  a30=(a4*a30);
  a25=(a25*a30);
  a26=(a26+a25);
  a26=(a26/a32);
  a18=(a18*a26);
  a32=arg[1]? arg[1][77] : 0;
  a36=(a36*a32);
  a36=(a4*a36);
  a35=(a35*a36);
  a32=arg[1]? arg[1][78] : 0;
  a40=(a40*a32);
  a40=(a4*a40);
  a39=(a39*a40);
  a35=(a35+a39);
  a35=(a35/a42);
  a34=(a34*a35);
  a42=(a18+a34);
  a39=arg[1]? arg[1][79] : 0;
  a47=(a47*a39);
  a47=(a4*a47);
  a46=(a46*a47);
  a39=arg[1]? arg[1][80] : 0;
  a51=(a51*a39);
  a51=(a4*a51);
  a50=(a50*a51);
  a46=(a46+a50);
  a46=(a46/a53);
  a45=(a45*a46);
  a42=(a42+a45);
  a42=(a28*a42);
  a42=(-a42);
  if (res[1]!=0) res[1][73]=a42;
  a10=(a24*a10);
  a55=(a55*a26);
  a55=(a6*a55);
  a10=(a10-a55);
  a30=(a2*a30);
  a18=(a8*a18);
  a30=(a30-a18);
  a10=(a10-a30);
  a36=(a27*a36);
  a31=(a31*a35);
  a31=(a3*a31);
  a36=(a36-a31);
  a10=(a10+a36);
  a40=(a11*a40);
  a34=(a16*a34);
  a40=(a40-a34);
  a10=(a10-a40);
  a47=(a29*a47);
  a41=(a41*a46);
  a41=(a20*a41);
  a47=(a47-a41);
  a10=(a10+a47);
  a51=(a19*a51);
  a45=(a14*a45);
  a51=(a51-a45);
  a10=(a10-a51);
  a10=(a44*a10);
  if (res[1]!=0) res[1][74]=a10;
  if (res[1]!=0) res[1][75]=a17;
  if (res[1]!=0) res[1][76]=a17;
  if (res[1]!=0) res[1][77]=a17;
  if (res[1]!=0) res[1][78]=a17;
  if (res[1]!=0) res[1][79]=a17;
  if (res[1]!=0) res[1][80]=a17;
  a10=arg[2]? arg[2][1] : 0;
  if (res[2]!=0) res[2][0]=a10;
  a10=(a0/a7);
  a51=(a28*a10);
  a45=(a2/a7);
  a47=(a6+a6);
  a5=cos(a5);
  a41=arg[2]? arg[2][3] : 0;
  a41=(a5*a41);
  a41=(a4*a41);
  a46=(a47*a41);
  a40=(a8+a8);
  a9=cos(a9);
  a34=arg[2]? arg[2][4] : 0;
  a34=(a9*a34);
  a34=(a4*a34);
  a36=(a40*a34);
  a46=(a46+a36);
  a36=(a7+a7);
  a46=(a46/a36);
  a31=(a45*a46);
  a35=(a11/a13);
  a30=(a3+a3);
  a12=cos(a12);
  a18=arg[2]? arg[2][5] : 0;
  a18=(a12*a18);
  a18=(a4*a18);
  a55=(a30*a18);
  a26=(a16+a16);
  a15=cos(a15);
  a42=arg[2]? arg[2][6] : 0;
  a42=(a15*a42);
  a42=(a4*a42);
  a53=(a26*a42);
  a55=(a55+a53);
  a53=(a13+a13);
  a55=(a55/a53);
  a50=(a35*a55);
  a39=(a31+a50);
  a32=(a19/a22);
  a25=(a20+a20);
  a21=cos(a21);
  a52=arg[2]? arg[2][7] : 0;
  a52=(a21*a52);
  a52=(a4*a52);
  a58=(a25*a52);
  a57=(a14+a14);
  a23=cos(a23);
  a33=arg[2]? arg[2][8] : 0;
  a33=(a23*a33);
  a33=(a4*a33);
  a37=(a57*a33);
  a58=(a58+a37);
  a37=(a22+a22);
  a58=(a58/a37);
  a48=(a32*a58);
  a39=(a39+a48);
  a39=(a28*a39);
  a51=(a51-a39);
  if (res[2]!=0) res[2][1]=a51;
  a41=(a24*a41);
  a51=(a24/a7);
  a46=(a51*a46);
  a46=(a6*a46);
  a41=(a41-a46);
  a34=(a2*a34);
  a31=(a8*a31);
  a34=(a34-a31);
  a41=(a41-a34);
  a18=(a27*a18);
  a34=(a27/a13);
  a55=(a34*a55);
  a55=(a3*a55);
  a18=(a18-a55);
  a41=(a41+a18);
  a42=(a11*a42);
  a50=(a16*a50);
  a42=(a42-a50);
  a41=(a41-a42);
  a52=(a29*a52);
  a42=(a29/a22);
  a58=(a42*a58);
  a58=(a20*a58);
  a52=(a52-a58);
  a41=(a41+a52);
  a33=(a19*a33);
  a48=(a14*a48);
  a33=(a33-a48);
  a41=(a41-a33);
  a41=(a44*a41);
  a10=(a8*a10);
  a10=(a44*a10);
  a41=(a41-a10);
  if (res[2]!=0) res[2][2]=a41;
  a41=1.;
  if (res[2]!=0) res[2][3]=a41;
  if (res[2]!=0) res[2][4]=a17;
  if (res[2]!=0) res[2][5]=a17;
  if (res[2]!=0) res[2][6]=a17;
  if (res[2]!=0) res[2][7]=a17;
  if (res[2]!=0) res[2][8]=a17;
  a10=arg[2]? arg[2][10] : 0;
  if (res[2]!=0) res[2][9]=a10;
  a10=arg[2]? arg[2][12] : 0;
  a10=(a5*a10);
  a10=(a4*a10);
  a33=(a47*a10);
  a48=arg[2]? arg[2][13] : 0;
  a48=(a9*a48);
  a48=(a4*a48);
  a52=(a40*a48);
  a33=(a33+a52);
  a33=(a33/a36);
  a52=(a45*a33);
  a58=arg[2]? arg[2][14] : 0;
  a58=(a12*a58);
  a58=(a4*a58);
  a50=(a30*a58);
  a18=arg[2]? arg[2][15] : 0;
  a18=(a15*a18);
  a18=(a4*a18);
  a55=(a26*a18);
  a50=(a50+a55);
  a50=(a50/a53);
  a55=(a35*a50);
  a31=(a52+a55);
  a46=arg[2]? arg[2][16] : 0;
  a46=(a21*a46);
  a46=(a4*a46);
  a39=(a25*a46);
  a59=arg[2]? arg[2][17] : 0;
  a59=(a23*a59);
  a59=(a4*a59);
  a1=(a57*a59);
  a39=(a39+a1);
  a39=(a39/a37);
  a1=(a32*a39);
  a31=(a31+a1);
  a31=(a28*a31);
  a31=(-a31);
  if (res[2]!=0) res[2][10]=a31;
  a7=(a0/a7);
  a7=(a6*a7);
  a7=(a44*a7);
  a10=(a24*a10);
  a33=(a51*a33);
  a33=(a6*a33);
  a10=(a10-a33);
  a48=(a2*a48);
  a52=(a8*a52);
  a48=(a48-a52);
  a10=(a10-a48);
  a58=(a27*a58);
  a50=(a34*a50);
  a50=(a3*a50);
  a58=(a58-a50);
  a10=(a10+a58);
  a18=(a11*a18);
  a55=(a16*a55);
  a18=(a18-a55);
  a10=(a10-a18);
  a46=(a29*a46);
  a39=(a42*a39);
  a39=(a20*a39);
  a46=(a46-a39);
  a10=(a10+a46);
  a59=(a19*a59);
  a1=(a14*a1);
  a59=(a59-a1);
  a10=(a10-a59);
  a10=(a44*a10);
  a7=(a7+a10);
  if (res[2]!=0) res[2][11]=a7;
  if (res[2]!=0) res[2][12]=a17;
  if (res[2]!=0) res[2][13]=a41;
  if (res[2]!=0) res[2][14]=a17;
  if (res[2]!=0) res[2][15]=a17;
  if (res[2]!=0) res[2][16]=a17;
  if (res[2]!=0) res[2][17]=a17;
  a7=arg[2]? arg[2][19] : 0;
  if (res[2]!=0) res[2][18]=a7;
  a7=(a0/a13);
  a10=(a28*a7);
  a59=arg[2]? arg[2][21] : 0;
  a59=(a5*a59);
  a59=(a4*a59);
  a1=(a47*a59);
  a46=arg[2]? arg[2][22] : 0;
  a46=(a9*a46);
  a46=(a4*a46);
  a39=(a40*a46);
  a1=(a1+a39);
  a1=(a1/a36);
  a39=(a45*a1);
  a18=arg[2]? arg[2][23] : 0;
  a18=(a12*a18);
  a18=(a4*a18);
  a55=(a30*a18);
  a58=arg[2]? arg[2][24] : 0;
  a58=(a15*a58);
  a58=(a4*a58);
  a50=(a26*a58);
  a55=(a55+a50);
  a55=(a55/a53);
  a50=(a35*a55);
  a48=(a39+a50);
  a52=arg[2]? arg[2][25] : 0;
  a52=(a21*a52);
  a52=(a4*a52);
  a33=(a25*a52);
  a31=arg[2]? arg[2][26] : 0;
  a31=(a23*a31);
  a31=(a4*a31);
  a56=(a57*a31);
  a33=(a33+a56);
  a33=(a33/a37);
  a56=(a32*a33);
  a48=(a48+a56);
  a48=(a28*a48);
  a10=(a10-a48);
  if (res[2]!=0) res[2][19]=a10;
  a59=(a24*a59);
  a1=(a51*a1);
  a1=(a6*a1);
  a59=(a59-a1);
  a46=(a2*a46);
  a39=(a8*a39);
  a46=(a46-a39);
  a59=(a59-a46);
  a18=(a27*a18);
  a55=(a34*a55);
  a55=(a3*a55);
  a18=(a18-a55);
  a59=(a59+a18);
  a58=(a11*a58);
  a50=(a16*a50);
  a58=(a58-a50);
  a59=(a59-a58);
  a52=(a29*a52);
  a33=(a42*a33);
  a33=(a20*a33);
  a52=(a52-a33);
  a59=(a59+a52);
  a31=(a19*a31);
  a56=(a14*a56);
  a31=(a31-a56);
  a59=(a59-a31);
  a59=(a44*a59);
  a7=(a16*a7);
  a7=(a44*a7);
  a59=(a59-a7);
  if (res[2]!=0) res[2][20]=a59;
  if (res[2]!=0) res[2][21]=a17;
  if (res[2]!=0) res[2][22]=a17;
  if (res[2]!=0) res[2][23]=a41;
  if (res[2]!=0) res[2][24]=a17;
  if (res[2]!=0) res[2][25]=a17;
  if (res[2]!=0) res[2][26]=a17;
  a59=arg[2]? arg[2][28] : 0;
  if (res[2]!=0) res[2][27]=a59;
  a59=arg[2]? arg[2][30] : 0;
  a59=(a5*a59);
  a59=(a4*a59);
  a7=(a47*a59);
  a31=arg[2]? arg[2][31] : 0;
  a31=(a9*a31);
  a31=(a4*a31);
  a56=(a40*a31);
  a7=(a7+a56);
  a7=(a7/a36);
  a56=(a45*a7);
  a52=arg[2]? arg[2][32] : 0;
  a52=(a12*a52);
  a52=(a4*a52);
  a33=(a30*a52);
  a58=arg[2]? arg[2][33] : 0;
  a58=(a15*a58);
  a58=(a4*a58);
  a50=(a26*a58);
  a33=(a33+a50);
  a33=(a33/a53);
  a50=(a35*a33);
  a18=(a56+a50);
  a55=arg[2]? arg[2][34] : 0;
  a55=(a21*a55);
  a55=(a4*a55);
  a46=(a25*a55);
  a39=arg[2]? arg[2][35] : 0;
  a39=(a23*a39);
  a39=(a4*a39);
  a1=(a57*a39);
  a46=(a46+a1);
  a46=(a46/a37);
  a1=(a32*a46);
  a18=(a18+a1);
  a18=(a28*a18);
  a18=(-a18);
  if (res[2]!=0) res[2][28]=a18;
  a13=(a0/a13);
  a13=(a3*a13);
  a13=(a44*a13);
  a59=(a24*a59);
  a7=(a51*a7);
  a7=(a6*a7);
  a59=(a59-a7);
  a31=(a2*a31);
  a56=(a8*a56);
  a31=(a31-a56);
  a59=(a59-a31);
  a52=(a27*a52);
  a33=(a34*a33);
  a33=(a3*a33);
  a52=(a52-a33);
  a59=(a59+a52);
  a58=(a11*a58);
  a50=(a16*a50);
  a58=(a58-a50);
  a59=(a59-a58);
  a55=(a29*a55);
  a46=(a42*a46);
  a46=(a20*a46);
  a55=(a55-a46);
  a59=(a59+a55);
  a39=(a19*a39);
  a1=(a14*a1);
  a39=(a39-a1);
  a59=(a59-a39);
  a59=(a44*a59);
  a13=(a13+a59);
  if (res[2]!=0) res[2][29]=a13;
  if (res[2]!=0) res[2][30]=a17;
  if (res[2]!=0) res[2][31]=a17;
  if (res[2]!=0) res[2][32]=a17;
  if (res[2]!=0) res[2][33]=a41;
  if (res[2]!=0) res[2][34]=a17;
  if (res[2]!=0) res[2][35]=a17;
  a13=arg[2]? arg[2][37] : 0;
  if (res[2]!=0) res[2][36]=a13;
  a13=(a0/a22);
  a59=(a28*a13);
  a39=arg[2]? arg[2][39] : 0;
  a39=(a5*a39);
  a39=(a4*a39);
  a1=(a47*a39);
  a55=arg[2]? arg[2][40] : 0;
  a55=(a9*a55);
  a55=(a4*a55);
  a46=(a40*a55);
  a1=(a1+a46);
  a1=(a1/a36);
  a46=(a45*a1);
  a58=arg[2]? arg[2][41] : 0;
  a58=(a12*a58);
  a58=(a4*a58);
  a50=(a30*a58);
  a52=arg[2]? arg[2][42] : 0;
  a52=(a15*a52);
  a52=(a4*a52);
  a33=(a26*a52);
  a50=(a50+a33);
  a50=(a50/a53);
  a33=(a35*a50);
  a31=(a46+a33);
  a56=arg[2]? arg[2][43] : 0;
  a56=(a21*a56);
  a56=(a4*a56);
  a7=(a25*a56);
  a18=arg[2]? arg[2][44] : 0;
  a18=(a23*a18);
  a18=(a4*a18);
  a10=(a57*a18);
  a7=(a7+a10);
  a7=(a7/a37);
  a10=(a32*a7);
  a31=(a31+a10);
  a31=(a28*a31);
  a59=(a59-a31);
  if (res[2]!=0) res[2][37]=a59;
  a39=(a24*a39);
  a1=(a51*a1);
  a1=(a6*a1);
  a39=(a39-a1);
  a55=(a2*a55);
  a46=(a8*a46);
  a55=(a55-a46);
  a39=(a39-a55);
  a58=(a27*a58);
  a50=(a34*a50);
  a50=(a3*a50);
  a58=(a58-a50);
  a39=(a39+a58);
  a52=(a11*a52);
  a33=(a16*a33);
  a52=(a52-a33);
  a39=(a39-a52);
  a56=(a29*a56);
  a7=(a42*a7);
  a7=(a20*a7);
  a56=(a56-a7);
  a39=(a39+a56);
  a18=(a19*a18);
  a10=(a14*a10);
  a18=(a18-a10);
  a39=(a39-a18);
  a39=(a44*a39);
  a13=(a14*a13);
  a13=(a44*a13);
  a39=(a39-a13);
  if (res[2]!=0) res[2][38]=a39;
  if (res[2]!=0) res[2][39]=a17;
  if (res[2]!=0) res[2][40]=a17;
  if (res[2]!=0) res[2][41]=a17;
  if (res[2]!=0) res[2][42]=a17;
  if (res[2]!=0) res[2][43]=a41;
  if (res[2]!=0) res[2][44]=a17;
  a39=arg[2]? arg[2][46] : 0;
  if (res[2]!=0) res[2][45]=a39;
  a39=arg[2]? arg[2][48] : 0;
  a5=(a5*a39);
  a5=(a4*a5);
  a47=(a47*a5);
  a39=arg[2]? arg[2][49] : 0;
  a9=(a9*a39);
  a9=(a4*a9);
  a40=(a40*a9);
  a47=(a47+a40);
  a47=(a47/a36);
  a45=(a45*a47);
  a36=arg[2]? arg[2][50] : 0;
  a12=(a12*a36);
  a12=(a4*a12);
  a30=(a30*a12);
  a36=arg[2]? arg[2][51] : 0;
  a15=(a15*a36);
  a15=(a4*a15);
  a26=(a26*a15);
  a30=(a30+a26);
  a30=(a30/a53);
  a35=(a35*a30);
  a53=(a45+a35);
  a26=arg[2]? arg[2][52] : 0;
  a21=(a21*a26);
  a21=(a4*a21);
  a25=(a25*a21);
  a26=arg[2]? arg[2][53] : 0;
  a23=(a23*a26);
  a4=(a4*a23);
  a57=(a57*a4);
  a25=(a25+a57);
  a25=(a25/a37);
  a32=(a32*a25);
  a53=(a53+a32);
  a28=(a28*a53);
  a28=(-a28);
  if (res[2]!=0) res[2][46]=a28;
  a0=(a0/a22);
  a0=(a20*a0);
  a0=(a44*a0);
  a24=(a24*a5);
  a51=(a51*a47);
  a6=(a6*a51);
  a24=(a24-a6);
  a2=(a2*a9);
  a8=(a8*a45);
  a2=(a2-a8);
  a24=(a24-a2);
  a27=(a27*a12);
  a34=(a34*a30);
  a3=(a3*a34);
  a27=(a27-a3);
  a24=(a24+a27);
  a11=(a11*a15);
  a16=(a16*a35);
  a11=(a11-a16);
  a24=(a24-a11);
  a29=(a29*a21);
  a42=(a42*a25);
  a20=(a20*a42);
  a29=(a29-a20);
  a24=(a24+a29);
  a19=(a19*a4);
  a14=(a14*a32);
  a19=(a19-a14);
  a24=(a24-a19);
  a44=(a44*a24);
  a0=(a0+a44);
  if (res[2]!=0) res[2][47]=a0;
  if (res[2]!=0) res[2][48]=a17;
  if (res[2]!=0) res[2][49]=a17;
  if (res[2]!=0) res[2][50]=a17;
  if (res[2]!=0) res[2][51]=a17;
  if (res[2]!=0) res[2][52]=a17;
  if (res[2]!=0) res[2][53]=a41;
  return 0;
}

CASADI_SYMBOL_EXPORT int model_expl_vde_forw(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int model_expl_vde_forw_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int model_expl_vde_forw_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void model_expl_vde_forw_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int model_expl_vde_forw_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void model_expl_vde_forw_release(int mem) {
}

CASADI_SYMBOL_EXPORT void model_expl_vde_forw_incref(void) {
}

CASADI_SYMBOL_EXPORT void model_expl_vde_forw_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int model_expl_vde_forw_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int model_expl_vde_forw_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real model_expl_vde_forw_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* model_expl_vde_forw_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* model_expl_vde_forw_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* model_expl_vde_forw_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    case 4: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* model_expl_vde_forw_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int model_expl_vde_forw_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif