/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) eval_hess_g_leg_simple_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)

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

static const casadi_int casadi_s0[100] = {96, 1, 0, 96, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95};
static const casadi_int casadi_s1[32] = {28, 1, 0, 28, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
static const casadi_int casadi_s2[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s3[172] = {96, 96, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 3, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 9, 13, 17, 21, 25, 29, 33, 37, 41, 45, 49, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 55, 57, 59, 64, 69, 70, 70, 70, 70, 72, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 64, 63, 64, 63, 64, 61, 62, 64, 65, 60, 62, 64, 65, 60, 61, 64, 65, 61, 62, 64, 65, 60, 62, 64, 65, 60, 61, 64, 65, 61, 62, 64, 65, 60, 62, 64, 65, 60, 61, 64, 65, 61, 62, 64, 65, 60, 62, 64, 65, 60, 61, 64, 65, 64, 65, 64, 65, 64, 65, 63, 64, 69, 70, 71, 64, 65, 69, 70, 71, 65, 70, 71, 71};

/* eval_hess_g_leg_simple:(w[96],lambda[28],p[14])->(hess_g[96x96,73nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a11, a110, a111, a112, a113, a114, a115, a116, a117, a118, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99;
  a0=arg[1]? arg[1][11] : 0;
  a1=-9.3212278260869572e-02;
  a2=arg[0]? arg[0][64] : 0;
  a3=cos(a2);
  a4=(a1*a3);
  a4=(a0*a4);
  a5=(-a4);
  if (res[0]!=0) res[0][0]=a5;
  a5=4.3125046780173909e-01;
  a6=cos(a2);
  a7=arg[0]? arg[0][63] : 0;
  a8=cos(a7);
  a9=(a6*a8);
  a9=(a5*a9);
  a9=(a0*a9);
  a10=arg[1]? arg[1][10] : 0;
  a11=sin(a7);
  a12=(a5*a11);
  a12=(a10*a12);
  a9=(a9-a12);
  a9=(-a9);
  if (res[0]!=0) res[0][1]=a9;
  a9=sin(a7);
  a12=sin(a2);
  a13=(a9*a12);
  a13=(a5*a13);
  a13=(a0*a13);
  if (res[0]!=0) res[0][2]=a13;
  a14=-4.6021926780173911e-01;
  a15=(a14*a8);
  a15=(a10*a15);
  a16=4.6021926780173911e-01;
  a17=(a6*a11);
  a17=(a16*a17);
  a17=(a0*a17);
  a15=(a15-a17);
  a15=(-a15);
  if (res[0]!=0) res[0][3]=a15;
  a15=cos(a7);
  a17=(a15*a12);
  a17=(a16*a17);
  a17=(a0*a17);
  if (res[0]!=0) res[0][4]=a17;
  a18=arg[2]? arg[2][0] : 0;
  a19=arg[1]? arg[1][9] : 0;
  a19=(a18*a19);
  a20=sin(a2);
  a21=(a19*a20);
  a22=(a18*a0);
  a21=(a21-a22);
  if (res[0]!=0) res[0][5]=a21;
  a21=(a18*a10);
  a23=arg[0]? arg[0][65] : 0;
  a24=cos(a23);
  a25=(a21*a24);
  a26=sin(a23);
  a27=(a6*a26);
  a27=(a19*a27);
  a25=(a25+a27);
  if (res[0]!=0) res[0][6]=a25;
  a25=arg[2]? arg[2][3] : 0;
  a27=arg[0]? arg[0][61] : 0;
  a28=(a25-a27);
  a29=(a28*a3);
  a30=arg[2]? arg[2][4] : 0;
  a31=arg[0]? arg[0][62] : 0;
  a30=(a30-a31);
  a32=(a26*a12);
  a33=(a30*a32);
  a29=(a29-a33);
  a29=(a19*a29);
  a29=(-a29);
  if (res[0]!=0) res[0][7]=a29;
  a29=sin(a23);
  a33=(a30*a29);
  a33=(a21*a33);
  a34=cos(a23);
  a35=(a6*a34);
  a36=(a30*a35);
  a36=(a19*a36);
  a33=(a33-a36);
  if (res[0]!=0) res[0][8]=a33;
  a33=(a19*a20);
  a33=(a22-a33);
  if (res[0]!=0) res[0][9]=a33;
  a33=(a21*a26);
  a36=(a6*a24);
  a36=(a19*a36);
  a33=(a33-a36);
  if (res[0]!=0) res[0][10]=a33;
  a33=arg[2]? arg[2][2] : 0;
  a36=arg[0]? arg[0][60] : 0;
  a37=(a33-a36);
  a38=(a37*a3);
  a39=(a24*a12);
  a40=(a30*a39);
  a38=(a38-a40);
  a38=(a19*a38);
  if (res[0]!=0) res[0][11]=a38;
  a38=(a30*a34);
  a38=(a21*a38);
  a40=(a6*a29);
  a41=(a30*a40);
  a41=(a19*a41);
  a38=(a38+a41);
  a38=(-a38);
  if (res[0]!=0) res[0][12]=a38;
  a38=(a6*a26);
  a38=(a19*a38);
  a41=(a21*a24);
  a38=(a38+a41);
  a38=(-a38);
  if (res[0]!=0) res[0][13]=a38;
  a38=(a6*a24);
  a38=(a19*a38);
  a41=(a21*a26);
  a38=(a38-a41);
  if (res[0]!=0) res[0][14]=a38;
  a38=(a27*a24);
  a41=(a36*a26);
  a42=(a38-a41);
  a43=(a25*a24);
  a42=(a42-a43);
  a43=(a33*a26);
  a42=(a42+a43);
  a43=(a42*a12);
  a43=(a19*a43);
  a43=(-a43);
  if (res[0]!=0) res[0][15]=a43;
  a43=(a25*a29);
  a44=(a27*a29);
  a45=(a36*a34);
  a46=(a44+a45);
  a43=(a43-a46);
  a46=(a33*a34);
  a43=(a43+a46);
  a43=(a6*a43);
  a43=(a19*a43);
  a46=(a27*a34);
  a47=(a36*a29);
  a48=(a46-a47);
  a49=(a33*a29);
  a48=(a48+a49);
  a49=(a25*a34);
  a48=(a48-a49);
  a48=(a21*a48);
  a43=(a43-a48);
  if (res[0]!=0) res[0][16]=a43;
  a43=(a19*a20);
  a43=(a43-a22);
  if (res[0]!=0) res[0][17]=a43;
  a43=(a21*a24);
  a48=(a6*a26);
  a48=(a19*a48);
  a43=(a43+a48);
  if (res[0]!=0) res[0][18]=a43;
  a43=arg[2]? arg[2][6] : 0;
  a48=(a43-a27);
  a49=(a48*a3);
  a50=arg[2]? arg[2][7] : 0;
  a50=(a50-a31);
  a51=(a26*a12);
  a52=(a50*a51);
  a49=(a49-a52);
  a49=(a19*a49);
  a49=(-a49);
  if (res[0]!=0) res[0][19]=a49;
  a49=(a50*a29);
  a49=(a21*a49);
  a52=(a6*a34);
  a53=(a50*a52);
  a53=(a19*a53);
  a49=(a49-a53);
  if (res[0]!=0) res[0][20]=a49;
  a49=(a19*a20);
  a49=(a22-a49);
  if (res[0]!=0) res[0][21]=a49;
  a49=(a21*a26);
  a53=(a6*a24);
  a53=(a19*a53);
  a49=(a49-a53);
  if (res[0]!=0) res[0][22]=a49;
  a49=arg[2]? arg[2][5] : 0;
  a53=(a49-a36);
  a54=(a53*a3);
  a55=(a24*a12);
  a56=(a50*a55);
  a54=(a54-a56);
  a54=(a19*a54);
  if (res[0]!=0) res[0][23]=a54;
  a54=(a50*a34);
  a54=(a21*a54);
  a56=(a6*a29);
  a57=(a50*a56);
  a57=(a19*a57);
  a54=(a54+a57);
  a54=(-a54);
  if (res[0]!=0) res[0][24]=a54;
  a54=(a6*a26);
  a54=(a19*a54);
  a57=(a21*a24);
  a54=(a54+a57);
  a54=(-a54);
  if (res[0]!=0) res[0][25]=a54;
  a54=(a6*a24);
  a54=(a19*a54);
  a57=(a21*a26);
  a54=(a54-a57);
  if (res[0]!=0) res[0][26]=a54;
  a54=(a38-a41);
  a57=(a43*a24);
  a54=(a54-a57);
  a57=(a49*a26);
  a54=(a54+a57);
  a57=(a54*a12);
  a57=(a19*a57);
  a57=(-a57);
  if (res[0]!=0) res[0][27]=a57;
  a57=(a43*a29);
  a58=(a44+a45);
  a57=(a57-a58);
  a58=(a49*a34);
  a57=(a57+a58);
  a57=(a6*a57);
  a57=(a19*a57);
  a58=(a46-a47);
  a59=(a49*a29);
  a58=(a58+a59);
  a59=(a43*a34);
  a58=(a58-a59);
  a58=(a21*a58);
  a57=(a57-a58);
  if (res[0]!=0) res[0][28]=a57;
  a57=(a19*a20);
  a57=(a57-a22);
  if (res[0]!=0) res[0][29]=a57;
  a57=(a21*a24);
  a58=(a6*a26);
  a58=(a19*a58);
  a57=(a57+a58);
  if (res[0]!=0) res[0][30]=a57;
  a57=arg[2]? arg[2][9] : 0;
  a58=(a57-a27);
  a59=(a58*a3);
  a60=arg[2]? arg[2][10] : 0;
  a60=(a60-a31);
  a61=(a26*a12);
  a62=(a60*a61);
  a59=(a59-a62);
  a59=(a19*a59);
  a59=(-a59);
  if (res[0]!=0) res[0][31]=a59;
  a59=(a60*a29);
  a59=(a21*a59);
  a62=(a6*a34);
  a63=(a60*a62);
  a63=(a19*a63);
  a59=(a59-a63);
  if (res[0]!=0) res[0][32]=a59;
  a59=(a19*a20);
  a59=(a22-a59);
  if (res[0]!=0) res[0][33]=a59;
  a59=(a21*a26);
  a63=(a6*a24);
  a63=(a19*a63);
  a59=(a59-a63);
  if (res[0]!=0) res[0][34]=a59;
  a59=arg[2]? arg[2][8] : 0;
  a63=(a59-a36);
  a64=(a63*a3);
  a65=(a24*a12);
  a66=(a60*a65);
  a64=(a64-a66);
  a64=(a19*a64);
  if (res[0]!=0) res[0][35]=a64;
  a64=(a60*a34);
  a64=(a21*a64);
  a66=(a6*a29);
  a67=(a60*a66);
  a67=(a19*a67);
  a64=(a64+a67);
  a64=(-a64);
  if (res[0]!=0) res[0][36]=a64;
  a64=(a6*a26);
  a64=(a19*a64);
  a67=(a21*a24);
  a64=(a64+a67);
  a64=(-a64);
  if (res[0]!=0) res[0][37]=a64;
  a64=(a6*a24);
  a64=(a19*a64);
  a67=(a21*a26);
  a64=(a64-a67);
  if (res[0]!=0) res[0][38]=a64;
  a64=(a38-a41);
  a67=(a57*a24);
  a64=(a64-a67);
  a67=(a59*a26);
  a64=(a64+a67);
  a67=(a64*a12);
  a67=(a19*a67);
  a67=(-a67);
  if (res[0]!=0) res[0][39]=a67;
  a67=(a57*a29);
  a68=(a44+a45);
  a67=(a67-a68);
  a68=(a59*a34);
  a67=(a67+a68);
  a67=(a6*a67);
  a67=(a19*a67);
  a68=(a46-a47);
  a69=(a59*a29);
  a68=(a68+a69);
  a69=(a57*a34);
  a68=(a68-a69);
  a68=(a21*a68);
  a67=(a67-a68);
  if (res[0]!=0) res[0][40]=a67;
  a67=(a19*a20);
  a67=(a67-a22);
  if (res[0]!=0) res[0][41]=a67;
  a67=(a21*a24);
  a68=(a6*a26);
  a68=(a19*a68);
  a67=(a67+a68);
  if (res[0]!=0) res[0][42]=a67;
  a67=arg[2]? arg[2][12] : 0;
  a68=(a67-a27);
  a69=(a68*a3);
  a70=arg[2]? arg[2][13] : 0;
  a70=(a70-a31);
  a31=(a26*a12);
  a71=(a70*a31);
  a69=(a69-a71);
  a69=(a19*a69);
  a69=(-a69);
  if (res[0]!=0) res[0][43]=a69;
  a69=(a70*a29);
  a69=(a21*a69);
  a71=(a6*a34);
  a72=(a70*a71);
  a72=(a19*a72);
  a69=(a69-a72);
  if (res[0]!=0) res[0][44]=a69;
  a69=(a19*a20);
  a69=(a22-a69);
  if (res[0]!=0) res[0][45]=a69;
  a69=(a21*a26);
  a72=(a6*a24);
  a72=(a19*a72);
  a69=(a69-a72);
  if (res[0]!=0) res[0][46]=a69;
  a69=arg[2]? arg[2][11] : 0;
  a72=(a69-a36);
  a73=(a72*a3);
  a74=(a24*a12);
  a75=(a70*a74);
  a73=(a73-a75);
  a73=(a19*a73);
  if (res[0]!=0) res[0][47]=a73;
  a73=(a70*a34);
  a73=(a21*a73);
  a75=(a6*a29);
  a76=(a70*a75);
  a76=(a19*a76);
  a73=(a73+a76);
  a73=(-a73);
  if (res[0]!=0) res[0][48]=a73;
  a73=(a6*a26);
  a73=(a19*a73);
  a76=(a21*a24);
  a73=(a73+a76);
  a73=(-a73);
  if (res[0]!=0) res[0][49]=a73;
  a73=(a6*a24);
  a73=(a19*a73);
  a76=(a21*a26);
  a73=(a73-a76);
  if (res[0]!=0) res[0][50]=a73;
  a38=(a38-a41);
  a41=(a67*a24);
  a38=(a38-a41);
  a41=(a69*a26);
  a38=(a38+a41);
  a41=(a38*a12);
  a41=(a19*a41);
  a41=(-a41);
  if (res[0]!=0) res[0][51]=a41;
  a41=(a67*a29);
  a44=(a44+a45);
  a41=(a41-a44);
  a44=(a69*a34);
  a41=(a41+a44);
  a41=(a6*a41);
  a41=(a19*a41);
  a46=(a46-a47);
  a47=(a69*a29);
  a46=(a46+a47);
  a47=(a67*a34);
  a46=(a46-a47);
  a46=(a21*a46);
  a41=(a41-a46);
  if (res[0]!=0) res[0][52]=a41;
  a41=arg[0]? arg[0][46] : 0;
  a46=(a41*a19);
  a47=(a46*a3);
  a44=arg[0]? arg[0][43] : 0;
  a45=(a44*a19);
  a73=(a45*a3);
  a47=(a47+a73);
  a73=arg[0]? arg[0][40] : 0;
  a76=(a73*a19);
  a77=(a76*a3);
  a47=(a47+a77);
  a77=arg[0]? arg[0][47] : 0;
  a78=(a77*a19);
  a79=(a78*a12);
  a80=arg[0]? arg[0][44] : 0;
  a81=(a80*a19);
  a82=(a81*a12);
  a83=(a79+a82);
  a84=arg[0]? arg[0][41] : 0;
  a85=(a84*a19);
  a86=(a85*a12);
  a83=(a83+a86);
  a87=arg[0]? arg[0][38] : 0;
  a88=(a87*a19);
  a89=(a88*a12);
  a83=(a83+a89);
  a90=(a26*a83);
  a47=(a47-a90);
  a90=arg[0]? arg[0][37] : 0;
  a91=(a90*a19);
  a92=(a91*a3);
  a47=(a47+a92);
  a47=(-a47);
  if (res[0]!=0) res[0][53]=a47;
  a47=(a6*a78);
  a92=(a6*a81);
  a93=(a47+a92);
  a94=(a6*a85);
  a93=(a93+a94);
  a95=(a6*a88);
  a93=(a93+a95);
  a96=(a93*a34);
  a77=(a77*a21);
  a80=(a80*a21);
  a97=(a77+a80);
  a84=(a84*a21);
  a97=(a97+a84);
  a87=(a87*a21);
  a97=(a97+a87);
  a98=(a97*a29);
  a96=(a96-a98);
  a96=(-a96);
  if (res[0]!=0) res[0][54]=a96;
  a96=arg[0]? arg[0][45] : 0;
  a98=(a96*a19);
  a99=(a98*a3);
  a100=arg[0]? arg[0][42] : 0;
  a101=(a100*a19);
  a102=(a101*a3);
  a99=(a99+a102);
  a102=arg[0]? arg[0][39] : 0;
  a103=(a102*a19);
  a104=(a103*a3);
  a99=(a99+a104);
  a104=(a79+a82);
  a104=(a104+a86);
  a104=(a104+a89);
  a105=(a24*a104);
  a99=(a99-a105);
  a105=arg[0]? arg[0][36] : 0;
  a106=(a105*a19);
  a107=(a106*a3);
  a99=(a99+a107);
  if (res[0]!=0) res[0][55]=a99;
  a99=(a77+a80);
  a99=(a99+a84);
  a99=(a99+a87);
  a107=(a99*a34);
  a108=(a47+a92);
  a108=(a108+a94);
  a108=(a108+a95);
  a109=(a108*a29);
  a107=(a107+a109);
  a107=(-a107);
  if (res[0]!=0) res[0][56]=a107;
  a31=(a98*a31);
  a74=(a46*a74);
  a31=(a31-a74);
  a61=(a101*a61);
  a65=(a45*a65);
  a61=(a61-a65);
  a31=(a31+a61);
  a51=(a103*a51);
  a55=(a76*a55);
  a51=(a51-a55);
  a31=(a31+a51);
  a32=(a106*a32);
  a39=(a91*a39);
  a32=(a32-a39);
  a31=(a31+a32);
  a31=(-a31);
  if (res[0]!=0) res[0][57]=a31;
  a75=(a46*a75);
  a41=(a41*a21);
  a31=(a41*a34);
  a96=(a96*a21);
  a32=(a96*a29);
  a31=(a31-a32);
  a75=(a75+a31);
  a71=(a98*a71);
  a75=(a75+a71);
  a66=(a45*a66);
  a44=(a44*a21);
  a71=(a44*a34);
  a100=(a100*a21);
  a31=(a100*a29);
  a71=(a71-a31);
  a66=(a66+a71);
  a62=(a101*a62);
  a66=(a66+a62);
  a75=(a75+a66);
  a56=(a76*a56);
  a73=(a73*a21);
  a66=(a73*a34);
  a102=(a102*a21);
  a62=(a102*a29);
  a66=(a66-a62);
  a56=(a56+a66);
  a52=(a103*a52);
  a56=(a56+a52);
  a75=(a75+a56);
  a40=(a91*a40);
  a90=(a90*a21);
  a34=(a90*a34);
  a105=(a105*a21);
  a29=(a105*a29);
  a34=(a34-a29);
  a40=(a40+a34);
  a35=(a106*a35);
  a40=(a40+a35);
  a75=(a75+a40);
  if (res[0]!=0) res[0][58]=a75;
  a75=arg[0]? arg[0][70] : 0;
  a40=arg[0]? arg[0][10] : 0;
  a40=(a75-a40);
  a35=(a40*a0);
  a35=(a5*a35);
  a34=(a6*a35);
  a29=arg[0]? arg[0][69] : 0;
  a56=arg[0]? arg[0][71] : 0;
  a52=(a29*a56);
  a66=(a52*a6);
  a62=3.6700698954086952e-01;
  a62=(a62*a22);
  a66=(a66*a62);
  a34=(a34-a66);
  a66=24358218631252112.;
  a71=-1.3877787807814460e-17;
  a71=(a71*a21);
  a21=(a29*a71);
  a31=(a66*a21);
  a32=(a75*a31);
  a34=(a34+a32);
  a32=arg[0]? arg[0][11] : 0;
  a32=(a56-a32);
  a39=(a32*a10);
  a39=(a14*a39);
  a34=(a34+a39);
  a39=arg[1]? arg[1][5] : 0;
  a39=(a18*a39);
  a51=(a39/a6);
  a55=(a75*a51);
  a34=(a34-a55);
  a55=arg[1]? arg[1][4] : 0;
  a55=(a18*a55);
  a61=(a56*a55);
  a34=(a34+a61);
  a61=arg[1]? arg[1][3] : 0;
  a18=(a18*a61);
  a61=(a18/a6);
  a65=(a20*a61);
  a74=(a75*a65);
  a34=(a34-a74);
  a74=sin(a7);
  a34=(a34*a74);
  a74=(a29*a75);
  a107=3.3803818954086950e-01;
  a107=(a107*a22);
  a109=(a6*a107);
  a110=(a74*a109);
  a32=(a32*a0);
  a32=(a16*a32);
  a111=(a6*a32);
  a110=(a110+a111);
  a111=26445640661418040.;
  a21=(a111*a21);
  a112=(a56*a21);
  a110=(a110+a112);
  a40=(a40*a10);
  a40=(a5*a40);
  a110=(a110+a40);
  a40=(a56*a51);
  a110=(a110-a40);
  a40=(a75*a55);
  a110=(a110-a40);
  a40=(a20*a61);
  a112=(a56*a40);
  a110=(a110-a112);
  a112=cos(a7);
  a110=(a110*a112);
  a34=(a34+a110);
  a34=(-a34);
  if (res[0]!=0) res[0][59]=a34;
  a34=cos(a7);
  a110=(a52*a12);
  a110=(a62*a110);
  a112=(a35*a12);
  a110=(a110-a112);
  a112=(a51/a6);
  a112=(a112*a12);
  a113=(a75*a112);
  a110=(a110-a113);
  a113=(a61/a6);
  a113=(a113*a12);
  a114=(a20*a113);
  a114=(a18+a114);
  a115=(a75*a114);
  a110=(a110-a115);
  a110=(a34*a110);
  a7=sin(a7);
  a115=(a107*a12);
  a116=(a74*a115);
  a117=(a32*a12);
  a116=(a116+a117);
  a117=(a56*a112);
  a116=(a116+a117);
  a117=(a20*a113);
  a117=(a18+a117);
  a118=(a56*a117);
  a116=(a116+a118);
  a116=(a7*a116);
  a110=(a110+a116);
  if (res[0]!=0) res[0][60]=a110;
  a110=(a75*a8);
  a110=(a66*a110);
  a116=(a56*a11);
  a116=(a111*a116);
  a110=(a110-a116);
  a110=(a71*a110);
  a11=(a109*a11);
  a11=(a75*a11);
  a8=(a62*a8);
  a8=(a6*a8);
  a8=(a56*a8);
  a11=(a11+a8);
  a110=(a110-a11);
  if (res[0]!=0) res[0][61]=a110;
  a110=(a5*a0);
  a110=(a6*a110);
  a110=(a110+a31);
  a110=(a110-a51);
  a110=(a110-a65);
  a110=(a34*a110);
  a65=(a109*a29);
  a5=(a5*a10);
  a65=(a65+a5);
  a65=(a65-a55);
  a65=(a7*a65);
  a110=(a110-a65);
  if (res[0]!=0) res[0][62]=a110;
  a14=(a14*a10);
  a10=(a6*a29);
  a10=(a62*a10);
  a14=(a14-a10);
  a14=(a14+a55);
  a34=(a34*a14);
  a16=(a16*a0);
  a16=(a6*a16);
  a16=(a16+a21);
  a16=(a16-a51);
  a16=(a16-a40);
  a7=(a7*a16);
  a34=(a34-a7);
  if (res[0]!=0) res[0][63]=a34;
  a34=(a75*a56);
  a7=-2.8968800000000020e-02;
  a7=(a7*a22);
  a34=(a34*a7);
  a22=arg[0]? arg[0][9] : 0;
  a22=(a29-a22);
  a22=(a22*a0);
  a1=(a1*a22);
  a34=(a34+a1);
  a72=(a72*a46);
  a34=(a34+a72);
  a68=(a68*a98);
  a34=(a34-a68);
  a63=(a63*a45);
  a34=(a34+a63);
  a58=(a58*a101);
  a34=(a34-a58);
  a53=(a53*a76);
  a34=(a34+a53);
  a48=(a48*a103);
  a34=(a34-a48);
  a37=(a37*a91);
  a34=(a34+a37);
  a28=(a28*a106);
  a34=(a34-a28);
  a28=(a75*a9);
  a37=(a28*a61);
  a34=(a34-a37);
  a37=(a56*a15);
  a48=(a37*a61);
  a34=(a34-a48);
  a48=sin(a2);
  a34=(a34*a48);
  a48=cos(a2);
  a53=(a28*a113);
  a58=(a37*a113);
  a53=(a53+a58);
  a48=(a48*a53);
  a34=(a34+a48);
  a74=(a74*a15);
  a74=(a74*a107);
  a62=(a9*a62);
  a52=(a52*a62);
  a74=(a74-a52);
  a32=(a15*a32);
  a74=(a74+a32);
  a35=(a9*a35);
  a74=(a74+a35);
  a38=(a38*a78);
  a74=(a74+a38);
  a46=(a70*a46);
  a38=(a24*a46);
  a74=(a74+a38);
  a98=(a70*a98);
  a38=(a26*a98);
  a74=(a74-a38);
  a64=(a64*a81);
  a74=(a74+a64);
  a45=(a60*a45);
  a64=(a24*a45);
  a74=(a74+a64);
  a101=(a60*a101);
  a64=(a26*a101);
  a74=(a74-a64);
  a54=(a54*a85);
  a74=(a74+a54);
  a76=(a50*a76);
  a54=(a24*a76);
  a74=(a74+a54);
  a103=(a50*a103);
  a54=(a26*a103);
  a74=(a74-a54);
  a42=(a42*a88);
  a74=(a74+a42);
  a91=(a30*a91);
  a24=(a24*a91);
  a74=(a74+a24);
  a106=(a30*a106);
  a26=(a26*a106);
  a74=(a74-a26);
  a26=(a56*a15);
  a24=(a75*a9);
  a26=(a26+a24);
  a26=(a26/a6);
  a24=(a26/a6);
  a42=(a24*a39);
  a74=(a74+a42);
  a42=(a29*a6);
  a88=(a37*a20);
  a42=(a42+a88);
  a88=(a28*a20);
  a42=(a42+a88);
  a42=(a42/a6);
  a88=(a42/a6);
  a54=(a88*a18);
  a74=(a74+a54);
  a61=(a29*a61);
  a74=(a74-a61);
  a61=cos(a2);
  a74=(a74*a61);
  a2=sin(a2);
  a26=(a26/a6);
  a26=(a26*a12);
  a26=(a26/a6);
  a24=(a24/a6);
  a24=(a24*a12);
  a26=(a26+a24);
  a39=(a39*a26);
  a37=(a37*a3);
  a26=(a29*a12);
  a37=(a37-a26);
  a28=(a28*a3);
  a37=(a37+a28);
  a37=(a37/a6);
  a42=(a42/a6);
  a42=(a42*a12);
  a37=(a37+a42);
  a37=(a37/a6);
  a88=(a88/a6);
  a88=(a88*a12);
  a37=(a37+a88);
  a18=(a18*a37);
  a39=(a39+a18);
  a113=(a29*a113);
  a39=(a39-a113);
  a2=(a2*a39);
  a74=(a74+a2);
  a34=(a34+a74);
  a34=(-a34);
  if (res[0]!=0) res[0][64]=a34;
  a34=cos(a23);
  a74=(a98*a12);
  a2=(a69*a79);
  a74=(a74-a2);
  a2=(a59*a82);
  a74=(a74-a2);
  a2=(a101*a12);
  a74=(a74+a2);
  a2=(a49*a86);
  a74=(a74-a2);
  a2=(a103*a12);
  a74=(a74+a2);
  a2=(a33*a89);
  a74=(a74-a2);
  a83=(a36*a83);
  a74=(a74+a83);
  a83=(a106*a12);
  a74=(a74+a83);
  a34=(a34*a74);
  a74=sin(a23);
  a79=(a67*a79);
  a83=(a46*a12);
  a79=(a79-a83);
  a82=(a57*a82);
  a79=(a79+a82);
  a82=(a45*a12);
  a79=(a79-a82);
  a86=(a43*a86);
  a79=(a79+a86);
  a86=(a76*a12);
  a79=(a79-a86);
  a89=(a25*a89);
  a79=(a79+a89);
  a104=(a27*a104);
  a79=(a79-a104);
  a104=(a91*a12);
  a79=(a79-a104);
  a74=(a74*a79);
  a34=(a34-a74);
  if (res[0]!=0) res[0][65]=a34;
  a12=(a62*a12);
  a34=(a56*a12);
  a115=(a15*a115);
  a74=(a75*a115);
  a34=(a34-a74);
  a34=(a34+a4);
  if (res[0]!=0) res[0][66]=a34;
  a3=(a7*a3);
  a56=(a56*a3);
  a115=(a29*a115);
  a56=(a56-a115);
  a56=(a56-a13);
  a13=(a9*a112);
  a56=(a56-a13);
  a114=(a9*a114);
  a56=(a56-a114);
  if (res[0]!=0) res[0][67]=a56;
  a75=(a75*a3);
  a29=(a29*a12);
  a75=(a75+a29);
  a75=(a75-a17);
  a112=(a15*a112);
  a75=(a75-a112);
  a117=(a15*a117);
  a75=(a75-a117);
  if (res[0]!=0) res[0][68]=a75;
  a75=(a67*a77);
  a41=(a70*a41);
  a75=(a75-a41);
  a41=(a57*a80);
  a75=(a75+a41);
  a44=(a60*a44);
  a75=(a75-a44);
  a44=(a43*a84);
  a75=(a75+a44);
  a73=(a50*a73);
  a75=(a75-a73);
  a73=(a25*a87);
  a75=(a75+a73);
  a99=(a27*a99);
  a75=(a75-a99);
  a90=(a30*a90);
  a75=(a75-a90);
  a90=(a69*a47);
  a75=(a75+a90);
  a98=(a6*a98);
  a75=(a75-a98);
  a98=(a59*a92);
  a75=(a75+a98);
  a101=(a6*a101);
  a75=(a75-a101);
  a101=(a49*a94);
  a75=(a75+a101);
  a103=(a6*a103);
  a75=(a75-a103);
  a103=(a33*a95);
  a75=(a75+a103);
  a93=(a36*a93);
  a75=(a75-a93);
  a106=(a6*a106);
  a75=(a75-a106);
  a106=sin(a23);
  a75=(a75*a106);
  a69=(a69*a77);
  a70=(a70*a96);
  a69=(a69-a70);
  a59=(a59*a80);
  a69=(a69+a59);
  a60=(a60*a100);
  a69=(a69-a60);
  a49=(a49*a84);
  a69=(a69+a49);
  a50=(a50*a102);
  a69=(a69-a50);
  a33=(a33*a87);
  a69=(a69+a33);
  a36=(a36*a97);
  a69=(a69-a36);
  a30=(a30*a105);
  a69=(a69-a30);
  a67=(a67*a47);
  a69=(a69-a67);
  a46=(a6*a46);
  a69=(a69+a46);
  a57=(a57*a92);
  a69=(a69-a57);
  a45=(a6*a45);
  a69=(a69+a45);
  a43=(a43*a94);
  a69=(a69-a43);
  a76=(a6*a76);
  a69=(a69+a76);
  a25=(a25*a95);
  a69=(a69-a25);
  a27=(a27*a108);
  a69=(a69+a27);
  a91=(a6*a91);
  a69=(a69+a91);
  a23=cos(a23);
  a69=(a69*a23);
  a75=(a75+a69);
  a75=(-a75);
  if (res[0]!=0) res[0][69]=a75;
  a109=(a15*a109);
  a66=(a66*a9);
  a66=(a71*a66);
  a109=(a109+a66);
  if (res[0]!=0) res[0][70]=a109;
  a111=(a111*a15);
  a71=(a71*a111);
  a6=(a6*a62);
  a71=(a71-a6);
  if (res[0]!=0) res[0][71]=a71;
  a20=(a20*a7);
  a7=2.8968800000000020e-02;
  a7=(a7*a19);
  a20=(a20+a7);
  if (res[0]!=0) res[0][72]=a20;
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int eval_hess_g_leg_simple(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

extern "C" CASADI_SYMBOL_EXPORT int eval_hess_g_leg_simple_alloc_mem(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int eval_hess_g_leg_simple_init_mem(int mem) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void eval_hess_g_leg_simple_free_mem(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT int eval_hess_g_leg_simple_checkout(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void eval_hess_g_leg_simple_release(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT void eval_hess_g_leg_simple_incref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT void eval_hess_g_leg_simple_decref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT casadi_int eval_hess_g_leg_simple_n_in(void) { return 3;}

extern "C" CASADI_SYMBOL_EXPORT casadi_int eval_hess_g_leg_simple_n_out(void) { return 1;}

extern "C" CASADI_SYMBOL_EXPORT casadi_real eval_hess_g_leg_simple_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* eval_hess_g_leg_simple_name_in(casadi_int i){
  switch (i) {
    case 0: return "w";
    case 1: return "lambda";
    case 2: return "p";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* eval_hess_g_leg_simple_name_out(casadi_int i){
  switch (i) {
    case 0: return "hess_g";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* eval_hess_g_leg_simple_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* eval_hess_g_leg_simple_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT int eval_hess_g_leg_simple_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

