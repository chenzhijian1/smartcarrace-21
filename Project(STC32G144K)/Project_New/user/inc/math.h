/*--------------------------------------------------------------------------
MATH.H

Prototypes for mathematic functions for C251 Version 4.
Copyright (c) 1995-2004 Keil Elektronik GmbH and Keil Software, Inc.
All rights reserved.
--------------------------------------------------------------------------*/

#pragma SAVE
#pragma PARM251

#if __C251__ >= 200
extern char   cabs  (char   val) reentrant;
extern int     abs  (int    val) reentrant;
extern long   labs  (long   val) reentrant;
extern double fabs  (double val) reentrant;
extern double sqrt  (double val) reentrant;
extern double exp   (double val) reentrant;
extern double log   (double val) reentrant;
extern double log10 (double val) reentrant;
extern double sin   (double val) reentrant;
extern double cos   (double val) reentrant;
extern double tan   (double val) reentrant;
extern double asin  (double val) reentrant;
extern double acos  (double val) reentrant;
extern double atan  (double val) reentrant;
extern double sinh  (double val) reentrant;
extern double cosh  (double val) reentrant;
extern double tanh  (double val) reentrant;
extern double atan2 (double y, double x) reentrant;
extern double ceil  (double val) reentrant;
extern double floor (double val) reentrant;
extern double modf  (double val, double *n) reentrant;
extern double pow   (double x, double y) reentrant;
extern double fmod  (double x, double y) reentrant;

extern unsigned char _chkfloat_  (float x)  reentrant;
extern unsigned char _chkdouble_ (double x) reentrant;
#else

extern char  cabs  (char  val);
extern int    abs  (int   val);
extern long  labs  (long  val);
extern float fabs  (float val);
extern float sqrt  (float val);
extern float exp   (float val);
extern float log   (float val);
extern float log10 (float val);
extern float sin   (float val);
extern float cos   (float val);
extern float tan   (float val);
extern float asin  (float val);
extern float acos  (float val);
extern float atan  (float val);
extern float sinh  (float val);
extern float cosh  (float val);
extern float tanh  (float val);
extern float atan2 (float y, float x);
extern float ceil  (float val);
extern float floor (float val);
extern float modf  (float val, float *n);
extern float pow   (float x, float y);
#endif

#pragma RESTORE
