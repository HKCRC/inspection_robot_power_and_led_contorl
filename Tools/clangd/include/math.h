/*
 * Minimal math.h stub for clangd only.
 *
 * The project is built with Keil ARMCC, whose libc headers clangd cannot
 * consume. When clangd indexes with `--target=arm-none-eabi` (freestanding),
 * <math.h> is unavailable, so this file supplies the declarations the code
 * references. It is NOT part of the Keil build and is never compiled.
 */
#ifndef _MATH_H_STUB
#define _MATH_H_STUB

#ifdef __cplusplus
extern "C" {
#endif

#define M_E        2.7182818284590452354
#define M_LOG2E    1.4426950408889634074
#define M_LOG10E   0.43429448190325182765
#define M_LN2      0.69314718055994530942
#define M_LN10     2.30258509299404568402
#define M_PI       3.14159265358979323846
#define M_PI_2     1.57079632679489661923
#define M_PI_4     0.78539816339744830962
#define M_1_PI     0.31830988618379067154
#define M_2_PI     0.63661977236758134308
#define M_2_SQRTPI 1.12837916709551257390
#define M_SQRT2    1.41421356237309504880
#define M_SQRT1_2  0.70710678118654752440

#define HUGE_VAL   (__builtin_huge_val())
#define HUGE_VALF  (__builtin_huge_valf())
#define HUGE_VALL  (__builtin_huge_vall())
#define INFINITY   (__builtin_inff())
#define NAN        (__builtin_nanf(""))

double      fabs(double x);
double      fmod(double x, double y);
double      fmax(double x, double y);
double      fmin(double x, double y);
double      sqrt(double x);
double      cbrt(double x);
double      pow(double x, double y);
double      exp(double x);
double      log(double x);
double      log10(double x);
double      log2(double x);
double      sin(double x);
double      cos(double x);
double      tan(double x);
double      asin(double x);
double      acos(double x);
double      atan(double x);
double      atan2(double y, double x);
double      sinh(double x);
double      cosh(double x);
double      tanh(double x);
double      floor(double x);
double      ceil(double x);
double      round(double x);
double      trunc(double x);
double      ldexp(double x, int exp);
double      frexp(double x, int *exp);
double      modf(double x, double *iptr);

float       fabsf(float x);
float       fmodf(float x, float y);
float       fmaxf(float x, float y);
float       fminf(float x, float y);
float       sqrtf(float x);
float       powf(float x, float y);
float       expf(float x);
float       logf(float x);
float       log10f(float x);
float       log2f(float x);
float       sinf(float x);
float       cosf(float x);
float       tanf(float x);
float       asinf(float x);
float       acosf(float x);
float       atanf(float x);
float       atan2f(float y, float x);
float       sinhf(float x);
float       coshf(float x);
float       tanhf(float x);
float       floorf(float x);
float       ceilf(float x);
float       roundf(float x);
float       truncf(float x);

long double fabsl(long double x);
long double sqrtl(long double x);
long double powl(long double x, long double y);
long double expl(long double x);
long double logl(long double x);
long double sinl(long double x);
long double cosl(long double x);
long double tanl(long double x);

#ifdef __cplusplus
}
#endif

#endif /* _MATH_H_STUB */
