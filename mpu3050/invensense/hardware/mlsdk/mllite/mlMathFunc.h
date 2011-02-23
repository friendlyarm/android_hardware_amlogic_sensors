/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
#ifndef INVENSENSE_ML_MATH_FUNC_H__
#define INVENSENSE_ML_MATH_FUNC_H__


#define NUM_ROTATION_MATRIX_ELEMENTS (9)
#define ROT_MATRIX_SCALE_LONG  (1073741824)
#define ROT_MATRIX_SCALE_FLOAT (1073741824.0f)
#define ROT_MATRIX_LONG_TO_FLOAT( longval ) \
    ((float) ((longval) / ROT_MATRIX_SCALE_FLOAT ))

#ifdef __cplusplus
extern "C" {
#endif

    long q29_mult( long a, long b );
    long q30_mult( long a, long b );
    void MLQMult(long *q1, long *q2, long *qProd);
    void MLQAdd(long *q1, long *q2, long *qSum);
    void MLQNormalize(long *q);
    void MLQInvert(long *q, long *qInverted);
    void MLQMultf(float *q1, float *q2, float *qProd);
    void MLQAddf(float *q1, float *q2, float *qSum);
    void MLQNormalizef(float *q);
    void MLQInvertf(float *q, float *qInverted);
    void quaternionToRotationMatrix( const long *quat, long *rot );
    unsigned char *Long32ToBig8(long x, unsigned char *big8);
    float matDet(float *p,int *n);
    void matDetInc(float *a,float *b,int *n,int x,int y);

#ifdef __cplusplus
}
#endif

#endif // INVENSENSE_ML_MATH_FUNC_H__
