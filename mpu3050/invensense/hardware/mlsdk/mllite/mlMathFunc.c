/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
#include "mlMathFunc.h"
#include "mlinclude.h"
#include "mlmath.h"

/** Performs a multiply and shift by 29. These are good functions to write in assembly on
 * with devices with small memory where you want to get rid of the long long which some
 * assemblers don't handle well
 * @param[in] a 
 * @param[in] b
 * @return ((long long)a*b)>>29
*/
long q29_mult( long a, long b )
{
    long long temp;
    long result;
    temp = (long long)a * b;
    result = (long)(temp >> 29);
    return result;
}

/** Performs a multiply and shift by 30. These are good functions to write in assembly on
 * with devices with small memory where you want to get rid of the long long which some
 * assemblers don't handle well
 * @param[in] a 
 * @param[in] b
 * @return ((long long)a*b)>>30
*/
long q30_mult( long a, long b )
{
    long long temp;
    long result;
    temp = (long long)a * b;
    result = (long)(temp >> 30);
    return result;
}
void MLQMult(long *q1, long *q2, long *qProd)
{   
    INVENSENSE_FUNC_START;
    qProd[0] = (long)(((long long)q1[0]*q2[0] - (long long)q1[1]*q2[1] -
        (long long)q1[2]*q2[2] - (long long)q1[3]*q2[3])>>30);     
    qProd[1] = (int)(((long long)q1[0]*q2[1] + (long long)q1[1]*q2[0] + 
        (long long)q1[2]*q2[3] - (long long)q1[3]*q2[2])>>30); 
    qProd[2] = (long)(((long long)q1[0]*q2[2] - (long long)q1[1]*q2[3] + 
        (long long)q1[2]*q2[0] + (long long)q1[3]*q2[1])>>30); 
    qProd[3] = (long)(((long long)q1[0]*q2[3] + (long long)q1[1]*q2[2] - 
        (long long)q1[2]*q2[1] + (long long)q1[3]*q2[0])>>30);
}

void MLQAdd(long *q1, long *q2, long *qSum)
{   
    INVENSENSE_FUNC_START;
    qSum[0] = q1[0]+q2[0];  
    qSum[1] = q1[1]+q2[1];  
    qSum[2] = q1[2]+q2[2];  
    qSum[3] = q1[3]+q2[3];
}

void MLQNormalize(long *q)
{    
    INVENSENSE_FUNC_START;
    double normSF = 0; 
    int i;
    for (i=0; i<4; i++) {
        normSF += ((double)q[i])/1073741824L*((double)q[i])/1073741824L;
    }
    if (normSF>0) {
        normSF = 1/sqrt(normSF);
        for (i=0; i<4; i++) {
            q[i] = (int)((double)q[i]*normSF);
        }
    }   else    {               
        q[0] = 1073741824L;     
        q[1] = 0;     
        q[2] = 0;     
        q[3] = 0; 
    }   
}
void MLQInvert(long *q, long *qInverted)
{   
    INVENSENSE_FUNC_START;
    qInverted[0] = q[0];    
    qInverted[1] = -q[1];   
    qInverted[2] = -q[2];   
    qInverted[3] = -q[3];
}


void MLQMultf(float *q1, float *q2, float *qProd)
{   
    INVENSENSE_FUNC_START;
    qProd[0] = (q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]);     
    qProd[1] = (q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]); 
    qProd[2] = (q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]); 
    qProd[3] = (q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]);
}

void MLQAddf(float *q1, float *q2, float *qSum) 
{   
    INVENSENSE_FUNC_START;
    qSum[0] = q1[0]+q2[0];  
    qSum[1] = q1[1]+q2[1];  
    qSum[2] = q1[2]+q2[2];  
    qSum[3] = q1[3]+q2[3];
}

void MLQNormalizef(float *q) 
{    
    INVENSENSE_FUNC_START;
    float normSF = 0;   
    float xHalf = 0;        
    normSF = (q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);     
    if (normSF<2)   {       
        xHalf = 0.5f*normSF;        
        normSF = normSF*(1.5f-xHalf*normSF*normSF);     
        normSF = normSF*(1.5f-xHalf*normSF*normSF);     
        normSF = normSF*(1.5f-xHalf*normSF*normSF);     
        normSF = normSF*(1.5f-xHalf*normSF*normSF);         
        q[0]*=normSF;       
        q[1]*=normSF;       
        q[2]*=normSF;       
        q[3]*=normSF;   
    } else {               
        q[0] = 1.0;     
        q[1] = 0.0;     
        q[2] = 0.0;     
        q[3] = 0.0; 
    }   
    normSF = (q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
}
void MLQInvertf(float *q, float *qInverted) 
{   
    INVENSENSE_FUNC_START;
    qInverted[0] = q[0];    
    qInverted[1] = -q[1];   
    qInverted[2] = -q[2];   
    qInverted[3] = -q[3];
}

/**
 * Converts a quaternion to a rotation matrix.
 * @param[in] quat 4-element quaternion in fixed point. One is 2^30.
 * @param[out] rot Rotation matrix in fixed point. One is 2^30. The
 *             First 3 elements of the rotation matrix, represent
 *             the first row of the matrix. Rotation matrix multiplied
 *             by a 3 element column vector transform a vector from Body
 *             to World.
 */
void quaternionToRotationMatrix( const long *quat, long *rot )
{
    rot[0] = q29_mult(quat[1],quat[1])+ q29_mult(quat[0],quat[0]) - 1073741824L;
    rot[1] = q29_mult(quat[1],quat[2])- q29_mult(quat[3],quat[0]);
    rot[2] = q29_mult(quat[1],quat[3])+ q29_mult(quat[2],quat[0]);
    rot[3] = q29_mult(quat[1],quat[2])+ q29_mult(quat[3],quat[0]);
    rot[4] = q29_mult(quat[2],quat[2])+ q29_mult(quat[0],quat[0]) - 1073741824L;
    rot[5] = q29_mult(quat[2],quat[3])- q29_mult(quat[1],quat[0]);
    rot[6] = q29_mult(quat[1],quat[3])- q29_mult(quat[2],quat[0]);
    rot[7] = q29_mult(quat[2],quat[3])+ q29_mult(quat[1],quat[0]);
    rot[8] = q29_mult(quat[3],quat[3])+ q29_mult(quat[0],quat[0]) - 1073741824L;
}

/** Converts a 32-bit long to a big endian byte stream
*/
unsigned char *Long32ToBig8(long x, unsigned char *big8)
{
    big8[0] = (unsigned char)((x>>24)&0xff);
    big8[1] = (unsigned char)((x>>16)&0xff);
    big8[2] = (unsigned char)((x>>8)&0xff);
    big8[3] = (unsigned char)(x&0xff);
    return big8;
}

void matDetInc(float *a,float *b,int *n,int x,int y)
{
    int k,l,i,j;
    for(i=0,k=0;i<*n;i++,k++)
    {
        for(j=0,l=0;j<*n;j++,l++)
        {
            if(i==x)
                i++;
            if(j==y)
                j++;
            *(b+10*k+l)=*(a+10*i+j);
        }
    }
    *n=*n-1;
}

float matDet(float *p,int *n)
{
    float d[10][10], sum=0;
    int i,j,m;
    m=*n;
    if(*n==2)
        return(*p**(p+11)-*(p+1)**(p+10));
    for(i=0,j=0;j<m;j++)
    {
        *n=m;
        matDetInc(p,&d[0][0],n,i,j);
        sum=sum+*(p+10*i+j)*(float)pow(-1,(i+j))*matDet(&d[0][0],n);
    }

    return(sum);
}

