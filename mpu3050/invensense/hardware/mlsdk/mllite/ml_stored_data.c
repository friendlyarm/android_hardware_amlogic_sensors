/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 * $Id: ml_stored_data.c 4175 2010-11-25 01:24:24Z nroyer $
 *****************************************************************************/

/**
 * @defgroup ML_STORED_DATA
 *
 * @{
 *      @file     ml_stored_data.c
 *      @brief    functions for reading and writing stored data sets.
 *                Typically, these functions process stored calibration data.
 */

#include "ml.h"
#include "mltypes.h"
#include "mlinclude.h"
#include "compass.h"
#include "dmpKey.h"
#include "dmpDefault.h"
#include "mlstates.h"
#include "checksum.h"
#include "mlsupervisor.h"

#include "mlsl.h"
#include "mlos.h"

extern tMLXData mlxData;

#define ML_CAL_HDR_LEN 6
#define ML_CAL_CHK_LEN 4

/**
 * @brief   Returns the length of the calibration data. Should be called 
 *          before allocating the memory required to store this data to a file.
 *          This function returns the total size required to store the cal 
 *          data including the header (4 bytes) and the checksum (2 bytes).
 *
 * @pre     MLDmpOpen()
 *
 * @param   length
 *              The length of the calibration data.
 * @return  Zero is returned if the command is successful;
 *          otherwise, an error code is returned.
 */
tMLError MLGetCalLength(unsigned int *length)
{
    INVENSENSE_FUNC_START;
 
    if ( MLGetState() != ML_STATE_DMP_STARTED )
        return ML_ERROR_SM_IMPROPER_STATE;

    /* the length is the gyro temp table, plus the accel biases, 
       header and checkusm */
    *length = BINS*PTS_PER_BIN*4*4+BINS*4*2 + 12 +
              ML_CAL_HDR_LEN + ML_CAL_CHK_LEN + 207;
    return ML_SUCCESS;
}


typedef tMLError(*t_ml_load_func)(unsigned char*, unsigned short);

tMLError MLLoadCal_V0(unsigned char *calData, unsigned short len)
{
    INVENSENSE_FUNC_START;
    long newGyroData[3] = {0};    
    long newTemp = 0;
    int bin;

    newTemp = ((long)calData[6])*256+((long)calData[7]);
    if (newTemp>32767L) newTemp-=65536L;
    newGyroData[0] = ((long)calData[8])*256+((long)calData[9]);
    if (newGyroData[0]>32767L) newGyroData[0]-=65536L;
    newGyroData[1] = ((long)calData[10])*256+((long)calData[11]);
    if (newGyroData[1] >32767L) newGyroData[1] -=65536L;
    newGyroData[2] = ((long)calData[12])*256+((long)calData[13]);
    if (newGyroData[2]>32767L) newGyroData[2]-=65536L;
    bin = (int)((newTemp-MIN_TEMP)/((MAX_TEMP-MIN_TEMP)/BINS));
    
    if (bin<0) bin = 0;
    if (bin>BINS-1) bin = BINS-1;
    
    mlxData.mlTempData[bin][mlxData.mlTempPtrs[bin]] =
        35.0f+((((float)newTemp)+13200.0f)/280.0f);
    mlxData.mlXGyroTempData[bin][mlxData.mlTempPtrs[bin]] =
        ((float)newGyroData[0])/65536.0f;    
    mlxData.mlYGyroTempData[bin][mlxData.mlTempPtrs[bin]] =
        ((float)newGyroData[0])/65536.0f;
    mlxData.mlZGyroTempData[bin][mlxData.mlTempPtrs[bin]] =
        ((float)newGyroData[0])/65536.0f;    
    mlxData.mlTempPtrs[bin] = (mlxData.mlTempPtrs[bin]+1) % PTS_PER_BIN;

    if (mlxData.mlTempPtrs[bin]==0) mlxData.mlTempValidData[bin] = 1; 

    return ML_SUCCESS;
}

tMLError MLLoadCal_V1(unsigned char *calData, unsigned short len)
{
    INVENSENSE_FUNC_START;
    long newGyroData[3] = {0};    
    float newTemp = 0;
    int bin;
    long accelBias[3] = {0};    
    float gyroBias[3] = {0};

    newTemp = (float)(((long)calData[6])*256+((long)calData[7]));
    if (newTemp>32767) newTemp-=65536L;
    newTemp = 35.0f+((((float)newTemp)+13200.0f)/280.0f);

    newGyroData[0] = ((long)calData[8])*256+((long)calData[9]);
    if (newGyroData[0]>32767L) newGyroData[0]-=65536L;
    newGyroData[1] = ((long)calData[10])*256+((long)calData[11]);
    if (newGyroData[1] >32767L) newGyroData[1] -=65536L;
    newGyroData[2] = ((long)calData[12])*256+((long)calData[13]);
    if (newGyroData[2]>32767L) newGyroData[2]-=65536L;

    accelBias[0] = ((long)calData[14])*256+((long)calData[15]);
    if (accelBias[0]>32767) accelBias[0]-=65536L;    
    accelBias[0] = (long)(
        (long long)accelBias[0]*65536L*mlxData.mlAccelSens/1073741824L);

    accelBias[1] = ((long)calData[16])*256+((long)calData[17]);
    if (accelBias[1]>32767) accelBias[1]-=65536L;
    accelBias[1] = (long)(
        (long long)accelBias[1]*65536L*mlxData.mlAccelSens/1073741824L);

    accelBias[2] = ((long)calData[18])*256+((int)calData[19]);
    if (accelBias[2]>32767) accelBias[2]-=65536L;
    accelBias[2] = (long)(
        (long long)accelBias[2]*65536L*mlxData.mlAccelSens/1073741824L);

    bin = (int)((newTemp-MIN_TEMP)/((MAX_TEMP-MIN_TEMP)/BINS));
    if (bin<0) bin = 0;
    else if (bin>BINS-1) bin = BINS-1;

    gyroBias[0] = ((float)newGyroData[0])/131.072f;
    gyroBias[1] = ((float)newGyroData[1])/131.072f;
    gyroBias[2] = ((float)newGyroData[2])/131.072f;

    mlxData.mlTempData[bin][mlxData.mlTempPtrs[bin]] = newTemp;
    mlxData.mlXGyroTempData[bin][mlxData.mlTempPtrs[bin]] = gyroBias[0];
    mlxData.mlYGyroTempData[bin][mlxData.mlTempPtrs[bin]] = gyroBias[1];
    mlxData.mlZGyroTempData[bin][mlxData.mlTempPtrs[bin]] = gyroBias[2];
    mlxData.mlTempPtrs[bin] = (mlxData.mlTempPtrs[bin]+1)%PTS_PER_BIN;

    if (mlxData.mlTempPtrs[bin]==0) mlxData.mlTempValidData[bin] = 1; 

    MLSetArray(ML_ACCEL_BIAS, accelBias);

    return ML_SUCCESS;
}

tMLError MLLoadCal_V2(unsigned char *calData, unsigned short len)
{
    INVENSENSE_FUNC_START;
    int i, j;
    int ptr=6;
    long accel_bias[3];
    long long tmp;

    for (i=0; i<BINS; i++) {
        mlxData.mlTempPtrs[i] = 0;
        mlxData.mlTempPtrs[i] += 16777216L*((long)calData[ptr++]);
        mlxData.mlTempPtrs[i] += 65536L*((long)calData[ptr++]);
        mlxData.mlTempPtrs[i] += 256*((int)calData[ptr++]);
        mlxData.mlTempPtrs[i] += (int)calData[ptr++];
    }
    for (i=0; i<BINS; i++) {
        mlxData.mlTempValidData[i] = 0;
        mlxData.mlTempValidData[i] += 16777216L*((long)calData[ptr++]);
        mlxData.mlTempValidData[i] += 65536L*((long)calData[ptr++]);
        mlxData.mlTempValidData[i] += 256*((int)calData[ptr++]);
        mlxData.mlTempValidData[i] += (int)calData[ptr++];
    }


    for (i=0; i<BINS; i++) {
        for (j=0; j<PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL*(long long)calData[ptr++];
            tmp += 65536LL*(long long)calData[ptr++];
            tmp += 256LL*(long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp>2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlTempData[i][j] = ((float)tmp)/65536.0f;
        }        
        
    }
    for (i=0; i<BINS; i++) {
        for (j=0; j<PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL*(long long)calData[ptr++];
            tmp += 65536LL*(long long)calData[ptr++];
            tmp += 256LL*(long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp>2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlXGyroTempData[i][j] = ((float)tmp)/65536.0f;
        }        
    }
    for (i=0; i<BINS; i++) {
        for (j=0; j<PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL*(long long)calData[ptr++];
            tmp += 65536LL*(long long)calData[ptr++];
            tmp += 256LL*(long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp>2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlYGyroTempData[i][j] = ((float)tmp)/65536.0f;
        }        
    }
    for (i=0; i<BINS; i++) {
        for (j=0; j<PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL*(long long)calData[ptr++];
            tmp += 65536LL*(long long)calData[ptr++];
            tmp += 256LL*(long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp>2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlZGyroTempData[i][j] = ((float)tmp)/65536.0f;
        }        
    }

    /* read the accel biases */
    for (i = 0; i < 3; i++) {
        uint32_t t=0;
        t += 16777216UL * ((uint32_t) calData[ptr++]);
        t += 65536UL * ((uint32_t) calData[ptr++]);
        t += 256u * ((uint32_t) calData[ptr++]);
        t += (uint32_t) calData[ptr++];
        accel_bias[i] = (int32_t)t;
    }

    MLSetArray(ML_ACCEL_BIAS, accel_bias); 

    return ML_SUCCESS;
}

tMLError MLLoadCal_V3(unsigned char *calData, unsigned short len)
{
    INVENSENSE_FUNC_START;

    int i, j;
    int ptr=6;
    long bias[3];
    long long tmp;
    union doubleToLongLong {
        double db;
        unsigned long long ll;
    } dToLL;  

    for (i=0; i<BINS; i++) {
        mlxData.mlTempPtrs[i] = 0;
        mlxData.mlTempPtrs[i] += 16777216L*((long)calData[ptr++]);
        mlxData.mlTempPtrs[i] += 65536L*((long)calData[ptr++]);
        mlxData.mlTempPtrs[i] += 256*((int)calData[ptr++]);
        mlxData.mlTempPtrs[i] += (int)calData[ptr++];
    }
    for (i=0; i<BINS; i++) {
        mlxData.mlTempValidData[i] = 0;
        mlxData.mlTempValidData[i] += 16777216L*((long)calData[ptr++]);
        mlxData.mlTempValidData[i] += 65536L*((long)calData[ptr++]);
        mlxData.mlTempValidData[i] += 256*((int)calData[ptr++]);
        mlxData.mlTempValidData[i] += (int)calData[ptr++];
    }


    for (i=0; i<BINS; i++) {
        for (j=0; j<PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL*(long long)calData[ptr++];
            tmp += 65536LL*(long long)calData[ptr++];
            tmp += 256LL*(long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp>2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlTempData[i][j] = ((float)tmp)/65536.0f;
        }        
        
    }
    for (i=0; i<BINS; i++) {
        for (j=0; j<PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL*(long long)calData[ptr++];
            tmp += 65536LL*(long long)calData[ptr++];
            tmp += 256LL*(long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp>2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlXGyroTempData[i][j] = ((float)tmp)/65536.0f;
        }        
    }
    for (i=0; i<BINS; i++) {
        for (j=0; j<PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL*(long long)calData[ptr++];
            tmp += 65536LL*(long long)calData[ptr++];
            tmp += 256LL*(long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp>2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlYGyroTempData[i][j] = ((float)tmp)/65536.0f;
        }        
    }
    for (i=0; i<BINS; i++) {
        for (j=0; j<PTS_PER_BIN; j++) {
            tmp = 0;
            tmp += 16777216LL*(long long)calData[ptr++];
            tmp += 65536LL*(long long)calData[ptr++];
            tmp += 256LL*(long long)calData[ptr++];
            tmp += (long long)calData[ptr++];
            if (tmp>2147483648LL) {
                tmp -= 4294967296LL;
            }
            mlxData.mlZGyroTempData[i][j] = ((float)tmp)/65536.0f;
        }
    }

    /* read the accel biases */
    for (i = 0; i < 3; i++) {
        uint32_t t=0;
        t += 16777216UL * ((uint32_t) calData[ptr++]);
        t += 65536UL * ((uint32_t) calData[ptr++]);
        t += 256u * ((uint32_t) calData[ptr++]);
        t += (uint32_t) calData[ptr++];
        bias[i] = (int32_t)t;
    }
    MLSetArray(ML_ACCEL_BIAS, bias);

    /* read the compass biases */
    mlxData.mlGotCompassBias = (int)calData[ptr++];
    mlxData.mlGotInitCompassBias = (int)calData[ptr++];
    mlxData.mlCompassState = (int)calData[ptr++];
    
    for (i = 0; i < 3; i++) {
        uint32_t t=0;
        t += 16777216UL * ((uint32_t) calData[ptr++]);
        t += 65536UL * ((uint32_t) calData[ptr++]);
        t += 256u * ((uint32_t) calData[ptr++]);
        t += (uint32_t) calData[ptr++];
        mlxData.mlMagBiasError[i] = (int32_t)t;
    }
    for (i = 0; i < 3; i++) {
        uint32_t t=0;
        t += 16777216UL * ((uint32_t) calData[ptr++]);
        t += 65536UL * ((uint32_t) calData[ptr++]);
        t += 256u * ((uint32_t) calData[ptr++]);
        t += (uint32_t) calData[ptr++];
        mlxData.mlInitMagBias[i] = (int32_t)t;
    }
    for (i = 0; i < 3; i++) {
        uint32_t t=0;
        t += 16777216UL * ((uint32_t) calData[ptr++]);
        t += 65536UL * ((uint32_t) calData[ptr++]);
        t += 256u * ((uint32_t) calData[ptr++]);
        t += (uint32_t) calData[ptr++];
        mlxData.mlMagBias[i] = (int32_t)t;
    }
    for (i = 0; i < 18; i++) {
        uint32_t t=0;
        t += 16777216UL * ((uint32_t) calData[ptr++]);
        t += 65536UL * ((uint32_t) calData[ptr++]);
        t += 256u * ((uint32_t) calData[ptr++]);
        t += (uint32_t) calData[ptr++];
        mlxData.mlMagPeaks[i] = (int32_t)t;
    }
    for (i = 0; i < 3; i++) {
        dToLL.ll = 0;
        dToLL.ll += 72057594037927936ULL * 
            ((unsigned long long) calData[ptr++]);
        dToLL.ll += 281474976710656ULL * ((unsigned long long) calData[ptr++]);
        dToLL.ll += 1099511627776ULL * ((unsigned long long) calData[ptr++]);
        dToLL.ll += 4294967296LL * ((unsigned long long) calData[ptr++]);
        dToLL.ll += 16777216ULL * ((unsigned long long) calData[ptr++]);
        dToLL.ll += 65536ULL * ((unsigned long long) calData[ptr++]);
        dToLL.ll += 256LL * ((unsigned long long) calData[ptr++]);
        dToLL.ll += (unsigned long long) calData[ptr++];

        mlxData.mlMagBiasV[i] = dToLL.db;
    }    
    for (i = 0; i < 9; i++) {
        dToLL.ll = 0;
        dToLL.ll += 72057594037927936ULL * 
            ((unsigned long long) calData[ptr++]);
        dToLL.ll += 281474976710656ULL * ((unsigned long long) calData[ptr++]);
        dToLL.ll += 1099511627776ULL * ((unsigned long long) calData[ptr++]);
        dToLL.ll += 4294967296LL * ((unsigned long long) calData[ptr++]);
        dToLL.ll += 16777216ULL * ((unsigned long long) calData[ptr++]);
        dToLL.ll += 65536ULL * ((unsigned long long) calData[ptr++]);
        dToLL.ll += 256LL * ((unsigned long long) calData[ptr++]);
        dToLL.ll += (unsigned long long) calData[ptr++];

        mlxData.mlMagBiasP[i] = dToLL.db;          
    }        
    

    return ML_SUCCESS;
}

/**
 * @brief   Loads a set of calibration data. 
 *          It parses a binary data set containing calibration data. 
 *          The binary data set is intended to be loaded from a file.
 *
 * @pre     MLDmpOpen() Must be called with MLDmpDefaultOpen() or
 *          MLDmpPedometerStandAloneOpen()
 *
 * @param   calData     
 *              A pointer to an array of bytes to be parsed.
 * @return  ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLLoadCal(unsigned char *calData)
{
    INVENSENSE_FUNC_START;
    int ptr;
    int calType = 0;
    int len = 0;
    uint32_t chk = 0;
    uint32_t cmp_chk = 0;

    t_ml_load_func loaders[] = { MLLoadCal_V0,
                                 MLLoadCal_V1,
                                 MLLoadCal_V2,
                                 MLLoadCal_V3};

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;  

    /* read the header (type and len)
       len is the total record length including header and checksum */
    len = 0;
    len += 16777216L*((int)calData[0]);
    len += 65536L*((int)calData[1]);
    len += 256*((int)calData[2]);
    len += (int)calData[3];

    calType = ((int)calData[4])*256 + ((int)calData[5]);
    if (calType>3) {
        MPL_LOGE("Unsopported calibration file format %d. "
                 "Valid types 0..3\n", calType);
        return ML_ERROR_INVALID_PARAMETER;
    }

    /* check the checksum */
    chk = 0;
    ptr = len - ML_CAL_CHK_LEN;

    chk += 16777216L*((uint32_t)calData[ptr++]);
    chk += 65536L*((uint32_t)calData[ptr++]);
    chk += 256*((uint32_t)calData[ptr++]);
    chk += (uint32_t)calData[ptr++];

    cmp_chk = ml_checksum(calData + ML_CAL_HDR_LEN, 
        len - (ML_CAL_HDR_LEN + ML_CAL_CHK_LEN));
         
    if(chk != cmp_chk) {
        return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
    }

    /* call the proper method to read in the data */
    return loaders[calType](calData, len);
}

/**
 *  @brief  Stores a set of calibration data. 
 *          It generates a binary data set containing calibration data. 
 *          The binary data set is intended to be stored into a file.
 *
 *  @pre    MLDmpOpen()
 *
 *  @param  calData
 *              A pointer to an array of bytes to be stored.
 *  @param  length
 *              The amount of bytes available in the array.
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLStoreCal(unsigned char *calData, int length)
{
    INVENSENSE_FUNC_START;
    int ptr = 0;
    int i = 0;
    int j = 0;
    long long tmp;
    uint32_t chk;
    long bias[3];
    unsigned char state;
    union doubleToLongLong {
        double db;
        unsigned long long ll;
    } dToLL;    

    state = MLGetState();
    if ( (state != ML_STATE_DMP_STARTED) && (state != ML_STATE_DMP_OPENED) )
        return ML_ERROR_SM_IMPROPER_STATE;

    // length
    calData[0] = (unsigned char)((length>>24) & 0xff);
    calData[1] = (unsigned char)((length>>16) & 0xff);
    calData[2] = (unsigned char)((length>>8)  & 0xff);
    calData[3] = (unsigned char)( length      & 0xff);
    // calibration data format type
    calData[4] = 0;
    calData[5] = 3;
    // data
    ptr = 6;    
    for (i=0; i<BINS; i++) {
        tmp = (int)mlxData.mlTempPtrs[i];
        calData[ptr++] = (unsigned char)((tmp>>24) & 0xff);
        calData[ptr++] = (unsigned char)((tmp>>16) & 0xff);
        calData[ptr++] = (unsigned char)((tmp>>8)  & 0xff);
        calData[ptr++] = (unsigned char)( tmp      & 0xff);
    }
    
    for (i=0; i<BINS; i++) {
        tmp = (int)mlxData.mlTempValidData[i];
        calData[ptr++] = (unsigned char)((tmp>>24) & 0xff);
        calData[ptr++] = (unsigned char)((tmp>>16) & 0xff);
        calData[ptr++] = (unsigned char)((tmp>>8)  & 0xff);
        calData[ptr++] = (unsigned char)(tmp       & 0xff);
    }
    for (i=0; i<BINS; i++) {
        for (j=0; j<PTS_PER_BIN; j++) {
            tmp = (long long)(mlxData.mlTempData[i][j]*65536.0f);
            if (tmp<0) {
                tmp += 4294967296LL;
            }            
            calData[ptr++] = (unsigned char)((tmp>>24) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>16) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>8)  & 0xff);
            calData[ptr++] = (unsigned char)( tmp      & 0xff);
        }
    }
    
    for (i=0; i<BINS; i++) {
        for (j=0; j<PTS_PER_BIN; j++) {
            tmp = (long long)(mlxData.mlXGyroTempData[i][j]*65536.0f);
            if (tmp<0) {
                tmp += 4294967296LL;
            }            
            calData[ptr++] = (unsigned char)((tmp>>24) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>16) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>8)  & 0xff);
            calData[ptr++] = (unsigned char)( tmp      & 0xff);
        }
    }
    for (i=0; i<BINS; i++) {
        for (j=0; j<PTS_PER_BIN; j++) {
            tmp = (long long)(mlxData.mlYGyroTempData[i][j]*65536.0f);
            if (tmp<0) {
                tmp += 4294967296LL;
            }            
            calData[ptr++] = (unsigned char)((tmp>>24) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>16) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>8)  & 0xff);
            calData[ptr++] = (unsigned char)( tmp      & 0xff);
        }
    }
    for (i=0; i<BINS; i++) {
        for (j=0; j<PTS_PER_BIN; j++) {
            tmp = (long long)(mlxData.mlZGyroTempData[i][j]*65536.0f);
            if (tmp<0) {
                tmp += 4294967296LL;
            }            
            calData[ptr++] = (unsigned char)((tmp>>24) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>16) & 0xff);
            calData[ptr++] = (unsigned char)((tmp>>8)  & 0xff);
            calData[ptr++] = (unsigned char)( tmp      & 0xff);
        }
    }

    MLGetArray(ML_ACCEL_BIAS, bias);
 
    /* write the accel biases */
    for (i = 0; i < 3; i++) {
        uint32_t t = (uint32_t) bias[i];
        calData[ptr++] = (unsigned char) ((t>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>8)  & 0xff);
        calData[ptr++] = (unsigned char) ( t      & 0xff);
    }    

    /* write the compass calibration state */   
    calData[ptr++] = (unsigned char) (mlxData.mlGotCompassBias);
    calData[ptr++] = (unsigned char) (mlxData.mlGotInitCompassBias);
    if (mlxData.mlCompassState==SF_UNCALIBRATED) {
        calData[ptr++] = SF_UNCALIBRATED;    
    } else {
        calData[ptr++] = SF_STARTUP_SETTLE;    
    }
    for (i = 0; i < 3; i++) {
        uint32_t t = (uint32_t) mlxData.mlMagBiasError[i];
        calData[ptr++] = (unsigned char) ((t>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>8)  & 0xff);
        calData[ptr++] = (unsigned char) (t & 0xff);
    }
    for (i = 0; i < 3; i++) {
        uint32_t t = (uint32_t) mlxData.mlInitMagBias[i];
        calData[ptr++] = (unsigned char) ((t>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>8)  & 0xff);
        calData[ptr++] = (unsigned char) (t & 0xff);
    }
    for (i = 0; i < 3; i++) {
        uint32_t t = (uint32_t) mlxData.mlMagBias[i];
        calData[ptr++] = (unsigned char) ((t>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>8)  & 0xff);
        calData[ptr++] = (unsigned char) (t & 0xff);
    }
    for (i = 0; i < 18; i++) {
        uint32_t t = (uint32_t) mlxData.mlMagPeaks[i];
        calData[ptr++] = (unsigned char) ((t>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((t>>8)  & 0xff);
        calData[ptr++] = (unsigned char) (t & 0xff);
    }
    for (i = 0; i < 3; i++) {
        dToLL.db = mlxData.mlMagBiasV[i];        
        calData[ptr++] = (unsigned char) ((dToLL.ll>>56) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>48) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>40) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>32)  & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>8)  & 0xff);
        calData[ptr++] = (unsigned char) (dToLL.ll & 0xff);
    }
    for (i = 0; i < 9; i++) {
        dToLL.db = mlxData.mlMagBiasP[i];
        calData[ptr++] = (unsigned char) ((dToLL.ll>>56) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>48) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>40) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>32)  & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>24) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>16) & 0xff);
        calData[ptr++] = (unsigned char) ((dToLL.ll>>8)  & 0xff);
        calData[ptr++] = (unsigned char) (dToLL.ll & 0xff);
    }
    

    /* add a checksum */
    chk = ml_checksum(calData+ML_CAL_HDR_LEN, 
                      length - (ML_CAL_HDR_LEN+ML_CAL_CHK_LEN));
    calData[ptr++] = (unsigned char)((chk>>24) & 0xff);
    calData[ptr++] = (unsigned char)((chk>>16) & 0xff);
    calData[ptr++] = (unsigned char)((chk>>8)  & 0xff);
    calData[ptr++] = (unsigned char)( chk      & 0xff);

    return ML_SUCCESS;
}

/**
 *  @brief  Load a calibration file
 *  @pre    MLDmpOpen() must have been called.
 *  @pre    MLDmpStart() must have NOT been called.
 *  @return 0 or error code.
 */
tMLError LoadCalibration(void)
{
    unsigned char *calData;
    tMLError result;
    unsigned int length;

    result = MLSLGetCalLength(&length);
    if (result == ML_ERROR_FILE_OPEN) {
        MPL_LOGI("Calibration data not loaded\n");
        return ML_SUCCESS;
    }
    if (result || length <= 0) {
        MPL_LOGE("Could not get file calibration length - "
               "error %d - aborting\n", result);
        return result;
    }
    calData = (unsigned char*)MLOSMalloc(length);
    if (!calData) {
        MPL_LOGE("Could not allocate buffer of %d bytes - "
               "aborting\n", length);
        return ML_ERROR_MEMORY_EXAUSTED;
    }
    result = MLSLReadCal(calData, length);
    if (result) {
        MPL_LOGE("Could not read the calibration data from file - "
               "error %d - aborting\n", result);
        return result;
    }
    result = MLLoadCal(calData);
    if (result) {
        MPL_LOGE("Could not load the calibration data - "
               "error %d - aborting\n",  result);
        return result;
    }
    MLOSFree(calData);

    return ML_SUCCESS;
}

/**
 *  @brief  Store runtime calibration data to a file
 *  @pre    MLDmpStop() must have been called.
 *  @pre    MLDmpClose() must have NOT been called.
 *  @return 0 or error code.
 */
tMLError StoreCalibration(void)
{
    unsigned char *calData;
    tMLError result;
    unsigned int length;

    result = MLGetCalLength(&length);
    if (result || length <= 0) {
        MPL_LOGE("Could not get calibration data length - "
                 "error %d - aborting\n", result);
        return result;
    }
    calData = (unsigned char*)MLOSMalloc(length);
    if (!calData) {
        MPL_LOGE("Could not allocate buffer of %d bytes - "
               "aborting\n", length);
        return ML_ERROR_MEMORY_EXAUSTED;
    }
    result = MLStoreCal(calData, length);
    if (result) {
        MPL_LOGE("Could not store calibrated data on file - "
               "error %d - aborting\n", result);
        return result;
    }
    result = MLSLWriteCal(calData, length);
    if (result) {
        MPL_LOGE("Could not write calibration data - "
               "error %d\n", result);
        return result;
    }
    MLOSFree(calData);

    return ML_SUCCESS;
}

/**
 *  @}
 */

