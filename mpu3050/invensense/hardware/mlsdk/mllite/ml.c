/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 *
 * $Id: ml.c 4253 2010-12-08 18:00:39Z prao $
 *
 *****************************************************************************/

/** 
 *  @defgroup ML 
 *  @brief  Motion Library APIs.
 *          The Motion Library processes gyroscopes, accelerometers, and 
 *          compasses to provide a physical model of the movement for the 
 *          sensors.
 *          The results of this processing may be used to control objects 
 *          within a user interface environment, detect gestures, track 3D 
 *          movement for gaming applications, and analyze the blur created 
 *          due to hand movement while taking a picture.
 *
 *  @{
 *      @file   ml.c
 *      @brief  The Motion Library APIs.
 */

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#include <string.h>

#include "ml.h"
#include "mldl.h"
#include "mltypes.h"
#include "mlinclude.h"
#include "compass.h"
#include "dmpKey.h"
#include "dmpDefault.h"
#include "mlstates.h"
#include "mlFIFO.h"
#include "mlFIFOHW.h"
#include "mlMathFunc.h"
#include "mlsupervisor.h"
#include "mlmath.h"
#include "mlcontrol.h"
#include "mldl_cfg.h"
#include "mpu.h"

#include "mlos.h"
#include "mlsl.h"
#include "mlos.h"

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-ml"

#define ML_MOT_TYPE_NONE 0
#define ML_MOT_TYPE_NO_MOTION 1
#define ML_MOT_TYPE_MOTION_DETECTED 2

#define ML_MOT_STATE_MOVING 0
#define ML_MOT_STATE_NO_MOTION 1
#define ML_MOT_STATE_BIAS_IN_PROG 2

#define _mlDebug(x) //{x}

/* - Global Variables. - */
const unsigned char mlVer[] = { ML_VERSION };

tMLParams mlParams = {  
                          ML_BIAS_UPDATE_FUNC_DEFAULT,             // biasUpdateFunc
                          ML_ORIENTATION_MASK_DEFAULT,             // orientationMask
                          ML_PROCESSED_DATA_CALLBACK_DEFAULT,      // processedDataCallback
                          ML_ORIENTATION_CALLBACK_DEFAULT,         // orientationCallback
                          ML_MOTION_CALLBACK_DEFAULT,              // motionCallback
                          ML_STATE_SERIAL_CLOSED                   // starting state
                     };          

tMLXData mlxData;
void *g_mlsl_handle;

typedef struct {

    // These describe callbacks happening everythime a DMP interrupt is processed
    int_fast8_t numInterruptProcesses;
    // Array of function pointers, function being void function taking void
    tMlxdataFunction processInterruptCb[MAX_INTERRUPT_PROCESSES];

} tMLXCallbackInterrupt;     // MLX_callback_t

tMLXCallbackInterrupt mlxCallbackInterrupt; 

/* --------------- */
/* -  Functions. - */
/* --------------- */

tMLError FreescaleSensorFusion16bit(const signed char *mtx);
tMLError FreescaleSensorFusion8bit(const signed char *mtx);

/**
 *  @brief  Open the serial connection with the device. 
 *          This is the entry point of the MPL and must be 
 *          called prior to any other function call.
 *          
 *  @param  portNum     number of the COM port the device is connected to.
 *  @return ML_SUCCESS or error code.
 */
tMLError MLSerialOpen(char const * port)
{
    INVENSENSE_FUNC_START;
    tMLError result;

    result = MLStateTransition(ML_STATE_SERIAL_OPENED);
    ERROR_CHECK(result);

    result = MLSLSerialOpen(port, &g_mlsl_handle);
    if (ML_SUCCESS == result){
        result = MLDLOpen(g_mlsl_handle);
        if (ML_SUCCESS != result){
            (void)MLSLSerialClose(g_mlsl_handle);
        }
    }
    if (ML_SUCCESS != result) {
        (void)MLStateTransition(ML_STATE_SERIAL_CLOSED);
    }
    return result;
}


/**
 *  @brief  Close the serial communication.
 *          This function needs to be called explicitly to shut down 
 *          the communication with the device.  Calling MLDmpClose()
 *          won't affect the exstablished serial communication.
 *  @return ML_SUCCESS; non-zero error code otherwise.
 */
tMLError MLSerialClose(void)
{
    INVENSENSE_FUNC_START;
    tMLError result = ML_SUCCESS;

    result = MLDLClose();
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLDLClose error in %s: %d\n", __func__, result);
    }
    result = MLStateTransition(ML_STATE_SERIAL_CLOSED);
    if (ML_SUCCESS != result) {
        MPL_LOGE("State Transition Failure in %s: %d\n", __func__, result);
    }
    result = MLSLSerialClose(g_mlsl_handle);
    if (ML_SUCCESS != result) {
        MPL_LOGE("Unable to close Serial Handle %s: %d\n", __func__, result);
    }
    return result;
}


/**
 *  @brief  get the serial file handle to the device.
 *  @return the serial file handle.
 */
void *MLSerialGetHandle(void)
{
    INVENSENSE_FUNC_START;
    return g_mlsl_handle;
}


/**
 *  @brief  apply the choosen orientation and full scale range
 *          for gyroscopes, accelerometer, and compass.
 *  @return ML_SUCCESS if successful, a non-zero code otherwise.
 */
tMLError MLApplyCalibration(void)
{
    INVENSENSE_FUNC_START;
    signed char gyroCal[9]  = {0};
    signed char accelCal[9] = {0};
    signed char magCal[9]   = {0};
    float gyroScale = 2000.f;
    float accelScale = 0.f;
    float magScale = 0.f;

    tMLError result;
    int ii;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    for (ii = 0; ii < 9; ii++) {
        gyroCal[ii]  = mldl_cfg->pdata->orientation[ii];
        accelCal[ii] = mldl_cfg->pdata->accel.orientation[ii];
        magCal[ii]   = mldl_cfg->pdata->compass.orientation[ii];
    }

    switch (mldl_cfg->full_scale) {
        case MPU_FS_250DPS:
            gyroScale = 250.f;
            break;
        case MPU_FS_500DPS:
            gyroScale = 500.f;
            break;
        case MPU_FS_1000DPS:
            gyroScale = 1000.f;
            break;
        case MPU_FS_2000DPS:
            gyroScale = 2000.f;
            break;
        default:
            MPL_LOGE("Unrecognized full scale setting for gyros : %02X\n", mldl_cfg->full_scale);
            return ML_ERROR_INVALID_PARAMETER;
            break;
    }

    RANGE_FIXEDPOINT_TO_FLOAT(mldl_cfg->accel->range, accelScale);
    mlxData.mlAccelSens = (long)(accelScale/2*65536L);

    RANGE_FIXEDPOINT_TO_FLOAT(mldl_cfg->compass->range, magScale);
    mlxData.mlMagSens = (long)(magScale*32768);

    if(MLGetState()==ML_STATE_DMP_OPENED) {

        result = MLSetGyroCalibration(gyroScale, gyroCal);
        if(ML_SUCCESS != result) {
            MPL_LOGE("Unable to set Gyro Calibration\n");
            return result;
        }
        
        result = MLSetAccelCalibration(accelScale, accelCal);
        if(ML_SUCCESS != result) {
            MPL_LOGE("Unable to set Accel Calibration\n");
            return result;
        }

        result = MLSetMagCalibration(magScale, magCal);
        if(ML_SUCCESS != result) {
            MPL_LOGE("Unable to set Mag Calibration\n");
            return result;
        }
    }
    return ML_SUCCESS;
}


/**
 *  @brief  Setup the DMP to handle the accelerometer endianess.
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLApplyAccelEndian(void)
{
    INVENSENSE_FUNC_START;
    tMLError result = ML_SUCCESS;
    unsigned char regs[4] = {0};
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    int endian = mldl_cfg->accel->endian;

    if (mldl_cfg->pdata->accel.bus != EXT_SLAVE_BUS_SECONDARY) {
        endian = EXT_SLAVE_BIG_ENDIAN;
    }
    switch (endian){
    case EXT_SLAVE_FS8_BIG_ENDIAN:
        result = FreescaleSensorFusion8bit(mldl_cfg->pdata->accel.orientation);
        ERROR_CHECK(result);
        regs[0] = 0;
        regs[1] = 64;
        regs[2] = 0;
        regs[3] = 0;
        break;
    case EXT_SLAVE_FS16_BIG_ENDIAN:
        result = FreescaleSensorFusion16bit(mldl_cfg->pdata->accel.orientation);
        ERROR_CHECK(result);
        regs[0] = 0;
        regs[1] = 64;
        regs[2] = 0;
        regs[3] = 0;
        break;
    case EXT_SLAVE_LITTLE_ENDIAN:
        regs[0] = 0;
        regs[1] = 64;
        regs[2] = 0;
        regs[3] = 0;
        break;
    case EXT_SLAVE_BIG_ENDIAN:
    default:
        regs[0] = 0;
        regs[1] = 0;
        regs[2] = 64;
        regs[3] = 0;
    }

    return MLDLSetMemoryMPU(KEY_D_1_236, 4, regs);
}



/**
 * @internal
 * @brief   Initialize MLX data.  This should be called to setup the mlx
 *          output buffers before any motion processing is done.
 */
void MLXInit(void)
{
    INVENSENSE_FUNC_START;

    // Set all values to zero by default
    memset( &mlxData,     0, sizeof(tMLXData) );
    memset( &mlxCallbackInterrupt, 0, sizeof(tMLXCallbackInterrupt) );

    // Now set all the non-zero values
    mlxData.mlEngineMask = ML_BASIC;             // mlEngineMask

    mlxData.mlMagCorrection[0] = 1073741824L;                

    //Not used with the ST accelerometer
    mlxData.mlNoMotionThreshold = 20;            // noMotionThreshold
    //Not used with the ST accelerometer
    mlxData.mlMotionDuration = 1536;             // motionDuration

    mlxData.mlMotionState = ML_MOTION;           // Motion state

    mlxData.mlBiasUpdateTime = 8000;
    mlxData.mlBiasCalcTime = 2000;

    mlxData.internalMotionState = ML_MOT_STATE_MOVING;

    mlxData.startTime = MLOSGetTickCount();

    mlxData.mlMagCal[0] = 322122560L;        
    mlxData.mlMagCal[4] = 322122560L;        
    mlxData.mlMagCal[8] = 322122560L;        
    mlxData.mlMagSens = 322122560L;     // Should only change when the sensor full-scale range (FSR) is changed.

    mlxData.mlMagScale[0] = 65536L;
    mlxData.mlMagScale[1] = 65536L;
    mlxData.mlMagScale[2] = 65536L;

    mlxData.mlMagTestScale[0] = 65536L;
    mlxData.mlMagTestScale[1] = 65536L;
    mlxData.mlMagTestScale[2] = 65536L;

    mlxData.mlMagBiasError[0] = P_INIT;
    mlxData.mlMagBiasError[1] = P_INIT;
    mlxData.mlMagBiasError[2] = P_INIT;

    mlxData.mlGotNoMotionBias = 0;
    mlxData.mlGotCompassBias = 0;
    mlxData.mlCompassState = SF_UNCALIBRATED;
    mlxData.mlAccState = SF_STARTUP_SETTLE;

    mlxData.mlGotInitCompassBias = 0;
    mlxData.mlResettingCompass = 0;

    mlxData.mlInitMagBias[0] = 0;
    mlxData.mlInitMagBias[1] = 0;
    mlxData.mlInitMagBias[2] = 0;

    mlxData.mlFactoryTempComp = 0;
    mlxData.mlGotCoarseHeading = 0;

    mlxData.mlMagBiasP[0] = P_INIT;
    mlxData.mlMagBiasP[4] = P_INIT;
    mlxData.mlMagBiasP[8] = P_INIT;
}

/**
 * @brief Enables the ML_MOTION_DETECT engine.
 *
 * @note  This function replaces MLEnable(ML_MOTION_DETECT)
 *
 * @pre MLDmpOpen() or MLDmpPedometerStandAlone() must 
 *      have been called.
 *
 * @return ML_SUCCESS or non-zero error code
 */
tMLError MLEnableMotionDetect(void) 
{
    INVENSENSE_FUNC_START;
    
    if ( MLGetState() != ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    mlxData.mlEngineMask |= ML_MOTION_DETECT;
    return ML_SUCCESS;
}

/**
 * @brief Disables the ML_MOTION_DETECT engine.
 *
 * @note  This function replaces MLDisable(ML_MOTION_DETECT)
 *
 * @pre MLDmpOpen() or MLDmpPedometerStandAlone() must 
 *      have been called.
 *
 * @return ML_SUCCESS or non-zero error code
 */
tMLError MLDisableMotionDetect(void)
{
    INVENSENSE_FUNC_START;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    mlxData.mlEngineMask &= (~ML_MOTION_DETECT);
    return ML_SUCCESS;
}

/**
 * @internal
 * @brief   This registers a function to be called for each time the DMP 
 *          generates an an interrupt. 
 *          It will be called after RegisterHighRateProcess() which gets called
 *          every time the FIFO data is processed.
 *          The FIFO does not have to be on for this callback.
 * @param func Function to be called when a DMP interrupt occurs.
 * @return ML_SUCCESS or non-zero error code.
 */
tMLError RegisterProcessDmpInterrupt( tMlxdataFunction func )
{
    INVENSENSE_FUNC_START;
    // Make sure we have not filled up our number of allowable callbacks
    if ( mlxCallbackInterrupt.numInterruptProcesses <= MAX_INTERRUPT_PROCESSES-1 ) {
        int kk;
        // Make sure we haven't registered this function already
        for (kk=0; kk<mlxCallbackInterrupt.numInterruptProcesses; ++kk) {
            if ( mlxCallbackInterrupt.processInterruptCb[kk] == func ) {
                return ML_ERROR_INVALID_PARAMETER;
            }
        }
        // Add new callback
        mlxCallbackInterrupt.processInterruptCb[mlxCallbackInterrupt.numInterruptProcesses] = func;
        mlxCallbackInterrupt.numInterruptProcesses++;
        return ML_SUCCESS;
    }
    return ML_ERROR_MEMORY_EXAUSTED;
}

/**
 * @internal
 * @brief This unregisters a function to be called for each DMP interrupt.
 * @return ML_SUCCESS or non-zero error code.
 */
tMLError UnRegisterProcessDmpInterrupt( tMlxdataFunction func )
{
    INVENSENSE_FUNC_START;
    int kk,jj;
    // Make sure we haven't registered this function already
    for (kk=0; kk<mlxCallbackInterrupt.numInterruptProcesses; ++kk) {
        if ( mlxCallbackInterrupt.processInterruptCb[kk] == func ) {
            // FIXME, we may need a thread block here
            for (jj=kk+1; jj<mlxCallbackInterrupt.numInterruptProcesses; ++jj) {
                mlxCallbackInterrupt.processInterruptCb[jj-1] = mlxCallbackInterrupt.processInterruptCb[jj];
            }
            mlxCallbackInterrupt.numInterruptProcesses--;
            return ML_SUCCESS;
        }
    }
    return ML_ERROR_INVALID_PARAMETER;
}


/**
 *  @internal
 *  @brief  Run the recorded interrupt process callbacks in the event 
 *          of an interrupt.
 */
void RunProcessDmpInterruptFuncs(void)
{
    int kk;
    for (kk=0; kk<mlxCallbackInterrupt.numInterruptProcesses; ++kk) {
        if ( mlxCallbackInterrupt.processInterruptCb[kk] )
            mlxCallbackInterrupt.processInterruptCb[kk]( &mlxData );
    }
}

/** @internal
* Resets the Motion/No Motion state which should be called at startup and resume.
*/
tMLError MLResetMotion()
{
    unsigned char regs[8];
    tMLError result;

    mlxData.mlMotionState = ML_MOTION;
    mlxData.mlFlags[ML_MOTION_STATE_CHANGE] = ML_MOTION;

    regs[0] = (unsigned char)((mlxData.mlMotionDuration>>8) & 0xff);
    regs[1] = (unsigned char)( mlxData.mlMotionDuration     & 0xff);
    result = MLDLSetMemoryMPU( KEY_D_1_106, 2, regs);
    ERROR_CHECK(result);
    memset(regs,0,8);
    result = MLDLSetMemoryMPU( KEY_D_1_96, 8, regs);
    return result;
}

/**
 * @internal
 * @brief   Manually update the motion/no motion status.  This is a 
 *          convienence function for implementations that do not wish to use 
 *          MLUpdateData.  
 *          This function can be called periodically to check for the 
 *          'no motion' state and update the internal motion status and bias 
 *          calculations.
 */
tMLError MLPollMotionStatus(void)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[3] = {0};
    unsigned short motionFlag = 0;
    static long repeatBiasUpdateTime = 0;
    tMLError result;

    if (MLGetGyroPresent()==0) {
        return ML_SUCCESS;
    }

    if (mlxData.mlEngineMask & ML_MOTION_DETECT) {
        result = MLDLGetMemoryMPU(KEY_D_1_98, 2, regs);
        ERROR_CHECK(result);

        motionFlag = (unsigned short)regs[0]*256 + (unsigned short)regs[1];

        _mlDebug(MPL_LOGV("motionFlag from RAM : 0x%04X\n", motionFlag);)
        if (motionFlag == mlxData.mlMotionDuration) {
            if (mlxData.mlMotionState==ML_MOTION) {
                MLUpdateBias();
                repeatBiasUpdateTime = MLOSGetTickCount();

                regs[0] = DINAD8 + 1;
                regs[1] = DINA0C;
                regs[2] = DINAD8 + 2;
                result = MLDLSetMemoryMPU( KEY_CFG_18, 3, regs);
                ERROR_CHECK(result);

                regs[0] = 0;
                regs[1] = 5;
                result = MLDLSetMemoryMPU( KEY_D_1_106, 2, regs);
                ERROR_CHECK(result);

                //Trigger no motion callback
                mlxData.mlMotionState = ML_NO_MOTION;
                mlxData.mlGotNoMotionBias = 1;
                mlxData.mlFlags[ML_MOTION_STATE_CHANGE] = ML_NO_MOTION;
                if (mlParams.motionCallback) {
                    mlParams.motionCallback(ML_NO_MOTION);
                }
            }
        }
        if (motionFlag == 5) {
            if (mlxData.mlMotionState==ML_NO_MOTION) {
                regs[0] = DINAD8 + 2;
                regs[1] = DINA0C;
                regs[2] = DINAD8 + 1;
                result = MLDLSetMemoryMPU(KEY_CFG_18, 3, regs);
                ERROR_CHECK(result);

                regs[0] = (unsigned char)((mlxData.mlMotionDuration>>8) & 0xff);
                regs[1] = (unsigned char)(mlxData.mlMotionDuration & 0xff);
                result = MLDLSetMemoryMPU(KEY_D_1_106, 2, regs);
                ERROR_CHECK(result);

                //Trigger no motion callback
                mlxData.mlMotionState = ML_MOTION;
                mlxData.mlFlags[ML_MOTION_STATE_CHANGE] = ML_MOTION;
                if (mlParams.motionCallback) {
                    mlParams.motionCallback(ML_MOTION);
                }
            }
        }
        if (mlxData.mlMotionState==ML_NO_MOTION) {            
            if ((MLOSGetTickCount() - repeatBiasUpdateTime) > 4000) {
                MLUpdateBias(); 
                repeatBiasUpdateTime = MLOSGetTickCount();
            }
        }            
    }

    return ML_SUCCESS;
}

tMLError MLUpdateBias(void)
{
    INVENSENSE_FUNC_START;
    tMLError result;
    unsigned char i;
    unsigned char regs[12];
    long biasTmp[3],biasTmp2[3];
    long biasPrev[3] = {0};
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();


    if ((mlParams.biasUpdateFunc & ML_BIAS_FROM_NO_MOTION) && (MLGetGyroPresent()==1)) {
        int sf;
        if (mldl_cfg->trim!=0) {
            sf = 2000*131/mldl_cfg->trim;
        } else {
            sf = 2000;
        }
        //Reset bias
        regs[0] = DINAA0 + 3;
        result = MLDLSetMemoryMPU( KEY_FCFG_6, 1, regs);
        ERROR_CHECK(result);

        result = MLDLGetMemoryMPU(KEY_D_1_244, 12, regs);
        ERROR_CHECK(result);

        for ( i=0; i<3; i++ ) {
            biasTmp2[i] = (((long)regs[i*4]<<24)+((long)regs[i*4+1]<<16) + ((long)regs[i*4+2]<<8)+ ((long)regs[i*4+3]));
        }
        // Rotate bias vector by the transpose of the orientation matrix
        for ( i=0; i<3; ++i ) {
            biasTmp[i] = (long)(biasTmp2[0] * (float)mlxData.mlGyroOrient[i] / (1L<<30) +
                biasTmp2[1] * (float)mlxData.mlGyroOrient[i+3] / (1L<<30) +
                biasTmp2[2] * (float)mlxData.mlGyroOrient[i+6] / (1L<<30));
        }
        regs[0] = DINAA0 + 15;
        result = MLDLSetMemoryMPU( KEY_FCFG_6, 1, regs);
        ERROR_CHECK(result);

        result = MLSLSerialRead(MLSerialGetHandle(), MLDLGetMPUSlaveAddr(),
                                MPUREG_X_OFFS_USRH, 6, regs);
        ERROR_CHECK(result);

        for( i=0 ; i<3 ; i++ ) {
            biasTmp[i]/=1430;
            biasPrev[i] = (long)regs[2*i]*256+(int)regs[2*i+1];
            if (biasPrev[i]>32767) biasPrev[i]-=65536L;
        }
        for (i=0; i<3; i++) {
            biasTmp[i]=biasPrev[i]-biasTmp[i];
            mlxData.mlBias[i] = -biasTmp[i]*sf;
            if (biasTmp[i]<0) biasTmp[i]+=65536L;
            regs[1+2*i] = (unsigned char)((biasTmp[i]>>8) & 0xff);
            regs[1+2*i+1] = (unsigned char)(biasTmp[i] & 0xff);
        }
        regs[0] = MPUREG_X_OFFS_USRH;
        result = MLSLSerialWrite(MLSerialGetHandle(), MLDLGetMPUSlaveAddr(), 
                                 7, regs);
        ERROR_CHECK(result);
        result = MLSLSerialRead(MLSerialGetHandle(), MLDLGetMPUSlaveAddr(), 
                                MPUREG_TEMP_OUT_H, 2, regs);
        ERROR_CHECK(result);
        result = MLDLSetMemoryMPU(KEY_DMP_PREVPTAT, 2, regs);
        ERROR_CHECK(result);
        mlxData.mlGotNoMotionBias = 1;
    }
    return ML_SUCCESS;
}

/**
 * @internal
 * @brief   MLBiasStart starts the bias calculation on the MPU.
 */
void MLBiasStart(void)
{
    INVENSENSE_FUNC_START;

    mlxData.suspend = 1;
}

/**
 * @internal
 * @brief   MLBiasStop stops the bias calculation on the MPU.
 */
void MLBiasStop(void)
{
    INVENSENSE_FUNC_START;

    mlxData.suspend = 0;
}

/**
 *  @brief  MLUpdateData updates all the realtime data from the motion algorithms.
 *
 *  @pre    MLDmpOpen() or
 *          MLDmpPedometerStandAloneOpen() and MLDmpStart() 
 *          must have been called.
 *
 *  @note   Functions like MLGetArray will return the same data if
 *          MLUpdateData is not called.
 *
 * @return
 * - ML_SUCCESS
 * - Non-zero error code
 */
tMLError MLUpdateData(void)
{
    INVENSENSE_FUNC_START;
    static unsigned long polltimeNoMotion = 0;
    tMLError result=ML_SUCCESS;
    int_fast8_t got,ftry;
    uint_fast8_t mpu_interrupt;

    if ( MLGetState() != ML_STATE_DMP_STARTED )
        return ML_ERROR_SM_IMPROPER_STATE;
    
    //Process all interrupts
    //Check if interrupt was from MPU
    mpu_interrupt = MLDLGetIntTrigger(INTSRC_MPU);
    if( mpu_interrupt) {
        MLDLClearIntTrigger(INTSRC_MPU);
    }
    
    // Set the maximum number of FIFO packets we want to process
    if (mlxData.mlDataMode & ML_DATA_FIFO) {
        ftry = 100; // Large enough to process all packets
    } else {
        ftry = 1;
    }
    
    // Go and process at most ftry number of packets, probably less than ftry
    result = readAndProcessFIFO( ftry, &got );
    ERROR_CHECK(result);

    //Check if interrupt was from motion/no motion
    if ( (mlxData.mlInterruptSources & ML_INT_MOTION) ||
         ( ((mlxData.mlInterruptSources & ML_INT_MOTION) == 0) &&
           (MLOSGetTickCount() -  polltimeNoMotion) > 1000)) { 
        polltimeNoMotion = MLOSGetTickCount();
        result = MLPollMotionStatus();
    }
    
    // Take care of the callbacks that want to be notified when there was an MPU interrupt
    if( mpu_interrupt ) {
        RunProcessDmpInterruptFuncs();
    }

    result = MLDLGetFIFOStatus();
    return result;
}

/**
 *  @brief  MLCheckFlag returns the value of a flag.
 *          MLCheckFlag can be used to check a number of flags,
 *          allowing users to poll flags rather than register callback
 *          functions. If a flag is set to True when MLCheckFlag is called,
 *          the flag is automatically reset.
 *          The flags are:
 *          - ML_RAW_DATA_READY
 *          Indicates that new raw data is available.
 *          - ML_PROCESSED_DATA_READY
 *          Indicates that new processed data is available.
 *          - ML_GOT_GESTURE
 *          Indicates that a gesture has been detected by the gesture engine.
 *          - ML_MOTION_STATE_CHANGE
 *          Indicates that a change has been made from motion to no motion,
 *          or vice versa.
 *
 *  @pre    MLDmpOpen() or 
 *          MLDmpPedometerStandAloneOpen() and MLDmpStart() 
 *          must have been called.
 *
 *  @param flag The flag to check.
 *
 * @return TRUE or FALSE state of the flag
 */
int MLCheckFlag(int flag)
{
    INVENSENSE_FUNC_START;
    int flagReturn = mlxData.mlFlags[flag];

    mlxData.mlFlags[flag] = 0;
    return flagReturn;
}

/**
 *  @brief  MLGetEngines is used to determine which engines are currently enabled.
 *          MLGetEngines returns a bitwise OR of all enabled engines.
 *          The avalaible engines are:
 *          - SENSOR FUSION
 *          - MOTION DETECT
 *          - CONTROL
 *          - ORIENTATION
 *          - GESTURE
 *          <!-- CROSS AXIS --> 
 *
 *  @pre    MLDmpOpen() or 
 *          MLDmpPedometerStandAloneOpen()
 *          must have been called.
 *
 *  @return     a bit mask of the engines currently enabled.
 */
int MLGetEngines(void)
{
    INVENSENSE_FUNC_START;

    return mlxData.mlEngineMask;
}

/**
 * @brief   Set the data source for the ML.
 *          Used to switch between FIFO data and data retrieved
 *          from the MPU ram.
 *
 *  @pre    MLDmpOpen() or 
 *          MLDmpPedometerStandAloneOpen() and MLDmpStart() 
 *          must <b>NOT</b> have been called.
 *
 * @param   dataMode The data mode can be one of
 *                   - ML_DATA_FIFO
 *                   - ML_DATA_POLL
 *                   - ML_DATA_GESTURE_INT
 *                   - 0 for none.
 *
 * @return  ML_SUCCESS if successful, error code on any error.
 */
tMLError MLSetDataMode(unsigned short dataMode)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[1] = {163};

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    /* check mutual exclusivity of data modes */
    if ( (dataMode & ML_DATA_FIFO) && (dataMode & ML_DATA_POLL) ) {
        return ML_ERROR_INVALID_PARAMETER;
    }
    /* check if dataMode contains other spurious bits other than 
     * ML_DATA_FIFO and ML_DATA_POLL */
    if ( (dataMode & (~(ML_DATA_GESTURE_INT|
                        ML_DATA_FIFO|
                        ML_DATA_POLL))) != 0 ) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    mlxData.mlDataMode = dataMode;

#ifdef M_HW
    if (mlxData.mlDataMode & ML_DATA_FIFO) {
        regs[0] = DINBF8;
    } else {
        regs[0] = DINAA0 + (19 & 0xf);
    }
#else
    if (mlxData.mlDataMode & ML_DATA_FIFO) {
        regs[0] = DINA18;
    } else {
        regs[0] = DINA08;
    }
#endif
    if ( MLDLSetMemoryMPU(KEY_CFG_19, 1, regs) != ML_SUCCESS )
        return ML_ERROR_MEMORY_SET;

    if (mlxData.mlDataMode & ML_DATA_POLL) {
        regs[0] = 1;
        if ( MLDLSetMemoryMPU(KEY_D_1_179, 1, regs) != ML_SUCCESS )
            return ML_ERROR_MEMORY_SET;
    }

    if ((mlxData.mlDataMode & ML_DATA_FIFO) || 
        (mlxData.mlDataMode & ML_DATA_POLL)) {
        regs[0] = DINADD;
        if ( MLDLSetMemoryMPU(KEY_CFG_17, 1, regs) != ML_SUCCESS )
            return ML_ERROR_MEMORY_SET;
    } else {
        regs[0] = DINAA0+3;
        if ( MLDLSetMemoryMPU(KEY_CFG_17, 1, regs) != ML_SUCCESS )
            return ML_ERROR_MEMORY_SET;
    }

    return ML_SUCCESS;
}

/**
 * @brief   Get the current data source used by the ML.
 *
 *
 *  @pre    MLDmpOpen() or 
 *          MLDmpPedometerStandAloneOpen()
 *          must have been called.
 *
 * @return  The data mode, one of ML_DATA_FIFO, ML_DATA_POLL, 
 *          or ML_DATA_GESTURE_INT.
 */
int MLGetDataMode(void)
{
    INVENSENSE_FUNC_START;

    return mlxData.mlDataMode;
}

/** 
 * Enable generation of the DMP interrupt when Motion or no-motion is detected
 * 
 * @param on Boolean to turn the interrupt on or off
 * 
 * @return ML_SUCCESS or non-zero error code
 */
tMLError MLSetMotionInterrupt(unsigned char on)
{
    INVENSENSE_FUNC_START;
    tMLError result;
    unsigned char  regs[2] = {0};

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    if (on) {
        result = MLDLCfgInt(BIT_DMP_INT_EN);
        ERROR_CHECK(result);
        mlxData.mlInterruptSources |= ML_INT_MOTION;
    } else {
        mlxData.mlInterruptSources &= ~ML_INT_MOTION;
        if (!mlxData.mlInterruptSources) {
            result = MLDLCfgInt(0);
            ERROR_CHECK(result);
        }
    }

    if (on) {
        regs[0] = DINAFE;
    } else {
        regs[0] = DINAD8;
    }
    result = MLDLSetMemoryMPU(KEY_CFG_7, 1, regs);
    ERROR_CHECK(result);
    return result;
}

/** 
 * Enable generation of the DMP interrupt when a FIFO packet is ready
 * 
 * @param on Boolean to turn the interrupt on or off
 * 
 * @return ML_SUCCESS or non-zero error code
 */
tMLError MLSetFifoInterrupt(unsigned char on)
{
    INVENSENSE_FUNC_START;
    tMLError result;
    unsigned char  regs[2] = {0};

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    if (on) {
        result = MLDLCfgInt(BIT_DMP_INT_EN);
        ERROR_CHECK(result);
        mlxData.mlInterruptSources |= ML_INT_FIFO;
    } else {
        mlxData.mlInterruptSources &= ~ML_INT_FIFO;
        if (!mlxData.mlInterruptSources) {
            result = MLDLCfgInt(0);
            ERROR_CHECK(result);
        }
    }

    if (on) {
        regs[0] = DINAFE;
    } else {
        regs[0] = DINAD8;
    }
    result = MLDLSetMemoryMPU(KEY_CFG_6, 1, regs);
    ERROR_CHECK(result);
    return result;
}

/**
 * @brief   Get the current set of DMP interrupt sources.
 *          These interrupts are generated by the DMP and can be
 *          routed to the MPU interrupt line via internal 
 *          settings.
 *
 *  @pre    MLDmpOpen() or 
 *          MLDmpPedometerStandAloneOpen()
 *          must have been called.
 *
 * @return  Currently enabled interrupt sources.  The possible interrupts are:
 *          - ML_INT_FIFO,
 *          - ML_INT_MOTION, 
 *          - ML_INT_GESTURE, and
 *          - ML_INT_TAP
 */
int MLGetInterrupts(void)
{
    INVENSENSE_FUNC_START;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return 0; // error

    return mlxData.mlInterruptSources;
}

/**
 *  @brief  MLGetArray is used to get an array of processed motion sensor data.
 *          MLGetArray can be used to retrieve various data sets. Certain data
 *          sets require functions to be enabled using MLEnable in order to be
 *          valid.
 *
 *          The available data sets are:
 *
 *          - ML_ROTATION_MATRIX
 *          - ML_QUATERNION
 *          - ML_EULER_ANGLES_X
 *          - ML_EULER_ANGLES_Y
 *          - ML_EULER_ANGLES_Z
 *          - ML_EULER_ANGLES
 *          - ML_LINEAR_ACCELERATION
 *          - ML_LINEAR_ACCELERATION_WORLD
 *          - ML_GRAVITY
 *          - ML_ANGULAR_VELOCITY
 *          - ML_RAW_DATA
 *          - ML_GYROS
 *          - ML_ACCELS
 *          - ML_MAGNETOMETER
 *          - ML_GYRO_BIAS
 *          - ML_ACCEL_BIAS
 *          - ML_MAG_BIAS 
 *          - ML_HEADING
 *          - ML_MAG_BIAS_ERROR
 *
 *          Please refer to the documentation of MLGetFloatArray() for a 
 *          description of these data sets.
 *
 *  @pre    MLDmpOpen() or MLDmpPedometerStandAloneOpen()
 *          must have been called.
 *
 *  @param  dataSet     
 *              A constant specifying an array of data processed by the
 *              motion processor.
 *  @param  data        
 *              A pointer to an array to be passed back to the user.
 *              <b>Must be 9 cells long at least</b>.
 *
 *  @return Zero if the command is successful; an ML error code otherwise.
 */
tMLError MLGetArray(int dataSet, long* data)
{
    INVENSENSE_FUNC_START;

    float  rotMatrix[9] = {
                           0.0f, 0.0f, 0.0f,
                           0.0f, 0.0f, 0.0f,
                           0.0f, 0.0f, 0.0f
                          };
    float tmp;
    tMLError result = ML_SUCCESS;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    if (NULL == data) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    _mlDebug(MPL_LOGV("MLGetArray sizeof(data): %d\n", sizeof(data)););

    switch (dataSet) {
        case ML_GYROS:
            result = FIFOGetGyro( data );
            break;
        case ML_ACCELS:
            result = FIFOGetAccel( data );
            break;
        case ML_TEMPERATURE:
            result = FIFOGetTemperature( data );
            break;
        case ML_BIAS_UNCERTAINTY:   
            data[0] = FIFOGetBiasUncertainty();
            break;
        case ML_ROTATION_MATRIX:
            {
                long qdata[4];
                FIFOGetQuaternion(qdata);
                quaternionToRotationMatrix( qdata, data );
            }
            break;
        case ML_QUATERNION:
            result = FIFOGetQuaternion(data);
            break;
        case ML_LINEAR_ACCELERATION:
            result = FIFOGetLinearAccel(data);
            break;
        case ML_LINEAR_ACCELERATION_WORLD:
            result = FIFOGetLinearAccelWorld(data);
            break;
        case ML_GRAVITY:
            result = FIFOGetGravBody( data );
            break;
        case ML_ANGULAR_VELOCITY:
            data[0] = mlxData.mlAngVBody[0];
            data[1] = mlxData.mlAngVBody[1];
            data[2] = mlxData.mlAngVBody[2];
            break;
        case ML_EULER_ANGLES:
            MLGetFloatArray(ML_ROTATION_MATRIX, rotMatrix);
            tmp = rotMatrix[6];
            if (tmp>1.0f) {
                tmp = 1.0f;
            }
            if (tmp<-1.0f) {
                tmp = -1.0f;
            }
            data[0] = (long)((float)((double)atan2(rotMatrix[7], rotMatrix[8])*57.29577951308)*65536L);
            data[1] = (long)((float)((double)asin(tmp)*57.29577951308)*65536L);
            data[2] = (long)((float)((double)atan2(rotMatrix[3], rotMatrix[0])*57.29577951308)*65536L);
            break;
        case ML_EULER_ANGLES_X:
            MLGetFloatArray(ML_ROTATION_MATRIX, rotMatrix);
            tmp = rotMatrix[6];
            if (tmp>1.0f) {
                tmp = 1.0f;
            }
            if (tmp<-1.0f) {
                tmp = -1.0f;
            }
            data[0] = (long)((float)((double)atan2(rotMatrix[7], rotMatrix[8])*57.29577951308)*65536L);
            data[1] = (long)((float)((double)asin(tmp)*57.29577951308)*65536L);
            data[2] = (long)((float)((double)atan2(rotMatrix[3], rotMatrix[0])*57.29577951308)*65536L);
            break;
        case ML_EULER_ANGLES_Y:
            MLGetFloatArray(ML_ROTATION_MATRIX, rotMatrix);
            tmp = rotMatrix[7];
            if (tmp>1.0f) {
                tmp = 1.0f;
            }
            if (tmp<-1.0f) {
                tmp = -1.0f;
            }
            data[0] = (long)((float)((double)atan2(rotMatrix[8], rotMatrix[6])*57.29577951308f)*65536L);
            data[1] = (long)((float)((double)asin(tmp)*57.29577951308)*65536L);
            data[2] = (long)((float)((double)atan2(rotMatrix[4], rotMatrix[1])*57.29577951308f)*65536L);
            break;
        case ML_EULER_ANGLES_Z:
            MLGetFloatArray(ML_ROTATION_MATRIX, rotMatrix);
            tmp = rotMatrix[8];
            if (tmp>1.0f) {
                tmp = 1.0f;
            }
            if (tmp<-1.0f) {
                tmp = -1.0f;
            }
            data[0] = (long)((float)((double)atan2(rotMatrix[6], rotMatrix[7])*57.29577951308)*65536L);
            data[1] = (long)((float)((double)asin(tmp)*57.29577951308)*65536L);
            data[2] = (long)((float)((double)atan2(rotMatrix[5], rotMatrix[2])*57.29577951308)*65536L);
            break;
        case ML_GYRO_TEMP_SLOPE:
            data[0] = mlxData.mlTempSlope[0];
            data[1] = mlxData.mlTempSlope[1];
            data[2] = mlxData.mlTempSlope[2];
            break;
        case ML_GYRO_BIAS:
            data[0] = mlxData.mlBias[0];
            data[1] = mlxData.mlBias[1];
            data[2] = mlxData.mlBias[2];
            break;
        case ML_ACCEL_BIAS:
            data[0] = mlxData.mlBias[3];
            data[1] = mlxData.mlBias[4];
            data[2] = mlxData.mlBias[5];
            break;
        case ML_MAG_BIAS:                        
            data[0] = mlxData.mlMagBias[0] + (long)((long long)mlxData.mlInitMagBias[0]* mlxData.mlMagSens / 16834);                     
            data[1] = mlxData.mlMagBias[1] + (long)((long long)mlxData.mlInitMagBias[1]* mlxData.mlMagSens / 16834);                     
            data[2] = mlxData.mlMagBias[2] + (long)((long long)mlxData.mlInitMagBias[2]* mlxData.mlMagSens / 16834);                     
            break; 
        case ML_RAW_DATA:
            result = FIFOGetSensorData( data );
            break;
        case ML_MAG_RAW_DATA:
            data[0] = mlxData.mlMagSensorData[0];
            data[1] = mlxData.mlMagSensorData[1];
            data[2] = mlxData.mlMagSensorData[2];
            break;      
        case ML_MAGNETOMETER:           
            data[0] = mlxData.mlMagCalibratedData[0];           
            data[1] = mlxData.mlMagCalibratedData[1];           
            data[2] = mlxData.mlMagCalibratedData[2];                  
            break; 
        case ML_HEADING:
            MLGetFloatArray(ML_ROTATION_MATRIX, rotMatrix);
            if ((rotMatrix[7]<0.707) && (rotMatrix[7]>-0.707)) {
                tmp = (float)((double)atan2(rotMatrix[4], rotMatrix[1])*57.29577951308 - 90.0f);                
            } else {
                tmp = (float)((double)atan2(rotMatrix[5], rotMatrix[2])*57.29577951308 + 90.0f);                
            }
            if (tmp<0) {
                tmp += 360.0f;
            }
            data[0] = (long)((360-tmp)*65536.0f);
            break;
        case ML_GYRO_CALIBRATION_MATRIX:
            data[0] = mlxData.mlGyroCal[0];
            data[1] = mlxData.mlGyroCal[1];
            data[2] = mlxData.mlGyroCal[2];
            data[3] = mlxData.mlGyroCal[3];
            data[4] = mlxData.mlGyroCal[4];
            data[5] = mlxData.mlGyroCal[5];
            data[6] = mlxData.mlGyroCal[6];
            data[7] = mlxData.mlGyroCal[7];
            data[8] = mlxData.mlGyroCal[8];
            break;
        case ML_ACCEL_CALIBRATION_MATRIX:
            data[0] = mlxData.mlAccelCal[0];
            data[1] = mlxData.mlAccelCal[1];
            data[2] = mlxData.mlAccelCal[2];
            data[3] = mlxData.mlAccelCal[3];
            data[4] = mlxData.mlAccelCal[4];
            data[5] = mlxData.mlAccelCal[5];
            data[6] = mlxData.mlAccelCal[6];
            data[7] = mlxData.mlAccelCal[7];
            data[8] = mlxData.mlAccelCal[8];
            break;
        case ML_MAG_CALIBRATION_MATRIX:            
            data[0] = mlxData.mlMagCal[0];
            data[1] = mlxData.mlMagCal[1];
            data[2] = mlxData.mlMagCal[2];
            data[3] = mlxData.mlMagCal[3];
            data[4] = mlxData.mlMagCal[4];
            data[5] = mlxData.mlMagCal[5];
            data[6] = mlxData.mlMagCal[6];
            data[7] = mlxData.mlMagCal[7];                     
            data[8] = mlxData.mlMagCal[8];                                        
            break;    
        case ML_MAG_BIAS_ERROR:
            data[0] = mlxData.mlMagBiasError[0]; 
            data[1] = mlxData.mlMagBiasError[1];
            data[2] = mlxData.mlMagBiasError[2];
            break;
        default:            
            return ML_ERROR_INVALID_PARAMETER;
            break;
    }

    return result;

}


/**
 *  @brief  MLGetFloatArray is used to get an array of processed motion sensor 
 *          data. MLGetArray can be used to retrieve various data sets. 
 *          Certain data sets require functions to be enabled using MLEnable 
 *          in order to be valid.
 *
 *          The available data sets are:
 *
 *          - ML_ROTATION_MATRIX :
 *          Returns an array of nine data points representing the rotation 
 *          matrix generated from all available sensors. 
 *          This requires that ML_SENSOR_FUSION be enabled.
 *          The array format will be R11, R12, R13, R21, R22, R23, R31, R32, 
 *          R33, representing the matrix:
 *          <center>R11 R12 R13</center>
 *          <center>R21 R22 R23</center>
 *          <center>R31 R32 R33</center>
 *          <b>Please refer to the "9-Axis Sensor Fusion Application Note" document, 
 *          section 7 "Sensor Fusion Output", for details regarding rotation 
 *          matrix output</b>.
 *
 *          - ML_QUATERNION :
 *          Returns an array of four data points representing the quaternion 
 *          generated from all available sensors. 
 *          This requires that ML_SENSOR_FUSION be enabled.
 *
 *          - ML_EULER_ANGLES_X :
 *          Returns an array of three data points representing roll, pitch, and 
 *          yaw using the X axis of the gyroscope, accelerometer, and compass as 
 *          reference axis. 
 *          This is typically the convention used for mobile devices where the X
 *          axis is the width of the screen, Y axis is the height, and Z the 
 *          depth. In this case roll is defined as the rotation around the X 
 *          axis of the device.
 *          The euler angles convention for this output is the following:
 *          <TABLE>
 *          <TR><TD><b>EULER ANGLE</b></TD><TD><b>ROTATION AROUND</b></TD></TR>
 *          <TR><TD>roll              </TD><TD>X axis                </TD></TR>
 *          <TR><TD>pitch             </TD><TD>Y axis                </TD></TR>
 *          <TR><TD>yaw               </TD><TD>Z axis                </TD></TR>
 *          </TABLE>
 *          ML_EULER_ANGLES_X corresponds to the ML_EULER_ANGLES output and is 
 *          therefore the default convention.
 *
 *          - ML_EULER_ANGLES_Y :
 *          Returns an array of three data points representing roll, pitch, and 
 *          yaw using the Y axis of the gyroscope, accelerometer, and compass as 
 *          reference axis. 
 *          This convention is typically used in augmented reality applications, 
 *          where roll is defined as the rotation around the axis along the 
 *          height of the screen of a mobile device, namely the Y axis.
 *          The euler angles convention for this output is the following:
 *          <TABLE>
 *          <TR><TD><b>EULER ANGLE</b></TD><TD><b>ROTATION AROUND</b></TD></TR>
 *          <TR><TD>roll              </TD><TD>Y axis                </TD></TR>
 *          <TR><TD>pitch             </TD><TD>X axis                </TD></TR>
 *          <TR><TD>yaw               </TD><TD>Z axis                </TD></TR>
 *          </TABLE>
 *
 *          - ML_EULER_ANGLES_Z :
 *          Returns an array of three data points representing roll, pitch, and 
 *          yaw using the Z axis of the gyroscope, accelerometer, and compass as 
 *          reference axis. 
 *          This convention is mostly used in application involving the use 
 *          of a camera, typically placed on the back of a mobile device, that 
 *          is along the Z axis.  In this convention roll is defined as the 
 *          rotation around the Z axis.
 *          The euler angles convention for this output is the following:
 *          <TABLE>
 *          <TR><TD><b>EULER ANGLE</b></TD><TD><b>ROTATION AROUND</b></TD></TR>
 *          <TR><TD>roll              </TD><TD>Z axis                </TD></TR>
 *          <TR><TD>pitch             </TD><TD>X axis                </TD></TR>
 *          <TR><TD>yaw               </TD><TD>Y axis                </TD></TR>
 *          </TABLE>
 *
 *          - ML_EULER_ANGLES :
 *          Returns an array of three data points representing roll, pitch, and 
 *          yaw corresponding to the ML_EULER_ANGLES_X output and it is 
 *          therefore the default convention for Euler angles.
 *          Please refer to the ML_EULER_ANGLES_X for a detailed description.
 *
 *
 *          - ML_LINEAR_ACCELERATION :
 *          Returns an array of three data points representing the linear 
 *          acceleration as derived from both gyroscopes and accelerometers. 
 *          This requires that ML_SENSOR_FUSION be enabled.
 *
 *          - ML_LINEAR_ACCELERATION_WORLD :
 *          Returns an array of three data points representing the linear 
 *          acceleration in world coordinates, as derived from both gyroscopes 
 *          and accelerometers.
 *          This requires that ML_SENSOR_FUSION be enabled.
 *
 *          - ML_GRAVITY :
 *          Returns an array of three data points representing the direction 
 *          of gravity in body coordinates, as derived from both gyroscopes 
 *          and accelerometers.
 *          This requires that ML_SENSOR_FUSION be enabled.
 *
 *          - ML_ANGULAR_VELOCITY :
 *          Returns an array of three data points representing the angular 
 *          velocity as derived from both gyroscopes and accelerometers. This 
 *          requires that ML_SENSOR_FUSION be enabled.
 *
 *          - ML_RAW_DATA :
 *          Returns an array of nine data points representing raw sensor data of 
 *          the gyroscope X, Y, Z, accelerometer X, Y, Z, and compass X, Y, Z 
 *          values.
 *
 *          - ML_GYROS :
 *          Returns an array of three data points representing the X gyroscope,
 *          Y gyroscope, and Z gyroscope values.
 *
 *          - ML_ACCELS :
 *          Returns an array of three data points representing the X 
 *          accelerometer, Y accelerometer, and Z accelerometer values.
 *
 *          - ML_MAGNETOMETER :
 *          Returns an array of three data points representing the compass
 *          X, Y, and Z values.
 *
 *          - ML_GYRO_BIAS :
 *          Returns an array of three data points representing the gyroscope 
 *          biases.
 *
 *          - ML_ACCEL_BIAS :
 *          Returns an array of three data points representing the 
 *          accelerometer biases.
 *
 *          - ML_MAG_BIAS :
 *          Returns an array of three data points representing the compass biases.
 *
 *          - ML_GYRO_CALIBRATION_MATRIX :
 *          Returns an array of nine data points representing the calibration 
 *          matrix for the gyroscopes:
 *          <center>C11 C12 C13</center>
 *          <center>C21 C22 C23</center>
 *          <center>C31 C32 C33</center>
 *
 *          - ML_ACCEL_CALIBRATION_MATRIX :
 *          Returns an array of nine data points representing the calibration 
 *          matrix for the accelerometers:
 *          <center>C11 C12 C13</center>
 *          <center>C21 C22 C23</center>
 *          <center>C31 C32 C33</center>
 *
 *          - ML_MAG_CALIBRATION_MATRIX :
 *          Returns an array of nine data points representing the calibration 
 *          matrix for the compass:
 *          <center>C11 C12 C13</center>
 *          <center>C21 C22 C23</center>
 *          <center>C31 C32 C33</center>
 *
 *          - ML_HEADING : 
 *          Returns a single number representing the heading of the device relative to the
 *          Earth, in which 0 represents North, 90 degrees represents East, and so on. 
 *          The heading is defined as the direction of the +Y axis if the Y axis is horizontal,
 *          and otherwise the direction of the -Z axis.
 *
 *          - ML_MAG_BIAS_ERROR :
 *          Returns an array of three numbers representing the current estimated
 *          error in the compass biases. These numbers are unitless and serve
 *          as rough estimates in which numbers less than 100 typically represent
 *          reasonably well calibrated compass axes.
 *
 *  @pre    MLDmpOpen() or MLDmpPedometerStandAloneOpen()
 *          must have been called.
 *
 *  @param  dataSet     
 *              A constant specifying an array of data processed by 
 *              the motion processor.
 *  @param  data        
 *              A pointer to an array to be passed back to the user.
 *              <b>Must be 9 cells long at least</b>.
 *
 *  @return ML_SUCCESS if the command is successful; an error code otherwise.
 */
tMLError MLGetFloatArray(int dataSet, float *data)
{
    INVENSENSE_FUNC_START;

    float  rotMatrix[9] = {
                           0.0f, 0.0f, 0.0f,
                           0.0f, 0.0f, 0.0f,
                           0.0f, 0.0f, 0.0f
                          };
    float tmp;
    long ldata[6];
    tMLError result = ML_SUCCESS;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    if (NULL == data) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    switch (dataSet) {
        case ML_ANGULAR_VELOCITY:
        case ML_GYROS:
            result = FIFOGetGyro( ldata );
            data[0] = (float)ldata[0]/65536.0f;
            data[1] = (float)ldata[1]/65536.0f;
            data[2] = (float)ldata[2]/65536.0f;
            break;
        case ML_ACCELS:
            result = FIFOGetAccelFloat( data );
            break;
        case ML_TEMPERATURE:
            result = FIFOGetTemperature( ldata );
            data[0] = (float)ldata[0]/65536.0f;
            break;
        case ML_BIAS_UNCERTAINTY:
            data[0] = (float)FIFOGetBiasUncertainty()/65536.0f;
            break;
        case ML_ROTATION_MATRIX:
            {
                long qdata[4],rdata[9];
                FIFOGetQuaternion(qdata);
                quaternionToRotationMatrix( qdata, rdata );
                data[0] = (float)rdata[0]/1073741824.0f;
                data[1] = (float)rdata[1]/1073741824.0f;
                data[2] = (float)rdata[2]/1073741824.0f;
                data[3] = (float)rdata[3]/1073741824.0f;
                data[4] = (float)rdata[4]/1073741824.0f;
                data[5] = (float)rdata[5]/1073741824.0f;
                data[6] = (float)rdata[6]/1073741824.0f;
                data[7] = (float)rdata[7]/1073741824.0f;
                data[8] = (float)rdata[8]/1073741824.0f;
                break;
            }
        case ML_QUATERNION:
            result = FIFOGetQuaternion( ldata );
            data[0] = (float)ldata[0]/1073741824.0f;
            data[1] = (float)ldata[1]/1073741824.0f;
            data[2] = (float)ldata[2]/1073741824.0f;
            data[3] = (float)ldata[3]/1073741824.0f;
            break;
        case ML_LINEAR_ACCELERATION:
            result = FIFOGetLinearAccel( ldata );
            data[0] = (float)ldata[0]/65536.0f;
            data[1] = (float)ldata[1]/65536.0f;
            data[2] = (float)ldata[2]/65536.0f;
            break;
        case ML_LINEAR_ACCELERATION_WORLD:
            result = FIFOGetLinearAccelWorld( ldata );
            data[0] = (float)ldata[0]/65536.0f;
            data[1] = (float)ldata[1]/65536.0f;
            data[2] = (float)ldata[2]/65536.0f;
            break;
        case ML_GRAVITY:
            result = FIFOGetGravBody( ldata );
            data[0] = (float)ldata[0]/65536.0f;
            data[1] = (float)ldata[1]/65536.0f;
            data[2] = (float)ldata[2]/65536.0f;
            break;
        case ML_GYRO_CALIBRATION_MATRIX:
            data[0] = (float)mlxData.mlGyroOrient[0]/1073741824.0f;
            data[1] = (float)mlxData.mlGyroOrient[1]/1073741824.0f;
            data[2] = (float)mlxData.mlGyroOrient[2]/1073741824.0f;
            data[3] = (float)mlxData.mlGyroOrient[3]/1073741824.0f;
            data[4] = (float)mlxData.mlGyroOrient[4]/1073741824.0f;
            data[5] = (float)mlxData.mlGyroOrient[5]/1073741824.0f;
            data[6] = (float)mlxData.mlGyroOrient[6]/1073741824.0f;
            data[7] = (float)mlxData.mlGyroOrient[7]/1073741824.0f;
            data[8] = (float)mlxData.mlGyroOrient[8]/1073741824.0f;
            break;
        case ML_ACCEL_CALIBRATION_MATRIX:
            data[0] = (float)mlxData.mlAccelCal[0]/1073741824.0f;
            data[1] = (float)mlxData.mlAccelCal[1]/1073741824.0f;
            data[2] = (float)mlxData.mlAccelCal[2]/1073741824.0f;
            data[3] = (float)mlxData.mlAccelCal[3]/1073741824.0f;
            data[4] = (float)mlxData.mlAccelCal[4]/1073741824.0f;
            data[5] = (float)mlxData.mlAccelCal[5]/1073741824.0f;
            data[6] = (float)mlxData.mlAccelCal[6]/1073741824.0f;
            data[7] = (float)mlxData.mlAccelCal[7]/1073741824.0f;
            data[8] = (float)mlxData.mlAccelCal[8]/1073741824.0f;
            break;
        case ML_MAG_CALIBRATION_MATRIX:            
            data[0] = (float)mlxData.mlMagCal[0]/1073741824.0f;                     
            data[1] = (float)mlxData.mlMagCal[1]/1073741824.0f;                     
            data[2] = (float)mlxData.mlMagCal[2]/1073741824.0f;                     
            data[3] = (float)mlxData.mlMagCal[3]/1073741824.0f;                     
            data[4] = (float)mlxData.mlMagCal[4]/1073741824.0f;                     
            data[5] = (float)mlxData.mlMagCal[5]/1073741824.0f;                     
            data[6] = (float)mlxData.mlMagCal[6]/1073741824.0f;                     
            data[7] = (float)mlxData.mlMagCal[7]/1073741824.0f;                     
            data[8] = (float)mlxData.mlMagCal[8]/1073741824.0f;                                       
            break;              
        case ML_GYRO_TEMP_SLOPE:            
            data[0] = (float)mlxData.mlTempSlope[0]/65536.0f;
            data[1] = (float)mlxData.mlTempSlope[1]/65536.0f;
            data[2] = (float)mlxData.mlTempSlope[2]/65536.0f;
            break;
        case ML_GYRO_BIAS:
            data[0] = (float)mlxData.mlBias[0]/65536.0f;
            data[1] = (float)mlxData.mlBias[1]/65536.0f;
            data[2] = (float)mlxData.mlBias[2]/65536.0f;
            break;
        case ML_ACCEL_BIAS:
            data[0] = (float)mlxData.mlBias[3]/65536.0f;
            data[1] = (float)mlxData.mlBias[4]/65536.0f;
            data[2] = (float)mlxData.mlBias[5]/65536.0f;
            break;
        case ML_MAG_BIAS:              
            data[0] = ((float)(mlxData.mlMagBias[0] + (long)((long long)mlxData.mlInitMagBias[0]* mlxData.mlMagSens / 16834)))/65536.0f;                     
            data[1] = ((float)(mlxData.mlMagBias[1] + (long)((long long)mlxData.mlInitMagBias[1]* mlxData.mlMagSens / 16834)))/65536.0f;                     
            data[2] = ((float)(mlxData.mlMagBias[2] + (long)((long long)mlxData.mlInitMagBias[2]* mlxData.mlMagSens / 16834)))/65536.0f;                      
            break;              
        case ML_RAW_DATA:
            result = FIFOGetSensorData( ldata );
            data[0] = (float)ldata[0];
            data[1] = (float)ldata[1];
            data[2] = (float)ldata[2];
            data[3] = (float)ldata[3];
            data[4] = (float)ldata[4];
            data[5] = (float)ldata[5];
            data[6] = (float)mlxData.mlMagSensorData[0];
            data[7] = (float)mlxData.mlMagSensorData[1];
            data[8] = (float)mlxData.mlMagSensorData[2];
            break;
        case ML_EULER_ANGLES:
        case ML_EULER_ANGLES_X:
            result = MLGetFloatArray(ML_ROTATION_MATRIX, rotMatrix);
            tmp = rotMatrix[6];
            if (tmp>1.0f) {
                tmp = 1.0f;
            }
            if (tmp<-1.0f) {
                tmp = -1.0f;
            }
            data[0] = (float)((double)atan2(rotMatrix[7], rotMatrix[8])*57.29577951308);
            data[1] = (float)((double)asin(tmp)*57.29577951308);
            data[2] = (float)((double)atan2(rotMatrix[3], rotMatrix[0])*57.29577951308);
            break;
        case ML_EULER_ANGLES_Y:
            result = MLGetFloatArray(ML_ROTATION_MATRIX, rotMatrix);
            tmp = rotMatrix[7];
            if (tmp>1.0f) {
                tmp = 1.0f;
            }
            if (tmp<-1.0f) {
                tmp = -1.0f;
            }
            data[0] = (float)((double)atan2(rotMatrix[8], rotMatrix[6])*57.29577951308);
            data[1] = (float)((double)asin(tmp)*57.29577951308);
            data[2] = (float)((double)atan2(rotMatrix[4], rotMatrix[1])*57.29577951308);
            break;
        case ML_EULER_ANGLES_Z:
            result = MLGetFloatArray(ML_ROTATION_MATRIX, rotMatrix);
            tmp = rotMatrix[8];
            if (tmp>1.0f) {
                tmp = 1.0f;
            }
            if (tmp<-1.0f) {
                tmp = -1.0f;
            }
            data[0] = (float)((double)atan2(rotMatrix[6], rotMatrix[7])*57.29577951308);
            data[1] = (float)((double)asin(tmp)*57.29577951308);
            data[2] = (float)((double)atan2(rotMatrix[5], rotMatrix[2])*57.29577951308);
            break;
        case ML_MAG_RAW_DATA:            
            data[0] = (float)mlxData.mlMagSensorData[0];            
            data[1] = (float)mlxData.mlMagSensorData[1];            
            data[2] = (float)mlxData.mlMagSensorData[2];            
            break;      
        case ML_MAGNETOMETER:            
            data[0] = (float)mlxData.mlMagCalibratedData[0]/65536.0f;            
            data[1] = (float)mlxData.mlMagCalibratedData[1]/65536.0f;            
            data[2] = (float)mlxData.mlMagCalibratedData[2]/65536.0f;            
            break;      
        case ML_HEADING:         
            MLGetFloatArray(ML_ROTATION_MATRIX, rotMatrix);
            if ((rotMatrix[7]<0.707) && (rotMatrix[7]>-0.707)) {
                tmp = (float)((double)atan2(rotMatrix[4], rotMatrix[1])*57.29577951308 - 90.0f);                
            } else {
                tmp = (float)((double)atan2(rotMatrix[5], rotMatrix[2])*57.29577951308 + 90.0f);                
            }            
            if (tmp<0) {
                tmp += 360.0f;
            }
            data[0] = 360-tmp;
            break;
        case ML_MAG_BIAS_ERROR:
            data[0] = (float)mlxData.mlMagBiasError[0]; 
            data[1] = (float)mlxData.mlMagBiasError[1];
            data[2] = (float)mlxData.mlMagBiasError[2];
            break;

        default:
            return ML_ERROR_INVALID_PARAMETER;
            break;
    }

    return result;
}


/**
 *  @brief  used to set an array of motion sensor data.
 *
 *          For more details about the use of the data sets
 *          please refer to the documentation of MLSetFloatArray().
 *
 *  @pre    MLDmpOpen() or 
 *          MLDmpPedometerStandAloneOpen() 
 *  @pre    MLDmpStart() must <b>NOT</b> have been called.
 *
 *  @param  dataSet     A constant specifying an array of data.
 *  @param  data        A pointer to an array to be copied from the user.
 *
 *  @return ML_SUCCESS if successful; a non-zero error code otherwise.
 */
tMLError MLSetArray(int dataSet, long* data)
{
    INVENSENSE_FUNC_START;
    tMLError result;
    unsigned char regs[12] = {0};
    unsigned char maxVal = 0;
    unsigned char tmpPtr = 0;
    unsigned char tmpSign = 0;
    unsigned char i = 0;
    unsigned char j = 0;
    long sf = 0;
    long biasTmp;
    struct mldl_cfg *mldl_cfg = MLDLGetCfg();    

    switch (dataSet) {
        case ML_GYRO_BIAS:          // internal
            mlxData.mlBias[0] = data[0];            
            mlxData.mlBias[1] = data[1];
            mlxData.mlBias[2] = data[2];            
            if (mldl_cfg->trim!=0) {
                sf = 2000*131/mldl_cfg->trim;
            } else {
                sf = 2000;
            }
            for (i=0; i<3; i++) {
                biasTmp = -mlxData.mlBias[i]/sf;
                if (biasTmp<0) biasTmp+=65536L;
                regs[1+2*i]   = (unsigned char)((biasTmp>>8) & 0xff);
                regs[1+2*i+1] = (unsigned char)( biasTmp     & 0xff);
            }
            regs[0] = MPUREG_X_OFFS_USRH;
            result = MLSLSerialWrite(MLSerialGetHandle(), MLDLGetMPUSlaveAddr(),
                                     7, regs);
            ERROR_CHECK(result);
            break;

        case ML_ACCEL_BIAS:         // internal
            mlxData.mlBias[3] = data[0];
            mlxData.mlBias[4] = data[1];
            mlxData.mlBias[5] = data[2];            
            for (i=0; i<3; i++) {
                if (mlxData.mlAccelSens!=0) {
                    long long tmp64 = 0;                        
                    for (j=0; j<3; j++) {
                        tmp64 += (long long)mlxData.mlBias[j+3]*mlxData.mlAccelCal[i*3+j];                                             
                    }                   
                    biasTmp = (long)(tmp64/mlxData.mlAccelSens);                      
                    biasTmp = biasTmp*8192/mlxData.mlAccelSens;                    
                } else {
                    biasTmp = 0;
                }
                if (biasTmp<0) biasTmp+=65536L;                
                regs[2*i] = (unsigned char)(biasTmp/256);
                regs[2*i+1] = (unsigned char)(biasTmp%256);
            }            
            result = MLDLSetMemoryMPU(KEY_D_1_8,  2, &regs[0] );  ERROR_CHECK(result);
            result = MLDLSetMemoryMPU(KEY_D_1_10, 2, &regs[2] );  ERROR_CHECK(result);
            result = MLDLSetMemoryMPU(KEY_D_1_2,  2, &regs[4] );  ERROR_CHECK(result);
            break;

        case ML_MAG_BIAS:
            CompassSetBias( data );                        
            mlxData.mlInitMagBias[0] = 0;
            mlxData.mlInitMagBias[1] = 0;
            mlxData.mlInitMagBias[2] = 0;
            mlxData.mlGotCompassBias = 1;
            mlxData.mlGotInitCompassBias = 1;
            mlxData.mlCompassState = SF_STARTUP_SETTLE;
            break;      

        case ML_GYRO_TEMP_SLOPE:         // internal    
            mlxData.mlFactoryTempComp = 1;
            mlxData.mlTempSlope[0] = data[0];            
            mlxData.mlTempSlope[1] = data[1];
            mlxData.mlTempSlope[2] = data[2];
            for (i = 0; i < MPU_NUM_AXES; i++) {
                sf = -mlxData.mlTempSlope[i]/1118;
                if (sf>127) {
                    sf-=256;
                }
                regs[i] = (unsigned char)sf; 
            }
            result = MLDLSetOffsetTC(regs);
            ERROR_CHECK(result);
            break;

        default:
            return ML_ERROR_INVALID_PARAMETER;
            break;
    }
    return ML_SUCCESS;
}

/** 
 *  @brief      Sets up the Accelerometer calibration and scale factor.
 *
 *              Please refer to the provided "9-Axis Sensor Fusion Application
 *              Note" document provided.  Section 5, "Sensor Mounting Orientation"
 *              offers a good coverage on the mounting matrices and explains
 *              how to use them.
 *
 *  @pre        MLDmpOpen() or 
 *              MLDmpPedometerStandAloneOpen() and MLDmpStart() 
 *              must <b>NOT</b> have been called.
 *
 *  @param[in]  range   
 *                      The range of the accelerometers in g's. An accelerometer
 *                      that has a range of +2g's to -2g's should pass in 2.
 *  @param[in]  orientation 
 *                      A 9 element matrix that represents how the accelerometers
 *                      are oriented with respect to the device they are mounted in
 *                      and the reference axis system.
 *                      A typical set of values are {1, 0, 0, 0, 1, 0, 0, 0, 1}. This
 *                      example corresponds to a 3 x 3 identity matrix.
 *                      
 *  @return     ML_SUCCESS if successful; a non-zero error code otherwise.
 */
tMLError MLSetAccelCalibration(float range, signed char *orientation)
{
    INVENSENSE_FUNC_START;
    float cal[9];
    float scale = range / 32768.f;
    int kk, ii;
    unsigned long sf;
    tMLError result;
    unsigned char regs[4] = {0, 0, 0 ,0};
    struct mldl_cfg *mldl_cfg = MLDLGetCfg();

    if ( MLGetState() != ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;
   
    /* Apply zero g-offset values */
    if (ACCEL_ID_KXSD9 == mldl_cfg->accel->id) {
        regs[0] = 0x80;
        regs[1] = 0x0;
        regs[2] = 0x80;
        regs[3] = 0x0;
    }

    if ( DmpFeatureSupported(KEY_D_1_152) ) {
        result = MLDLSetMemoryMPU(KEY_D_1_152, 4, &regs[0] );
        ERROR_CHECK(result);
    }

    if (scale==0) {
        mlxData.mlAccelSens = 0;
    }

    for( kk=0; kk<9; ++kk ) {
        cal[kk] = scale * orientation[kk];
        mlxData.mlAccelCal[kk] = (long)(cal[kk] * (float)(1L << 30));
    }

    {
#ifdef M_HW
        unsigned char tmp[3] = {DINA0C,
                                DINAC9,
                                DINA2C};
#else
        unsigned char tmp[3] = {DINA4C,
                                DINACD,
                                DINA6C};
#endif

        struct mldl_cfg *mldl_cfg = MLDLGetCfg();
        unsigned char regs[12] = {0};
        unsigned char tmpPtr = 0;

        regs[3] = DINA26;
        regs[4] = DINA46;
        regs[5] = DINA66;
        for (ii=0; ii<3; ii++) {
            for (kk = 0; kk < 3; kk++) {
                if (orientation[3 * ii + kk]) {
                    break;
                }
            }
            if (kk >= 3) {
                // Invalid orientation
                return ML_ERROR_INVALID_PARAMETER;
            }

            regs[ii] = tmp[kk];
            if (orientation[3*ii + kk] < 0)
                regs[ii+3] |= 0x01;
        }

        if ((ACCEL_ID_MMA8450 != mldl_cfg->accel->id) && 
            (ACCEL_ID_MMA8451 != mldl_cfg->accel->id)) {
            result = MLDLSetMemoryMPU(KEY_FCFG_2, 3, regs );
            ERROR_CHECK(result);
            result = MLDLSetMemoryMPU(KEY_FCFG_7, 3, &regs[3] );
            ERROR_CHECK(result);
        }
    }

    if (mlxData.mlAccelSens != 0) {
        sf = (1073741824L/mlxData.mlAccelSens);
    } else {
        sf = 0;
    }
    regs[0] = (unsigned char)((sf>>8)&0xff);
    regs[1] = (unsigned char)(sf & 0xff);
    result = MLDLSetMemoryMPU(KEY_D_0_108, 2, regs);  ERROR_CHECK(result);
    ERROR_CHECK(result);

    sf = mlxData.mlAccelSens/1024;
    regs[0] = (unsigned char)((sf>>8)&0xff);
    regs[1] = (unsigned char)(sf & 0xff);
    result = MLDLSetMemoryMPU(KEY_D_0_96, 2, regs);  ERROR_CHECK(result);
    ERROR_CHECK(result);
    
    return result;
}

/**
 *  @brief      Sets up the Gyro calibration and scale factor.
 *
 *              Please refer to the provided "9-Axis Sensor Fusion Application
 *              Note" document provided.  Section 5, "Sensor Mounting Orientation"
 *              offers a good coverage on the mounting matrices and explains
 *              how to use them.
 *
 *  @pre        MLDmpOpen() or MLDmpPedometerStandAloneOpen().
 *  @pre        MLDmpStart() must have <b>NOT</b> been called.
 *
 *  @param[in] range 
 *                  The range of the gyros in degrees per second. A gyro
 *                  that has a range of +2000 dps to -2000 dps should pass in 
 *                  2000.
 *  @param[in] orientation 
 *                  A 9 element matrix that represents how the gyro are oriented 
 *                  with respect to the device they are mounted in. A typical set 
 *                  of values are {1, 0, 0, 0, 1, 0, 0, 0, 1}. This example corresponds 
 *                  to a 3 x 3 identity matrix.
 *
 *  @return    ML_SUCCESS if successful or Non-zero error code otherwise.
 */
tMLError MLSetGyroCalibration( float range, signed char *orientation )
{
    INVENSENSE_FUNC_START;

    struct mldl_cfg *mldl_cfg = MLDLGetCfg();
    int kk;
    float scale;
    tMLError result;

    unsigned char regs[12] = {0};
    unsigned char maxVal = 0;
    unsigned char tmpPtr = 0;
    unsigned char tmpSign = 0;
    unsigned char i = 0;
    int sf = 0;    

    if ( MLGetState() != ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;   

    if(mldl_cfg->trim!=0) {
        /*---- adjust the declared range to consider the sensitivity 
               trim of this part when different from the default (first dividend) ----*/
        range *= (32768.f/250) / mldl_cfg->trim;
    }

    scale = range / 32768.f;   // inverse of sensitivity for the given full scale range
    mlxData.mlGyroSens = (long)(range * 32768L);

    for( kk=0; kk<9; ++kk ) {
        mlxData.mlGyroCal[kk] = (long)(scale * orientation[kk] * (1L<<30));
        mlxData.mlGyroOrient[kk] = (long)((double)orientation[kk] * (1L<<30));
    }

    {
#ifdef M_HW
        unsigned char tmpD = DINA4C;
        unsigned char tmpE = DINACD;
        unsigned char tmpF = DINA6C;
#else
        unsigned char tmpD = DINAC9;
        unsigned char tmpE = DINA2C;
        unsigned char tmpF = DINACB;
#endif
        regs[3] = DINA36;
        regs[4] = DINA56;
        regs[5] = DINA76;
        
        for (i=0; i<3; i++) {
            maxVal = 0;
            tmpSign = 0;
            if (mlxData.mlGyroOrient[0+3*i]<0)
                tmpSign = 1;
            if (abs(mlxData.mlGyroOrient[1+3*i])>abs(mlxData.mlGyroOrient[0+3*i])) {
                maxVal = 1;
                if (mlxData.mlGyroOrient[1+3*i]<0)
                    tmpSign = 1;
            }
            if (abs(mlxData.mlGyroOrient[2+3*i])>abs(mlxData.mlGyroOrient[1+3*i])) {
                tmpSign = 0;
                maxVal = 2;
                if (mlxData.mlGyroOrient[2+3*i]<0)
                    tmpSign = 1;
            }
            if (maxVal==0) {
                regs[tmpPtr++] = tmpD;
                if (tmpSign)
                    regs[tmpPtr+2] |= 0x01;
            } else if (maxVal==1) {
                regs[tmpPtr++] = tmpE;
                if (tmpSign)
                    regs[tmpPtr+2] |= 0x01;
            } else {
                regs[tmpPtr++] = tmpF;
                if (tmpSign)
                    regs[tmpPtr+2] |= 0x01;
            }
        }
        result = MLDLSetMemoryMPU(KEY_FCFG_1, 3, regs );
        ERROR_CHECK(result);
        result = MLDLSetMemoryMPU(KEY_FCFG_3, 3, &regs[3] );
        ERROR_CHECK(result);
        
        //sf = (gyroSens) * (0.5 * (pi/180) / 200.0) * 16384
        sf = (long)(((long long)mlxData.mlGyroSens*767603923LL)/1073741824LL);
        regs[0] = (unsigned char)(sf>>24);
        regs[1] = (unsigned char)((sf>>16)&0xff);
        regs[2] = (unsigned char)((sf>>8)&0xff);
        regs[3] = (unsigned char)(sf&0xff);        
        result = MLDLSetMemoryMPU(KEY_D_0_104, 4, regs);
        ERROR_CHECK(result);
        
        if (mlxData.mlGyroSens!=0) {
            sf = (long)((long long)23832619764371LL/mlxData.mlGyroSens);
        } else {
            sf = 0;
        }
        regs[0] = (unsigned char)(sf>>24);
        regs[1] = (unsigned char)((sf>>16)&0xff);
        regs[2] = (unsigned char)((sf>>8)&0xff);
        regs[3] = (unsigned char)(sf&0xff);        

        result = MLDLSetMemoryMPU(KEY_D_0_24, 4, regs);
        ERROR_CHECK(result);
    }
    return ML_SUCCESS;
}

/**
 *  @brief      Sets up the Compass calibration and scale factor.
 *
 *              Please refer to the provided "9-Axis Sensor Fusion Application
 *              Note" document provided.  Section 5, "Sensor Mounting Orientation"
 *              offers a good coverage on the mounting matrices and explains
 *              how to use them.
 *
 *  @pre        MLDmpOpen() or MLDmpPedometerStandAloneOpen().
 *  @pre        MLDmpStart() must have <b>NOT</b> been called.
 *  @see        MLSetGyroCalibration 
 *  @see        MLSetAccelCalibration.
 *
 *  @param[in] range 
 *                  The range of the compass.
 *  @param[in] orientation 
 *                  A 9 element matrix that represents how the compass is oriented 
 *                  with respect to the device they are mounted in. A typical set 
 *                  of values are {1, 0, 0, 0, 1, 0, 0, 0, 1}. This example corresponds 
 *                  to a 3 x 3 identity matrix. The matrix describes how to go from the chip
 *                  mounting to the body of the device.
 *
 *  @return    ML_SUCCESS if successful or Non-zero error code otherwise.
 */
tMLError MLSetMagCalibration( float range, signed char *orientation )
{
    INVENSENSE_FUNC_START;
    float cal[9];
    float scale = range / 32768.f;
    int kk;

    if (CompassGetId() == COMPASS_ID_YAS529) {
        scale /= 32.0f;
    }

    for( kk=0; kk<9; ++kk ) {
        cal[kk] = scale * orientation[kk];
        mlxData.mlMagCal[kk] = (long)(cal[kk]*(float)(1L << 30));
    }
    
    mlxData.mlMagSens = (long)(scale*1073741824L);

    if ( DmpFeatureSupported( KEY_CPASS_MTX_00 ) ) {
        unsigned char reg0[4]={0,0,0,0};
        unsigned char regp[4]={64,0,0,0};
        unsigned char regn[4]={64+128,0,0,0};
        unsigned char *reg;
        int_fast8_t kk;
        unsigned short keyList[9]={KEY_CPASS_MTX_00,KEY_CPASS_MTX_01,KEY_CPASS_MTX_02,
            KEY_CPASS_MTX_10,KEY_CPASS_MTX_11,KEY_CPASS_MTX_12,
            KEY_CPASS_MTX_20,KEY_CPASS_MTX_21,KEY_CPASS_MTX_22 };

        for (kk=0;kk<9;++kk) {

            if ( orientation[kk] == 1 )
                reg=regp;
            else if ( orientation[kk] == -1 )
                reg=regn;
            else
                reg=reg0;
            MLDLSetMemoryMPU( keyList[kk], 4, reg );
        }
    }

    return ML_SUCCESS;
}

/**
 *  @brief  used to set an array of motion sensor data.
 *
 *          Please refer to the provided "9-Axis Sensor Fusion Application
 *          Note" document provided.  Section 5, "Sensor Mounting Orientation"
 *          offers a good coverage on the mounting matrices and explains
 *          how to use them.
 *
 *  @pre    MLDmpOpen() or 
 *          MLDmpPedometerStandAloneOpen() 
 *  @pre    MLDmpStart() must <b>NOT</b> have been called.
 *
 *  @param  dataSet     A constant specifying an array of data.
 *  @param  data        A pointer to an array to be copied from the user.
 *
 *  @return ML_SUCCESS if successful; a non-zero error code otherwise.
 */
tMLError MLSetFloatArray(int dataSet, float *data)
{
    INVENSENSE_FUNC_START;
    long arrayTmp[9] = {0};
    unsigned char i;

    switch (dataSet) {
        case ML_GYRO_TEMP_SLOPE:    // internal
            for (i=0; i<3; i++) {
                arrayTmp[i] = (long)(data[i]*65536.f);
            }
            break;
        case ML_GYRO_BIAS:          // internal
            for (i=0; i<3; i++) {
                arrayTmp[i] = (long)(data[i]*65536.f);
            }
            break;
        case ML_ACCEL_BIAS:         // internal
            for (i=0; i<3; i++) {
                arrayTmp[i] = (long)(data[i]*65536.f);
            }
            break;
        case ML_MAG_BIAS:         // internal            
            for (i=0; i<3; i++) {                
                arrayTmp[i] = (long)(data[i]*65536.f);            
            }                                  
            break;        
        default:
            break;
    }

    return MLSetArray(dataSet, arrayTmp);
}

/** 
* @internal
* @brief Sets the Gyro Dead Zone based upon LPF filter settings and Control setup.
*/
tMLError MLSetDeadZone()
{
    unsigned char reg;
    tMLError result;
    extern tMLCTRLParams mlCtrlParams;

    if (mlCtrlParams.functions & ML_DEAD_ZONE) {
        reg = 0x08;
    } else {
        if (mlParams.biasUpdateFunc & ML_BIAS_FROM_LPF) {
            reg = 0x2;
        } else {
            reg = 0;
        }
    }

    result = MLDLSetMemoryMPU(KEY_D_0_163, 1, &reg);   ERROR_CHECK(result);
    return result;
}

/**
 *  @brief  MLSetBiasUpdateFunc is used to register which algorithms will be
 *          used to automatically reset the gyroscope bias.
 *          The engine ML_BIAS_UPDATE must be enabled for these algorithms to
 *          run.
 *
 *  @pre    MLDmpOpen() or 
 *          MLDmpPedometerStandAloneOpen() and MLDmpStart() 
 *          must <b>NOT</b> have been called.
 *
 *  @param  function    A function or bitwise OR of functions that determine
 *                      how the gyroscope bias will be automatically updated.
 *                      Functions include:
 *                      - ML_NONE or 0,
 *                      - ML_BIAS_FROM_NO_MOTION,
 *                      - ML_BIAS_FROM_GRAVITY,
 *                      - ML_BIAS_FROM_TEMPERATURE,
 *                      - ML_BIAS_FROM_LPF,
 *                      - ML_MAG_BIAS_FROM_MOTION,
 *                      - ML_MAG_BIAS_FROM_GYRO,
 *                      - ML_ALL.
 *
 *  @return ML_SUCCESS if successful or Non-zero error code otherwise.
 */
tMLError MLSetBiasUpdateFunc(unsigned short function)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[4];
    long tmp[3] = {0, 0, 0};
    tMLError result = ML_SUCCESS;
    struct mldl_cfg *mldl_cfg = MLDLGetCfg();

    if ( MLGetState() != ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;      

    /*--- remove magnetic components from bias tracking 
          if there is no compass ---*/
    if ( !CompassGetPresent() ) {

        function &= ~(ML_MAG_BIAS_FROM_GYRO|ML_MAG_BIAS_FROM_MOTION);
    } 
    
    mlParams.biasUpdateFunc = function;

    if (mlParams.biasUpdateFunc & ML_BIAS_FROM_LPF) {
        regs[0] = DINA80 + 2;
        regs[1] = DINA2D;
        regs[2] = DINA55;
        regs[3] = DINA7D;
    } else {
        regs[0] = DINA80 + 7;
        regs[1] = DINA2D;
        regs[2] = DINA35;
        regs[3] = DINA3D;
    }
    result = MLDLSetMemoryMPU(KEY_FCFG_5, 4, regs);   ERROR_CHECK(result);
    result = MLSetDeadZone();                         ERROR_CHECK(result);

    if ((mlParams.biasUpdateFunc & ML_BIAS_FROM_GRAVITY) && !CompassGetPresent()) {
        regs[0] = DINAA0+5;
        regs[1] = DINA90+7;
        regs[2] = DINA80+10;
    } else {
        regs[0] = DINA28;
        regs[1] = DINA4C;
        regs[2] = DINA6C;
    }
    result = MLDLSetMemoryMPU(KEY_CFG_2, 3, regs);   ERROR_CHECK(result);
             

    if ((mldl_cfg->offset_tc[0]!=0) ||
        (mldl_cfg->offset_tc[1]!=0) ||
        (mldl_cfg->offset_tc[2]!=0)) {
            mlxData.mlFactoryTempComp = 1;
    }

    if (mlxData.mlFactoryTempComp == 0) {
        if (mlParams.biasUpdateFunc & ML_BIAS_FROM_TEMPERATURE) {
            result = MLSetArray(ML_GYRO_TEMP_SLOPE, mlxData.mlTempSlope);
            ERROR_CHECK(result);
        } else {
            result = MLSetArray(ML_GYRO_TEMP_SLOPE, tmp);
            ERROR_CHECK(result);
        }
    } else {
        mlParams.biasUpdateFunc &= ~ML_LEARN_BIAS_FROM_TEMPERATURE;
    }

    /*---- hard requirement for using bias tracking BIAS_FROM_GRAVITY, relying on
           compass and accel data, is to have accelerometer data delivered in the 
           FIFO ----*/
    if (((mlParams.biasUpdateFunc & ML_BIAS_FROM_GRAVITY) && CompassGetPresent()) ||
        (mlParams.biasUpdateFunc & ML_MAG_BIAS_FROM_GYRO) ||
        (mlParams.biasUpdateFunc & ML_MAG_BIAS_FROM_MOTION))
    {
        FIFOSendAccel(ML_ALL, ML_32_BIT);
        FIFOSendGyro(ML_ALL, ML_32_BIT);
    }
    return result;
}

/**
 *  @brief  MLSetMotionCallback is used to register a callback function that
 *          will trigger when a change of motion state is detected.
 *
 *  @pre    MLDmpOpen() or 
 *          MLDmpPedometerStandAloneOpen() and MLDmpStart() 
 *          must <b>NOT</b> have been called.
 *
 *  @param  func    A user defined callback function accepting a
 *                  motionState parameter, the new motion state.
 *                  May be one of ML_MOTION or ML_NO_MOTION.
 *  @return ML_SUCCESS if successful or Non-zero error code otherwise.
 */
tMLError MLSetMotionCallback(void (*func)(unsigned short motionState) )
{
    INVENSENSE_FUNC_START;

    if ( MLGetState() != ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;  

    mlParams.motionCallback = func;

    return ML_SUCCESS;
}

/**
 *  @brief  MLGetMotionState is used to determine if the device is in
 *          a 'motion' or 'no motion' state.
 *          MLGetMotionState returns ML_MOTION of the device is moving,
 *          or ML_NO_MOTION if the device is not moving. 
 *
 *  @pre    MLDmpOpen() or 
 *          MLDmpPedometerStandAloneOpen() and MLDmpStart() 
 *          must have been called.
 *
 *  @return ML_SUCCESS if successful or Non-zero error code otherwise.
 */
int MLGetMotionState(void)
{
    INVENSENSE_FUNC_START;
    return mlxData.mlMotionState;
}

/**
 *  @brief  MLSetNoMotionThresh is used to set the threshold for
 *          detecting ML_NO_MOTION
 *
 *  @pre    MLDmpOpen() or 
 *          MLDmpPedometerStandAloneOpen() and MLDmpStart() 
 *          must <b>NOT</b> have been called.
 *
 *  @param  thresh  A threshold scaled in degrees per second.                  
 *                  
 *  @return ML_SUCCESS if successful or Non-zero error code otherwise.
 */
tMLError MLSetNoMotionThresh(float thresh)
{
    tMLError result = ML_SUCCESS;
    long tmp;
    unsigned char regs[4] = {0};
    INVENSENSE_FUNC_START;

    tmp = (long)(thresh*thresh*2.045f);
    if (tmp<0) {
        return ML_ERROR;
    } else if (tmp>8180000L) {
        return ML_ERROR;
    }
    mlxData.mlNoMotionThreshold = tmp;

    regs[0] = (unsigned char)(tmp>>24);
    regs[1] = (unsigned char)((tmp>>16)&0xff);
    regs[2] = (unsigned char)((tmp>>8)&0xff);
    regs[3] = (unsigned char)(tmp&0xff);        
    
    result = MLDLSetMemoryMPU( KEY_D_1_108, 4, regs);
    
    return result;
}

/**
 *  @brief  MLSetNoMotionTime is used to set the time required for
 *          detecting ML_NO_MOTION
 *
 *  @pre    MLDmpOpen() or 
 *          MLDmpPedometerStandAloneOpen() and MLDmpStart() 
 *          must <b>NOT</b> have been called.
 *
 *  @param  time    A time in seconds.                  
 *                  
 *  @return ML_SUCCESS if successful or Non-zero error code otherwise.
 */
tMLError MLSetNoMotionTime(float time)
{
    tMLError result = ML_SUCCESS;
    unsigned char regs[2] = {0};    
    long tmp;
    
    INVENSENSE_FUNC_START;

    tmp = (long)(time*200);    
    if (tmp<0) {
        return ML_ERROR;
    } else if (tmp>65535L) {
        return ML_ERROR;
    }
    mlxData.mlMotionDuration = (unsigned short)tmp;
    
    regs[0] = (unsigned char)((mlxData.mlMotionDuration>>8) & 0xff);
    regs[1] = (unsigned char)( mlxData.mlMotionDuration     & 0xff);
    result = MLDLSetMemoryMPU( KEY_D_1_106, 2, regs);
    
    return result;
}

/**
 *  @brief  MLVersion is used to get the ML version.
 *
 *  @note MLVersion can be called at any time.
 *  @param  version     MLVersion writes the ML version
 *                      string pointer to version.
 *  @return ML_SUCCESS if successful or Non-zero error code otherwise.
 */
tMLError MLVersion(unsigned char **version)
{
    INVENSENSE_FUNC_START;

    *version = (unsigned char *)mlVer; //fixme we are wiping const

    return ML_SUCCESS;
}

/** 
 * @brief Check for the presence of the gyro sensor.
 * @return  a boolean value (true if the gyro is present)
 */
int MLGetGyroPresent(void)
{
    static int checked=0;
    static int present=1;
    if (mlxData.mlGyroSens==0) {
        return 0;
    }
#ifdef ML_USE_DMP_SIM
    /* actually check to see if the MPU is present */
    if(!checked) {
        unsigned char d;
        unsigned short res = MLSLSerialRead(MLSerialGetHandle(), MLDGetMPUSlaveAddr(), 0x00, 1, &d);

        if(res == ML_SUCCESS) {
            present = 1;
        } else {
            present = 0;
        }
        checked = 1;
    }
    return present;
#else
    return 1;
#endif
}

static unsigned short row2scale( const signed char *row )
{
    unsigned short b;

    if ( row[0] > 0 )
        b = 0;
    else if ( row[0] < 0 )
        b = 4;
    else if ( row[1] > 0 )
        b = 1;
    else if ( row[1] < 0 )
        b = 5;
    else if ( row[2] > 0 )
        b = 2;
    else if ( row[2] < 0 )
        b = 6;
    else
        b = 7; // error
    return b;
}

unsigned short MLOrientationMatrixToScalar( const signed char *mtx )
{
    unsigned short scalar;
    /*
      XYZ  010_001_000 Identity Matrix
      XZY  001_010_000 
      YXZ  010_000_001
      YZX  000_010_001
      ZXY  001_000_010 
      ZYX  000_001_010 
     */

    scalar = row2scale(mtx);
    scalar |= row2scale(mtx+3) << 3;
    scalar |= row2scale(mtx+6) << 6;

    return scalar;
}

/* Setups up the Freescale 16-bit accel for Sensor Fusion
* @param[in] mtx Is a transformation matrix that describes how to go from the Chip Orientation
*  to the Board Orientation often times called the Body Orientation in Invensense Documentation.
*/
tMLError FreescaleSensorFusion16bit( const signed char *mtx)
{
    unsigned char rr[50];
    unsigned char r1,r2,r3;
    unsigned short orient;
    unsigned char rm[3];
    unsigned char regs2b[3] = {DINA26, DINA46, DINA66};
    tMLError result;
    signed char trans[9];
    uint_fast8_t kk;
    uint_fast16_t nf;
    
    trans[0]=mtx[0];
    trans[1]=mtx[3];
    trans[2]=mtx[6];
    trans[3]=mtx[1];
    trans[4]=mtx[4];
    trans[5]=mtx[7];
    trans[6]=mtx[2];
    trans[7]=mtx[5];
    trans[8]=mtx[8];

    orient = MLOrientationMatrixToScalar( trans );
    
    // KEY_FCFG_2
    for (kk=0;kk<3;++kk) {
        if ( (orient & 3) == kk ) {
            nf = orient & 4;
        } else if ( (orient & 0x18) == (kk<<3)) {
            nf = orient & 0x20;
        } else {
            nf = orient & 0x100;
        }
        if ( nf )
            regs2b[kk] |= 1;
    }
    
    orient = orient & 0xdb;
    
    r1 = DINA4C;
    r2 = DINACD;
    r3 = DINA6C;
    
    kk=0;
    rr[kk++]=DINAC1;
    rr[kk++]=DINAF8+2;
    rr[kk++]=DINAA0+7;
    rr[kk++]=DINACE;
    rr[kk++]=DINAC0;
    rr[kk++]=DINA90+7;
    rr[kk++]=DINAA0+9;
    if ( (orient == 0x11) || (orient == 0x0a) ) {
        // 
        rr[kk++]=DINA2E;
    } else {
        // 
        rr[kk++]=DINA2D;
        rr[kk++]=DINAD8+2;
        rr[kk++]=DINA20;
        rr[kk++]=DINAD8;
    }
    if ( (orient == 0x42) || (orient == 0x50) ) {
        // 
        rr[kk++]=DINA2E;
    } else {
        // Fixed bit on YL
        rr[kk++]=DINA4D;
        rr[kk++]=DINAD8+2;
        rr[kk++]=DINA40;
        rr[kk++]=DINAD8;
    }
    if ( (orient == 0x81) || (orient == 0x88) ) {
        // 
        rr[kk++]=DINA2E;
    } else {
        // 
        rr[kk++]=DINA6D;
        rr[kk++]=DINAD8+2;
        rr[kk++]=DINA60;
        rr[kk++]=DINAD8;
    }
    // 
    rr[kk++]=DINAC1;
    rr[kk++]=DINAA0+7;
    rr[kk++]=DINA68;
    rr[kk++]=DINAC0;
    // 
    rr[kk++]=DINAA0+5;
    switch (orient) {
    case 0x88:
        rr[kk++]=DINACE;
        rr[kk++]=DINACB;
        rr[kk++]=DINACD;
        rm[0]=r2;
        rm[1]=r3;
        rm[2]=r1;
        break;
    case 0x50:
        rr[kk++]=DINACC;
        rr[kk++]=DINACF;
        rr[kk++]=DINACB;
        rm[0]=r2;
        rm[1]=r1;
        rm[2]=r3;
        break;
    case 0x81:
        rr[kk++]=DINACC;
        rr[kk++]=DINACF;
        rr[kk++]=DINACB;
        rm[0]=r3;
        rm[1]=r2;
        rm[2]=r1;
        break;
    case 0x11:
        rr[kk++]=DINACE;
        rr[kk++]=DINACB;
        rr[kk++]=DINACD;
        rm[0]=r1;
        rm[1]=r2;
        rm[2]=r3;
        break;
    case 0x42:
        rr[kk++]=DINACE;
        rr[kk++]=DINACB;
        rr[kk++]=DINACD;
        rm[0]=r3;
        rm[1]=r1;
        rm[2]=r2;
        break;
    case 0x0a:
        rr[kk++]=DINACC;
        rr[kk++]=DINACF;
        rr[kk++]=DINACB;
        rm[0]=r1;
        rm[1]=r3;
        rm[2]=r2;
        break;
    default:
        return ML_ERROR; // bad orientation?
    }
   
    // Combine 
    rr[kk++]=DINAA0+9;
    rr[kk++]=DINA90+5;
    rr[kk++]=DINA25;
    rr[kk++]=DINA4D;
    rr[kk++]=DINA75;
    // Zero out low 16-bits
    rr[kk++]=DINAF8+3;
    rr[kk++]=DINAA0+9;
    rr[kk++]=DINA90+7;
    rr[kk++]=DINA2E;
    rr[kk++]=DINA2E;
    rr[kk++]=DINA2E;

    result = MLDLSetMemoryMPU(KEY_FCFG_2, 3, rm );      ERROR_CHECK(result);
    result = MLDLSetMemoryMPU(KEY_FCFG_7, 3, regs2b );     ERROR_CHECK(result);
    if ( kk != 35 )
        return ML_ERROR; // bad orientation

    result = MLDLSetMemoryMPU(KEY_FCFG_FSCALE, kk, rr );  ERROR_CHECK(result);

    return result;
    
}

/* Setups up the Freescale 8-bit accel for Sensor Fusion
* @param[in] mtx Is a transformation matrix that describes how to go from the Chip Orientation
*  to the Board Orientation often times called the Body Orientation in Invensense Documentation.
*/
tMLError FreescaleSensorFusion8bit( const signed char *mtx)
{
    unsigned char regs[35];
    unsigned short orient,orientSav;
    unsigned char regs2[3];
    unsigned char regs2b[3] = {DINA26, DINA46, DINA66};
    tMLError result;
    uint_fast8_t kk;
    uint_fast16_t nf;
    signed char trans[9];

    orient = MLOrientationMatrixToScalar( mtx );
    orientSav = orient;

    // KEY_FCFG_2
    for (kk=0;kk<3;++kk) {
        if ( (orient & 3) == kk ) {
            regs2[kk] = DINA4C;
        } else if ( (orient & 0x18) == (kk<<3)) {
            regs2[kk] = DINACD;
        } else {
            regs2[kk]=DINA6C;
        }
    }
    result = MLDLSetMemoryMPU(KEY_FCFG_2, 3, regs2 );      ERROR_CHECK(result);

    trans[0]=mtx[0];
    trans[1]=mtx[3];
    trans[2]=mtx[6];
    trans[3]=mtx[1];
    trans[4]=mtx[4];
    trans[5]=mtx[7];
    trans[6]=mtx[2];
    trans[7]=mtx[5];
    trans[8]=mtx[8];

    orient = MLOrientationMatrixToScalar( trans );
    
    // KEY_FCFG_2
    for (kk=0;kk<3;++kk) {
        if ( (orient & 3) == kk ) {
            nf = orient & 4;
        } else if ( (orient & 0x18) == (kk<<3)) {
            nf = orient & 0x20;
        } else {
            nf = orient & 0x100;
        }
        if ( nf )
            regs2b[kk] |= 1;
    }
    result = MLDLSetMemoryMPU(KEY_FCFG_7, 3, regs2b );     ERROR_CHECK(result);

    orient = orientSav & 0xdb;

    kk = 0;
    regs[kk++] = DINAA0+9;
    regs[kk++] = DINA90+9;

    switch (orient) {
    case 0x88:
        regs[kk++] = DINAF8+1;
        regs[kk++] = DINACB;
        regs[kk++] = DINAF8+2;

        regs[kk++] = DINA54;
        regs[kk++] = DINA50;
        regs[kk++] = DINA50;
        regs[kk++] = DINA50;
        regs[kk++] = DINA50;
        regs[kk++] = DINA50;
        regs[kk++] = DINA50;
        regs[kk++] = DINA50;

        regs[kk++] = DINACD;
        break;
    case 0x50:
        regs[kk++] = DINAF8+1;
        regs[kk++] = DINACB;
        regs[kk++] = DINAF8+2;

        regs[kk++] = DINACF;

        regs[kk++] = DINA7C;
        regs[kk++] = DINA78;
        regs[kk++] = DINA78;
        regs[kk++] = DINA78;
        regs[kk++] = DINA78;
        regs[kk++] = DINA78;
        regs[kk++] = DINA78;
        regs[kk++] = DINA78;
        break;
    case 0x81:
        regs[kk++] = DINAF8+2;
        regs[kk++] = DINA2C;
        regs[kk++] = DINA28;
        regs[kk++] = DINA28;
        regs[kk++] = DINA28;
        regs[kk++] = DINA28;
        regs[kk++] = DINA28;
        regs[kk++] = DINA28;
        regs[kk++] = DINA28;

        regs[kk++] = DINACD;

        regs[kk++] = DINACB;
        break;
    case 0x11:
        regs[kk++] = DINAF8+2;
        regs[kk++] = DINA7C;
        regs[kk++] = DINA28;
        regs[kk++] = DINA28;
        regs[kk++] = DINA28;
        regs[kk++] = DINA28;
        regs[kk++] = DINA28;
        regs[kk++] = DINA28;
        regs[kk++] = DINA28; 
        
        regs[kk++] = DINACF;

        regs[kk++] = DINACD;
        break;
    case 0x42:
        regs[kk++] = DINAF8+1;
        regs[kk++] = DINACB;
        regs[kk++] = DINAF8+2;

        regs[kk++] = DINACF;

        regs[kk++] = DINACB;

        regs[kk++] = DINA78;
        regs[kk++] = DINA78;
        regs[kk++] = DINA78;
        regs[kk++] = DINA78;
        regs[kk++] = DINA78;
        regs[kk++] = DINA78;
        regs[kk++] = DINA78;
        regs[kk++] = DINA78;
        break;
    case 0x0a:
        regs[kk++] = DINAF8+2;
        regs[kk++] = DINACD;

        regs[kk++] = DINA54;
        regs[kk++] = DINA50;
        regs[kk++] = DINA50;
        regs[kk++] = DINA50;
        regs[kk++] = DINA50;
        regs[kk++] = DINA50;
        regs[kk++] = DINA50;
        regs[kk++] = DINA50;
        
        regs[kk++] = DINACF;
        break;
    default:
        return ML_ERROR; // bad orientation?
    }

    regs[kk++] = DINA90+7;
    regs[kk++] = DINAF8+3;
    regs[kk++] = DINAA0+9;
    regs[kk++] = DINA0E;
    regs[kk++] = DINA0E;
    regs[kk++] = DINA0E;

    while ( kk < 35 ) {
        regs[kk++] = DINA90+7;
    }

    result = MLDLSetMemoryMPU(KEY_FCFG_FSCALE, 35, regs );  ERROR_CHECK(result);

    return result;
}

/***************************/
/**@}*/ /* end of defgroup */
/***************************/
