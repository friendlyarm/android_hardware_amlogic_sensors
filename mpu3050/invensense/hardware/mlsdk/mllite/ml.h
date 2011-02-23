/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: ml.h 4260 2010-12-08 19:56:45Z prao $
 *
 ******************************************************************************/

/** 
 *  @defgroup ML
 *  @brief  The Motion Library processes gyroscopes and accelerometers to 
 *          provide a physical model of the movement of the sensors. 
 *          The results of this processing may be used to control objects 
 *          within a user interface environment, detect gestures, track 3D 
 *          movement for gaming applications, and analyze the blur created 
 *          due to hand movement while taking a picture.
 *  
 *  @{
 *      @file ml.h
 *      @brief Header file for the Motion Library.
**/


#ifndef ML_H
#define ML_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"
#include "mldmp.h"
#include "mlsl.h"
#include "mpu.h"

    /* ------------ */
    /* - Defines. - */
    /* ------------ */

    /* - Module defines. - */

    /**************************************************************************/
    /*  Motion Library Vesion                                                 */
    /**************************************************************************/

#define ML_VERSION_MAJOR                 3
#define ML_VERSION_MINOR                 2
#define ML_VERSION_SUB_MINOR             1

#define ML_VERSION_MAJOR_STR            "3"
#define ML_VERSION_MINOR_STR            "2"
#define ML_VERSION_SUB_MINOR_STR        "1"

#define ML_VERSION_ENGINEERING          "EngA"
#define ML_VERSION_PRE_ALPHA            "Pre-Alpha"
#define ML_VERSION_ALPHA                "Alpha"
#define ML_VERSION_BETA                 "Beta"
#define ML_VERSION_PRODUCTION           "Prod"

#ifndef ML_VERSION_TYPE
#define ML_VERSION_TYPE ML_VERSION_ALPHA
#endif

#define ML_VERSION  "InvenSense MPL" " " ML_VERSION_TYPE " "\
    "v" ML_VERSION_MAJOR_STR "." ML_VERSION_MINOR_STR "." ML_VERSION_SUB_MINOR_STR \
    " " __DATE__ " " __TIME__


    /**************************************************************************/
    /*  Motion processing engines                                             */
    /**************************************************************************/
		
#define ML_MOTION_DETECT				(0x0004)
#define ML_BIAS_UPDATE					(0x0008)
#define ML_GESTURE						(0x0020)
#define ML_CONTROL						(0x0040)
#define ML_ORIENTATION                  (0x0100)
#define ML_PEDOMETER                    (0x0200)
#define ML_BASIC                        (ML_MOTION_DETECT | ML_BIAS_UPDATE)

    /*******************************************************************************/
    /*  Data Source                                                                */
    /*******************************************************************************/
		
#define ML_DATA_FIFO        (0x1)
#define ML_DATA_POLL        (0x2)
// Gesture Interrupts use the fifo
#define ML_DATA_GESTURE_INT (0x4 | ML_DATA_FIFO)

    /*******************************************************************************/
    /*  Interrupt Source                                                           */
    /*******************************************************************************/
#define ML_INT_MOTION           (0x01)
#define ML_INT_FIFO             (0x02)
#define ML_INT_GESTURE          (0x04)
#define ML_INT_TAP              (0x08)
#define ML_INT_ORIENTATION      (0x10)
#define ML_INT_SHAKE_PITCH      (0x20)
#define ML_INT_SHAKE_ROLL       (0x40)
#define ML_INT_SHAKE_YAW        (0x80)

    /*******************************************************************************/
    /*  Bias update functions                                                      */
    /*******************************************************************************/

#define ML_BIAS_FROM_NO_MOTION          0x0001
#define ML_BIAS_FROM_GRAVITY            0x0002
#define ML_BIAS_FROM_TEMPERATURE        0x0004
#define ML_BIAS_FROM_LPF                0x0008
#define ML_MAG_BIAS_FROM_MOTION         0x0010
#define ML_MAG_BIAS_FROM_GYRO           0x0020
#define ML_LEARN_BIAS_FROM_TEMPERATURE  0x0040
#define ML_AUTO_RESET_MAG_BIAS          0x0080

    /*******************************************************************************/
    /*  Euler angles                                                               */
    /*******************************************************************************/

#define ML_PITCH                        0x0001
#define ML_ROLL                         0x0002
#define ML_YAW                          0x0004

    /*******************************************************************************/
    /*  Elements                                                                   */
    /*******************************************************************************/

#define ML_ELEMENT_1                    0x0001
#define ML_ELEMENT_2                    0x0002
#define ML_ELEMENT_3                    0x0004
#define ML_ELEMENT_4                    0x0008
#define ML_ELEMENT_5                    0x0010
#define ML_ELEMENT_6                    0x0020
#define ML_ELEMENT_7                    0x0040
#define ML_ELEMENT_8                    0x0080

    /*******************************************************************************/
    /*  Accuracy                                                                   */
    /*******************************************************************************/

//#define ML_16_BIT                       0x4000
//#define ML_32_BIT                       0x8000

    /*******************************************************************************/
    /*  Sensors                                                                    */
    /*******************************************************************************/

#define ML_X_GYRO                       0x0001
#define ML_Y_GYRO                       0x0002
#define ML_Z_GYRO                       0x0004
#define ML_X_ACCEL                      0x0008
#define ML_Y_ACCEL                      0x0010
#define ML_Z_ACCEL                      0x0020
#define ML_FIVE_AXIS                    0x003B
#define ML_SIX_AXIS                     0x003F
#define ML_TEMPERATURE                  0x0040
#define ML_TIME                         0x0080

    /*******************************************************************************/
    /*  Sensor types                                                               */
    /*******************************************************************************/

#define ML_GYROS                        0x0001
#define ML_ACCELS                       0x0002

    /*******************************************************************************/
    /*  Motion arrays                                                              */
    /*******************************************************************************/

#define ML_ROTATION_MATRIX              0x0003
#define ML_QUATERNION                   0x0004
#define ML_EULER_ANGLES                 0x0005
#define ML_LINEAR_ACCELERATION          0x0006
#define ML_LINEAR_ACCELERATION_WORLD    0x0007
#define ML_GRAVITY                      0x0008
#define ML_ANGULAR_VELOCITY             0x0009

#define ML_GYRO_CALIBRATION_MATRIX      0x000B
#define ML_ACCEL_CALIBRATION_MATRIX     0x000C
#define ML_GYRO_BIAS                    0x000D
#define ML_ACCEL_BIAS                   0x000E
#define ML_GYRO_TEMP_SLOPE              0x000F

#define ML_RAW_DATA                     0x0011
#define ML_DMP_TAP                      0x0012
#define ML_DMP_TAP2                     0x0021

#define ML_EULER_ANGLES_X               0x0013
#define ML_EULER_ANGLES_Y               0x0014
#define ML_EULER_ANGLES_Z               0x0015

#define ML_BIAS_UNCERTAINTY             0x0016
#define ML_DMP_PACKET_NUMBER            0x0017
#define ML_FOOTER                       0x0018

#define ML_CONTROL_DATA                 0x0019

#define ML_MAGNETOMETER                 0x001A
#define ML_PEDLBS                       0x001B
#define ML_MAG_RAW_DATA					0x001C 
#define ML_MAG_CALIBRATION_MATRIX		0x001D
#define ML_MAG_BIAS						0x001E
#define ML_HEADING						0x001F

#define ML_MAG_BIAS_ERROR				0x0020



#define SET_QUATERNION                                  0x0001
#define SET_GYROS                                       0x0002
#define SET_LINEAR_ACCELERATION                         0x0004
#define SET_GRAVITY                                     0x0008
#define SET_ACCELS                                      0x0010
#define SET_TAP                                         0x0020
#define SET_PEDLBS                                      0x0040
#define SET_LINEAR_ACCELERATION_WORLD                   0x0080
#define SET_CONTROL                                     0x0100
#define SET_PACKET_NUMBER                               0x4000
#define SET_FOOTER                                      0x8000

    /*******************************************************************************/
    /*  Integral reset options                                                     */
    /*******************************************************************************/

#define ML_NO_RESET                     0x0000
#define ML_RESET                        0x0001

    /*******************************************************************************/
    /*  Motion states                                                              */
    /*******************************************************************************/

#define ML_MOTION                       0x0001
#define ML_NO_MOTION                    0x0002

    /**************************************************************************/
    /* Orientation and Gesture states                                         */
    /**************************************************************************/

#define ML_STATE_IDLE       (0)
#define ML_STATE_RUNNING    (1)

    /**************************************************************************/
    /*  Flags                                                                 */
    /**************************************************************************/

#define ML_RAW_DATA_READY               0x0001
#define ML_PROCESSED_DATA_READY         0x0002

#define ML_GOT_GESTURE                  0x0004

#define ML_MOTION_STATE_CHANGE          0x0006

    /*******************************************************************************/
    /*  General                                                                    */
    /*******************************************************************************/

#define ML_NONE              (0x0000)
#define ML_INVALID_FIFO_RATE (0xFFFF)

    /*******************************************************************************/
    /*  ML Params Structure Default Values                                         */
    /*******************************************************************************/

#define ML_BIAS_UPDATE_FUNC_DEFAULT               ML_BIAS_FROM_NO_MOTION|ML_BIAS_FROM_GRAVITY
#define ML_ORIENTATION_MASK_DEFAULT               0x3f
#define ML_PROCESSED_DATA_CALLBACK_DEFAULT           0
#define ML_ORIENTATION_CALLBACK_DEFAULT              0
#define ML_MOTION_CALLBACK_DEFAULT                   0

    /* ------------ */
    /* - Defines. - */
    /* ------------ */
#define MAX_HIGH_RATE_PROCESSES 5
#define MAX_INTERRUPT_PROCESSES 3
/* Number of quantized samples from tap */
#define ML_MAX_NUM_TAP_SAMPLES (8)

#define BINS 25
#define PTS_PER_BIN 5
#define MIN_TEMP -40
#define MAX_TEMP 85
#define TEMPMGR_ADD_DATA 0
#define TEMPMGR_RECOMPUTE 1
#define TEMPMGR_APPLY    2

#define PRECISION 10000.f
#define RANGE_FLOAT_TO_FIXEDPOINT(range, x) {\
    range.mantissa = (long)x; \
    range.fraction = (long)((float)(x-(long)x)*PRECISION); \
}
#define RANGE_FIXEDPOINT_TO_FLOAT(range, x) {\
    x = (float)(range.mantissa); \
    x += ((float)range.fraction/PRECISION); \
}

    /* --------------- */
    /* - Structures. - */
    /* --------------- */

    typedef struct {
        //Sensor data variables
        unsigned short mlEngineMask;

        //Calibration parameters
        long mlBias[6];
        long mlMagBias[3];                
        long mlMagScale[3]; 
        long mlMagTestBias[3];        
        long mlMagTestScale[3];  

        long mlMagBiasError[3];  

        long mlGotNoMotionBias;
        long mlGotCompassBias;
        long mlCompassState;
        long mlAccState;

        long mlFactoryTempComp;
        long mlGotCoarseHeading;

        long mlGyroBiasTest[3];
        long mlAccelCal[9];
        // Deprecated, used mlGyroOrient
        long mlGyroCal[MPU_NUM_AXES * MPU_NUM_AXES];
        long mlGyroOrient[MPU_NUM_AXES * MPU_NUM_AXES];
        long mlAccelSens;
        long mlMagCal[9];        
        long mlGyroSens;
        long mlTempSlope[MPU_NUM_AXES];
        long mlMagSens;
        long mlTempOffset[MPU_NUM_AXES];

        float mlXGyroCoeff[3];
        float mlYGyroCoeff[3];
        float mlZGyroCoeff[3];
        float mlXGyroTempData[BINS][PTS_PER_BIN];
        float mlYGyroTempData[BINS][PTS_PER_BIN];
        float mlZGyroTempData[BINS][PTS_PER_BIN];
        float mlTempData[BINS][PTS_PER_BIN];
        
        long mlTempPtrs[BINS];
        long mlTempValidData[BINS];

        long mlMagCorrection[4];
        long mlMagCorrectionBackup1[4];
        long mlMagCorrectionBackup2[4];

        double mlMagBiasP[9];
        double mlMagBiasV[3];
        int mlMagPeaks[18];
        int mlAllSensorsNoMotion;

        long mlInitMagBias[3];

        int mlGotInitCompassBias;
        int mlResettingCompass;

        long mlAngVBody[MPU_NUM_AXES];
        long mlMagSensorData[3];
        long mlMagCalibratedData[3];
        long mlMagTestCalibratedData[3];
        unsigned short mlFlags[7];
        unsigned short suspend;

        long mlNoMotionThreshold;
        long mlMotionDuration;

        unsigned short mlMotionState;

        unsigned short mlDataMode;
        unsigned short mlInterruptSources;

        unsigned short mlBiasUpdateTime;
        unsigned short mlBiasCalcTime;

        unsigned char internalMotionState;
        long startTime;
        unsigned char saveData[17];
    } tMLXData;

    typedef tMLError (*tMlxdataFunction)(tMLXData *);

    extern tMLXData mlxData;

    /* --------------------- */
    /* - Params Structure. - */
    /* --------------------- */

    typedef struct {


        unsigned short  biasUpdateFunc;                        // A function or bitwise OR of functions that determine how the gyroscope bias will be automatically updated.
        // Functions include ML_BIAS_FROM_NO_MOTION, ML_BIAS_FROM_GRAVITY, and ML_BIAS_FROM_TEMPERATURE.
        // The engine ML_BIAS_UPDATE must be enabled for these algorithms to run.

        unsigned short  orientationMask;                       // Allows a user to register which orientations will trigger the user defined callback function.
        // The orientations are ML_X_UP, ML_X_DOWN, ML_Y_UP, ML_Y_DOWN, ML_Z_UP, and ML_Z_DOWN.
        // ML_ORIENTATION_ALL is equivalent to ML_X_UP | ML_X_DOWN | ML_Y_UP | ML_Y_DOWN | ML_Z_UP | ML_Z_DOWN.

        void (*processedDataCallback)(void);          // Callback function that triggers when all the processing has been finished by the motion processing engines.

        void (*orientationCallback)(unsigned short orient);    // Callback function that will run when a change of orientation is detected.
        // The new orientation. May be one of ML_X_UP, ML_X_DOWN, ML_Y_UP, ML_Y_DOWN, ML_Z_UP, or ML_Z_DOWN.

        void (*motionCallback)(unsigned short motionState);    // Callback function that will run when a change of motion state is detected.
        // The new motion state. May be one of ML_MOTION, or ML_NO_MOTION.

        unsigned char mlState;

    }   tMLParams,   // new type
        ML_Params_t; // backward-compatibily type

    extern tMLParams mlParams;
    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */

    tMLError MLSerialOpen(char const * port);
    tMLError MLSerialClose(void);
    void *MLSerialGetHandle(void);

    /*API for handling the buffer*/
    tMLError MLUpdateData( void );

    /*API for handling polling*/
    int MLCheckFlag(int flag);

    /*API for enabling and disabling engines*/
    int MLGetEngines(void);

    /*API for setting bias update function*/
    tMLError MLSetBiasUpdateFunc(unsigned short biasFunction);

    /*Functions for handling augmented data*/
    tMLError MLGetArray         (int dataSet, long *data);
    tMLError MLGetFloatArray    (int dataSet, float *data);
    tMLError MLSetArray         (int dataSet, long* data);
    tMLError MLSetFloatArray    (int dataSet, float *data);

    tMLError MLApplyAccelEndian         ( void );
    tMLError MLApplyCalibration         ( void );
    tMLError MLSetGyroCalibration       ( float range, signed char *orientation );
    tMLError MLSetAccelCalibration      ( float range, signed char *orientation );
    tMLError MLSetMagCalibration        ( float range, signed char *orientation );

    /*API for detecting change of state*/
    tMLError MLSetMotionCallback(void (*func)(unsigned short motionState) );
    int MLGetMotionState(void);

    /*API for getting ML version. */
    tMLError MLVersion(unsigned char **version);

    /*API for configuring ML interrupt and data mode*/
    tMLError MLSetDataMode(unsigned short dataMode);
    int MLGetDataMode(void);

    tMLError MLSetMotionInterrupt(unsigned char on);
    tMLError MLSetFifoInterrupt(unsigned char on);

    int MLGetInterrupts(void);
    tMLError MLSetFIFORate(unsigned short fifoRate);
    unsigned short MLGetFIFORate(void);

    // new
    tMLError MLEnableMotionDetect ( void );
    tMLError MLDisableMotionDetect ( void );

    // Math
    void MLQMultf(float *q1, float *q2, float *qProd);
    void MLQInvertf(float *q, float *qInverted);
    void MLQMult(long *q1, long *q2, long *qProd);
    void MLQInvert(long *q, long *qInverted);

    tMLError MLLoadCal(unsigned char *calData);
    tMLError MLStoreCal(unsigned char *calData, int length);
    tMLError MLGetCalLength(unsigned int *length);

    //Simulated DMP
    int MLGetGyroPresent(void);

    tMLError MLSetNoMotionTime(float time);
    tMLError MLSetNoMotionThresh(float thresh);
    tMLError MLResetMotion();

    tMLError MLPollMotionStatus(void);
    tMLError MLUpdateBias(void);
    tMLError MLSetDeadZone();
    void MLBiasStart(void);
    void MLBiasStop(void);

    // Private functions shared accross modules
    void MLXInit(void);
    
    tMLError RegisterProcessDmpInterrupt( tMlxdataFunction func );
    tMLError UnRegisterProcessDmpInterrupt( tMlxdataFunction func );
    void RunProcessDmpInterruptFuncs(void);

#ifdef __cplusplus
}
#endif

#endif // ML_H



  /******************/
 /** @} defgroup  **/
/******************/

