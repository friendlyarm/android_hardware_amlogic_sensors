/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
#include "mltypes.h"

    // The value of getGyroMagSqrd is scaled such the (1 dps)^2 = 2^this_number
    // this number must be >=0 and even.
#define GYRO_MAG_SQR_SHIFT 6
    // The value of getAccMagSqrd is scaled such that (1g)^2 = 2^this_number
#define ACC_MAG_SQR_SHIFT 16

#define CAL_RUN     0
#define CAL_RESET   1
#define CAL_RESET_X   2
#define CAL_RESET_TIME   3
#define CAL_ADD_DATA    4
//#define P_INIT  10000000000
#define P_INIT  100000


#define SF_NORMAL           0
#define SF_DISTURBANCE      1
#define SF_FAST_SETTLE      2
#define SF_SLOW_SETTLE      3
#define SF_STARTUP_SETTLE   4
#define SF_UNCALIBRATED     5


typedef struct {
    void (*MLAccelCompassFusion)(double magFB);
    void (*MLSensorFusionTempMgr)(unsigned long deltaTime);
    void (*MLSensorFusionMagCalAdvanced)(double *magFB, unsigned long deltaTime);
    void (*MLResetMagCalAdvanced)(void);
} tMLSupervisorCB;

tMLError MLResetMagCalibration(void);
void MLSensorFusionSupervisorInit(void);
tMLError MLAccelCompassSupervisor(void);




