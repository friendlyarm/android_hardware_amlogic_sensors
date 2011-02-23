/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: pointer.c 3922 2010-10-21 17:58:43Z nroyer $
 *
 *******************************************************************************/

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sys/select.h>

#include "ml.h"
#include "accel.h"
#include "compass.h"
#include "mlFIFO.h"
#include "mldl.h"
#include "mlcontrol.h"

#include "mlsetup.h"
#include "helper.h"

#define _pointerDebug(x) //{x}

int moveCursor = 0;

// Keyboard hit function
int kbhit(void)
{
    struct timeval tv;
    fd_set read_fd;

    tv.tv_sec=0;
    tv.tv_usec=0;
    FD_ZERO(&read_fd);
    FD_SET(0,&read_fd);

    if(select(1, &read_fd, NULL, NULL, &tv) == -1)
        return 0;

    if(FD_ISSET(0,&read_fd))
        return 1;

    return 0;
}

//Motion/no motion callback function
void onMotion(unsigned short motionType)
{
    switch (motionType) {
        case ML_MOTION:
            printf("Motion\n");
            if (moveCursor==1) {
                moveCursor = 2;
            }
            break;
        case ML_NO_MOTION:
            printf("No Motion\n");
            if (moveCursor!=2) {
                moveCursor = 1;
            }
            break;
        default:
            break;
    }
}

//Update grid position
void updateGridPos()
{
    int controlSignal[4] = {0};
    int gridNum[4] = {0};
    int gridChange[4] = {0};

    MLGetControlData(controlSignal, gridNum, gridChange);

    _pointerDebug(
        printf("controlSignal: %10d %10d %10d %10d\n",
               controlSignal[0], controlSignal[1],
               controlSignal[2], controlSignal[3]
              );
        printf("gridNum      : %10d %10d %10d %10d\n",
               gridNum[0], gridNum[1], gridNum[2], gridNum[3]
              );
        printf("gridChange   : %10d %10d %10d %10d\n",
               gridChange[0], gridChange[1], gridChange[2], gridChange[3]
              );
    );
    printf("grid pos: %d %d\n", gridNum[0], gridNum[1]);
}


int main(int argc, char *argv[])
{
    unsigned short accelId = ID_INVALID, 
                   compassId = ID_INVALID;
    unsigned char *verStr;
    int result;
    int index;
    int controlSensitivity[2] = {150,150};

    MLVersion(&verStr);
    printf("%s\n",verStr);

    result = MLSerialOpen("/dev/mpu");
    if (ML_SUCCESS != result) {
        return result;
    }

    result = MenuHwChoice(&accelId, &compassId);
    if (ML_SUCCESS == result) {
        CALL_N_CHECK( SetupPlatform(PLATFORM_ID_MSB, accelId, compassId) );
    }

    CALL_N_CHECK( MLDmpOpen() );

    MLSetBiasUpdateFunc( ML_ALL );

    // Register callback function to detect "motion" or "no motion"
    MLSetMotionCallback(onMotion);

    //Initialize control parameters
    MLSetControlSignals(ML_CONTROL_1 | ML_CONTROL_2);
    MLSetControlData(ML_CONTROL_1, ML_ANGULAR_VELOCITY, ML_PITCH);
    MLSetControlData(ML_CONTROL_2, ML_ANGULAR_VELOCITY, ML_YAW);
    MLSetControlSensitivity(ML_CONTROL_1, controlSensitivity[0]);
    MLSetControlSensitivity(ML_CONTROL_2, controlSensitivity[1]);
    MLSetGridThresh(ML_CONTROL_1, 1000);
    MLSetGridThresh(ML_CONTROL_2, 1000);
    MLSetGridMax(ML_CONTROL_1, 3);
    MLSetGridMax(ML_CONTROL_2, 3);

    MLSetControlFunc( ML_DEAD_ZONE | ML_SMOOTH | ML_HYSTERESIS | ML_GRID );

    //Enable motion sensing engines
    MLEnableControl();
    
    FIFOSendControlData(ML_ALL,ML_32_BIT);    
    FIFOSendQuaternion(ML_32_BIT);        
    FIFOSendGyro(ML_ALL,ML_32_BIT);        
    FIFOSendAccel(ML_ALL,ML_32_BIT);

    MLSetFIFORate(5);

    //MLSetFifoInterrupt(TRUE);
    //MLSetMotionInterrupt(TRUE);
    MLSetDataMode(ML_DATA_FIFO);

    MLDmpStart();

    moveCursor = 0;

    // consume previous keypresses
    while(kbhit()) {   
        getchar();
    }

    printf("\nPress <ENTER> key to exit.\n");
    //Loop until return key is hit
    while (!kbhit()) {
        result = MLUpdateData();
        if(ML_SUCCESS != result) {
            printf("MLUpdateData returned %d\n", result);
        }

        if (moveCursor==2) {
            updateGridPos();
        }
        usleep(50000);
    }

    MLDmpClose();
    MLSerialClose();

    return ML_SUCCESS;
}


