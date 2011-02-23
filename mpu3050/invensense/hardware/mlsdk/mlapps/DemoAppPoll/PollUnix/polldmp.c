/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 *
 * $Id: polldmp.c 3922 2010-10-21 17:58:43Z nroyer $
 * 
 *******************************************************************************/

#include <stdio.h>
#include <time.h>
#include <sys/select.h>
#include <unistd.h>
#include <string.h>

#include "ml.h"
#include "accel.h"
#include "compass.h"
#include "mlFIFO.h"
#include "int.h"
#include "mldl.h"
#include "mlmath.h"
#include "mlsl.h"

#include "mlsetup.h"
#include "helper.h"

#define _pollDebug(x) //{x}

float quat[4] = {1, 0, 0, 0};
    
unsigned int flag=0x82;

// Keyboard hit function
int kbhit(void)
{
    struct timeval tv;
    fd_set read_fd;
    int result;

    tv.tv_sec=0;
    tv.tv_usec=0;
    FD_ZERO(&read_fd);
    FD_SET(0,&read_fd);

    result = select(1, &read_fd, NULL, NULL, &tv);
    if(result == -1)
        return 0;

    result = FD_ISSET(0,&read_fd);
    if(result)
        return 1;

    return 0;
}


// Motion/no motion callback function
void onMotion(unsigned short motionType)
{

    switch(motionType) {

    case ML_MOTION:
        printf("Motion\n");
        break;

    case ML_NO_MOTION:
        printf("No Motion\n");
        break;

    default:
        break;
    }

}

void processedData(void)
{
    if (flag & 0x80) {
        float checksum = 0.0;
        int i;
        float quat[4] = {1, 0, 0, 0};
        FIFOGetQuaternionFloat(quat);

        for(i=0; i<4; i++)
            checksum += (quat[i] * quat[i]);

        MPL_LOGI("%12.4f %12.4f %12.4f %12.4f    -(%12.4f)\n",
                 quat[0],quat[1],quat[2],quat[3], checksum);
    }

    if ( MLCheckFlag(ML_GOT_GESTURE) ){
        printf("Start of gesture\n");
    }

}

void dumpData(void)
{
    if (flag & 0x20) {
        int ii;
        float data[6];
        long fixedData[6];
        FIFOGetAccel(fixedData);
        FIFOGetGyro(&fixedData[3]);
        for (ii = 0; ii < 6; ii++) {
            data[ii] = fixedData[ii] / 65536.0f;
        }
        MPL_LOGI(("A: %12.4f %12.4f %12.4f "
                  "G: %12.4f %12.4f %12.4f \n"),
                 data[0], data[1], data[2],
                 data[3], data[4], data[5]);
	}
}

// Main function
int main(int argc, char *argv[])
{            
    //unsigned short accelSlaveAddr = ACCEL_SLAVEADDR_INVALID;
    unsigned short accelId = ID_INVALID;
    unsigned short compassId = ID_INVALID;
    unsigned char  reg[32];
    unsigned char *verStr;
    int            result;
    int            key=0;
    int            option;
    int            interror;

    MLVersion(&verStr); 
    printf("%s\n",verStr);

    CALL_CHECK_N_RETURN( MLSerialOpen("/dev/mpu") );
    
    if (ML_SUCCESS == MenuHwChoice(&accelId, &compassId)) {
        CALL_CHECK_N_RETURN( SetupPlatform(PLATFORM_ID_MSB, accelId, compassId) );
    }
    interror = IntOpen("/dev/mpuirq");

    CALL_CHECK_N_RETURN( MLDmpOpen() );

    CALL_CHECK_N_RETURN( MLSetBiasUpdateFunc(ML_ALL) );

    // Register callback function to detect "motion" or "no motion"
    CALL_CHECK_N_RETURN( MLSetMotionCallback(onMotion) );

    // Register processed data callback function
    CALL_CHECK_N_RETURN( MLSetProcessedDataCallback(processedData) );

    // Setup FIFO
    CALL_CHECK_N_RETURN( FIFOSendQuaternion(ML_32_BIT) );
    CALL_CHECK_N_RETURN( FIFOSendGyro(ML_ALL,ML_32_BIT) );
    CALL_CHECK_N_RETURN( FIFOSendAccel(ML_ALL,ML_32_BIT) );
    CALL_CHECK_N_RETURN( MLSetFIFORate(20) );
    CALL_CHECK_N_RETURN( MLSetDataMode(ML_DATA_FIFO) );

    // Check to see if interrupts are available.  If so use them
    if (ML_SUCCESS == interror) {
        CALL_CHECK_N_RETURN( MLSetFifoInterrupt(TRUE));
        CALL_CHECK_N_RETURN( MLSetMotionInterrupt(TRUE));
        CALL_CHECK_N_RETURN( IntSetTimeout(100) );

        MPL_LOGI("Interrupts Configured\n");
        flag |= 0x04;
    } 
    else {
        MPL_LOGI("Interrupts unavailable on this platform\n");
        flag &= ~0x04;
    }

    CALL_CHECK_N_RETURN( MLDmpStart() );

    //Loop  
    while (1) {            

        usleep(8000);

        result = kbhit();
        _pollDebug(printf("kbhit result : %d\n", result););
        if (result) {
            key = getchar();
            _pollDebug(printf("getchar key : %c (%d)\n", key, key);); 
        } else{
            key = 0; 
        } 

        if (key=='q') {
            printf("quit...\n");
            break;
        } else if (key=='0') {
            printf("flag=0\n");
            flag  = 0; 
        } else if (key=='1') {
            if (flag & 1) {
                MPL_LOGI("flag &= ~1 - who am i\n");
                flag &= ~1;
            } else {
                MPL_LOGI("flag |= 1 - who am i\n");
                flag |= 1;
            }
        } else if (key=='2') {
            if (flag & 2) {
                MPL_LOGI("flag &= ~2 - MLUpdateData()\n");
                flag &= ~2;
            } else {
                MPL_LOGI("flag |= 2 - MLUpdateData()\n");
                flag |= 2;
            }
        } else if (key=='4') {
            if (flag & 4) {
                MPL_LOGI("flag &= ~4 - IntProcess()\n");
                flag &= ~4;
            } else {
                MPL_LOGI("flag |= 4 - IntProcess()\n");
                flag |= 4;
            }
        } else if (key=='a') {
            if (flag & 0x80) {
                MPL_LOGI("flag &= ~0x80 - Quaternion\n");
                flag &= ~0x80;
            } else {
                MPL_LOGI("flag |= 0x80  - Quaternion\n");
                flag |= 0x80;
            }
        } else if (key=='b') {
            if (flag & 0x20) {
                printf("flag &= ~0x20 - dumpData()\n");
                flag &= ~0x20;
            } else {
                printf("flag |= 0x20 - dumpData()\n");
                flag |= 0x20;
            }                      
        } else if (key=='h') {
            MPL_LOGI(
                "\n\n"
                "0   -   turn all the features OFF\n"
                "1   -   read WHO_AM_I\n"
                "2   -   call MLUpdateData()\n"
                "4   -   call IntProcess()\n"
                "a   -   print Quaternion data\n"
                "b   -   Print raw accel and gyro data\n"
                "h   -   show this help\n"
                "\n\n"
                );
        } 

        if (flag & 1) {
            _pollDebug(printf("MLSLSerialReadSingle(0x68,0,reg)\n"););
            CALL_CHECK_N_RETURN( MLSLSerialRead(MLSerialGetHandle(), 
                                                0x68,0,1,reg) );
            printf("\nreg[0]=%02x",reg[0]);
        }
        if (flag & 2) {
            result = MLUpdateData();
            if (ML_SUCCESS != result) {
                MPL_LOGE("MLUpdateData returned %d\n", result);
            }
        }
        if (flag & 4) {
            struct mpuirq_data data;
            _pollDebug(printf("IntProcess\n"););
            memset(&data,0,sizeof(data));
            if (IntProcess(&data, 0, 500000) > 0) {
                if (data.interruptcount) {
                    MLDLIntHandler(INTSRC_MPU);
                }
            }
        }

        if (flag & 0x20) {
            dumpData();
        }
    }    

    // Close Motion Library
    CALL_CHECK_N_RETURN( MLDmpClose() );
    CALL_CHECK_N_RETURN( MLSerialClose() );

    IntClose();

    return ML_SUCCESS;
}

