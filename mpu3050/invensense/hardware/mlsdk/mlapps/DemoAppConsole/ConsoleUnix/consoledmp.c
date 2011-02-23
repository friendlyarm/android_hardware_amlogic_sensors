/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 *
 * $Id: consoledmp.c 4201 2010-11-29 19:30:49Z prao $
 *
 *****************************************************************************/

#define CONFIG_SENSORS_KXTF9 y

#include <stdio.h>
#include <time.h>
#include <sys/select.h>
#include <unistd.h>
#include <string.h>

#include "ml.h"
#include "mldl.h"
#include "accel.h"
#include "compass.h"
#include "mlFIFO.h"
#include "int.h"
#include "mpu.h"
#include "mldl_cfg.h"
#include "mlsupervisor_9axis.h"

#include "gesture.h"
#include "orientation.h"
#include "mlMathFunc.h"

#include "log.h"
#include "mlmath.h"

#include "helper.h"
#include "mlsetup.h"
#include "mputest.h"

#include "gestureMenu.h"


#define abs(x) ((x)<0?(-x):(x))

static int setup_calibration(void)
{
    struct ext_slave_descr* accel;
    struct mldl_cfg *mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    /* Interrupt configuration */
    mldl_cfg->pdata->int_config = 0x10;
    /* Gyro Calibration */
    memset(mldl_cfg->pdata->orientation,0,
           sizeof(mldl_cfg->pdata->accel.orientation));
    mldl_cfg->pdata->orientation[0] = 1;
    mldl_cfg->pdata->orientation[4] = -1;
    mldl_cfg->pdata->orientation[8] = -1;
    
    /* Kionix Calibration */
    accel = kxtf9_get_slave_descr();
    memcpy(mldl_cfg->accel,accel,sizeof(*accel));
    memset(mldl_cfg->pdata->accel.orientation,0,
           sizeof(mldl_cfg->pdata->accel.orientation));
    mldl_cfg->pdata->accel.orientation[1] =  1;   // X is -Y
    mldl_cfg->pdata->accel.orientation[3] =  1;   // Y is X
    mldl_cfg->pdata->accel.orientation[8] = -1;   // Y is X
    mldl_cfg->pdata->accel.adapt_num = 0;
    mldl_cfg->pdata->accel.bus       = EXT_SLAVE_BUS_SECONDARY;
    mldl_cfg->pdata->accel.address   = 0x0f;
    mldl_cfg->pdata->accel.get_slave_descr = kxtf9_get_slave_descr;
}


static tGestureMenuParams gestureMenuParams;
unsigned int flag=0x2;

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


// Motion/no motion callback function
void onMotion(unsigned short motionType)
{

    switch(motionType) {

        case ML_MOTION:
            MPL_LOGI("Motion\n");
            break;

        case ML_NO_MOTION:
            MPL_LOGI("No Motion\n");
            break;

        default:
            break;
    }

}

//Heading Information for Compass
void headingInfo(void)
{ 
    float hInfo[4];
    float eulerAngle[3] = {0,0,0};
    
    tMLError result;

    if (flag & 0x40) {
        MLGetFloatArray(ML_EULER_ANGLES,eulerAngle);
        MLGetFloatArray(ML_HEADING,hInfo);
        MPL_LOGI("Heading : %+12.3f    Euler Angle : %12.3f    %12.3f    %12.3f \n",hInfo[0],eulerAngle[0],eulerAngle[1],eulerAngle[2]);
    }
}

void dumpData(void)
{
    if (flag & 0x20) {
        int ii;
        float data[6];
        float compData[4];
        long fixedData[6];
        FIFOGetAccel(fixedData);
        FIFOGetGyro(&fixedData[3]);
        for (ii = 0; ii < 6; ii++) {
            data[ii] = fixedData[ii] / 65536.0f;
        }
        MLGetFloatArray(ML_MAGNETOMETER,compData);

        MPL_LOGI(("A: %12.4f %12.4f %12.4f "
                  "G: %12.4f %12.4f %12.4f "
                  "C: %12.4f %12.4f %12.4f \n"),
                 data[0], data[1], data[2],
                 data[3], data[4], data[5],
                 compData[0],compData[1],compData[2]);
	}
}

// Processed data callback function
void processedData(void)
{
    if (flag & 0x80) {
        float quat[4] = {1, 0, 0, 0};
        FIFOGetQuaternionFloat(quat);
        MPL_LOGI("%12.4f %12.4f %12.4f %12.4f\n",
                 quat[0],quat[1],quat[2],quat[3]);
    }
}

//Orientation callback function
void onOrientation(unsigned short orientation)
{
    // Determine if it was a flip
    static int sLastOrientation = 0;
    int flip = orientation | sLastOrientation;

    if ((ML_X_UP | ML_X_DOWN) == flip) {
        MPL_LOGI("Flip about the X Axis: \n");
    } else if ((ML_Y_UP | ML_Y_DOWN) == flip) {
        MPL_LOGI("Flip about the Y axis: \n");
    } else if ((ML_Z_UP | ML_Z_DOWN) == flip) {
        MPL_LOGI("Flip about the Z axis: \n");
    }
    sLastOrientation = orientation;

    switch (orientation) {
        case ML_X_UP:
            MPL_LOGI("X Axis is up\n");
            break;
        case ML_X_DOWN:
            MPL_LOGI("X Axis is down\n");
            break;
        case ML_Y_UP:
            MPL_LOGI("Y Axis is up\n");
            break;
        case ML_Y_DOWN:
            MPL_LOGI("Y Axis is down\n");
            break;
        case ML_Z_UP:
            MPL_LOGI("Z Axis is up\n");
            break;
        case ML_Z_DOWN:
            MPL_LOGI("Z Axis is down\n");
            break;
        default:
            break;
    }
}

int test_register_dump() 
{
    int ii;
    unsigned char data;
    unsigned short res;
    unsigned int error = FALSE;
    static char buf[256];
    static char tmp[16];
    
    // Successful whoami read.  Now read all registers
    MPL_LOGD("Register Dump:\n    ");
    MPL_LOGD("        00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f\n");
    MPL_LOGD("         |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |\n");
    snprintf(buf,sizeof(buf), "    %02d: ", 0);
    for (ii = 0; ii < NUM_OF_MPU_REGISTERS; ii++) {
        res = MLSLSerialRead(MLSerialGetHandle(), MLDLGetMPUSlaveAddr(),
                             ii, 1, &data);
        
        if (ML_SUCCESS != res) { 
            snprintf(tmp,sizeof(buf),"-- ");
            strcat(buf,tmp);
            error = TRUE;
        } else {
            snprintf(tmp,sizeof(buf),"%02x ",data);
            strcat(buf,tmp);
        }
        if ((ii+1) % 16 == 0) {
            MPL_LOGD("%s\n",buf);
            snprintf(buf,sizeof(buf),"    %02d: ", 1+(ii / 16));
        }
    }
    MPL_LOGD("%s\n",buf);

    return error;
}


// Main function
int main(int argc, char *argv[])
{
    unsigned short accelId = ID_INVALID;
    unsigned short compassId = ID_INVALID;
    unsigned char  reg[32];
    unsigned char *verStr;
    int key = 0, oldKey;
    int result;
    int index;
    int interror;

    MLVersion(&verStr);
    MPL_LOGI("%s\n",verStr);

    CALL_CHECK_N_RETURN( MLSerialOpen("/dev/mpu") );

    if( ML_SUCCESS == MenuHwChoice(&accelId, &compassId)) {
        CALL_CHECK_N_RETURN( SetupPlatform(PLATFORM_ID_MSB, accelId, compassId) );
    }
    interror = IntOpen("/dev/mpuirq");

    CALL_CHECK_N_RETURN( MLDmpOpen() );

    CALL_CHECK_N_RETURN( MLSetBiasUpdateFunc(ML_ALL) );
    CALL_CHECK_N_RETURN( MLEnable9axisFusion() );

    // Register callback function to detect "motion" or "no motion"
    CALL_CHECK_N_RETURN( MLSetMotionCallback(onMotion) );

    // Register processed data callback function
//    DataLoggerSelector(0x10);
//    CALL_CHECK_N_RETURN( MLSetProcessedDataCallback(DataLoggerCb) );

    // Set up orientation
    CALL_CHECK_N_RETURN( MLEnableOrientation() );
    CALL_CHECK_N_RETURN( MLSetOrientations(ML_ORIENTATION_ALL) );
    CALL_CHECK_N_RETURN( MLSetOrientationCallback(onOrientation) );

    //Register gestures to be detected
    CALL_CHECK_N_RETURN( MLEnableGesture() );
    CALL_CHECK_N_RETURN( MLSetGestureCallback(PrintGesture) );
    CALL_CHECK_N_RETURN( MLSetGestures(ML_GESTURE_ALL) );

    // Set up default parameters and gesture menu
    GestureMenuSetDefaults(&gestureMenuParams);
    CALL_CHECK_N_RETURN( GestureMenuSetMpl(&gestureMenuParams) );
    
    CALL_CHECK_N_RETURN( FIFOSendQuaternion(ML_32_BIT) );
    CALL_CHECK_N_RETURN( FIFOSendGyro(ML_ALL,ML_32_BIT) );        
    CALL_CHECK_N_RETURN( FIFOSendAccel(ML_ALL,ML_32_BIT) );
  
    CALL_CHECK_N_RETURN( MLSetFIFORate(6) );
    CALL_CHECK_N_RETURN( MLSetDataMode(ML_DATA_FIFO) );
  
    // Check to see if interrupts are available.  If so use them
    if (ML_SUCCESS == interror) {
        CALL_CHECK_N_RETURN(MLSetFifoInterrupt(TRUE));
        CALL_CHECK_N_RETURN(MLSetMotionInterrupt(TRUE));
        CALL_CHECK_N_RETURN( IntSetTimeout(100) );

        MPL_LOGI("Interrupts Configured\n");
        flag |= 0x04;
    } 
    else {
        MPL_LOGI("Interrupts unavailable on this platform\n");
        flag &= ~0x04;
    }
           
    CALL_CHECK_N_RETURN( MLDmpStart() );
    
    PrintGestureMenu(&gestureMenuParams);
    test_register_dump();
    MPL_LOGI("Starting using flag %d\n", flag);
    
    //Loop
    while (1) {
        result = kbhit();
        if (result) {
            oldKey = key;
            key = getchar();
        } else {
            oldKey = 0;
            key = 0;
        }

        if (key=='q') {
            MPL_LOGI("quit...\n");
            break;
        } else if (key=='0') {
            MPL_LOGI("flag = 0\n");
            flag  = 0;
        } else if (key=='1') {
            if (flag & 1) {
                MPL_LOGI("flag &= ~1 - who am i\n");
                flag &= ~1;
            } else {
                MPL_LOGI("flag |=  1 - who am i\n");
                flag |= 1;
            }
        } else if (key=='2') {
            if (flag & 2) {
                MPL_LOGI("flag &= ~2 - MLUpdateData()\n");
                flag &= ~2;
            } else {
                MPL_LOGI("flag |=  2 - MLUpdateData()\n");
                flag |= 2;
            }
        } else if (key=='4') {
            if (flag & 4) {
                MPL_LOGI("flag &= ~4 - IntProcess()\n");
                flag &= ~4;
            } else {
                MPL_LOGI("flag |=  4 - IntProcess()\n");
                flag |= 4;
            }
        } else if (key=='a') {
            if (flag & 0x80) {
                MPL_LOGI("flag &= ~0x80 - Quaternion\n");
                flag &= ~0x80;
                if (ML_SUCCESS != MLSetProcessedDataCallback(NULL))
                    MPL_LOGI("could not set the callbacki\n");
            } else {
                MPL_LOGI("flag |=  0x80  - Quaternion\n");
                flag |= 0x80;
                if (ML_SUCCESS != MLSetProcessedDataCallback(processedData))
                    MPL_LOGI("could not set the callbacki\n");
            }
        } else if (key=='b') {
            if (flag & 0x20) {
                MPL_LOGI("flag &= ~0x20 - dumpData()\n");
                flag &= ~0x20;
            } else {
                MPL_LOGI("flag |=  0x20 - dumpData()\n");
                flag |= 0x20;
            }                      
        } else if (key=='c') {
            if (flag & 0x40) {
                MPL_LOGI("flag &= ~0x40 - Heading & Euler Angle\n");
                flag &= ~0x40;
            } else {
                MPL_LOGI("flag |=  0x40 - Heading & Euler Angle\n");
                flag |= 0x40;
            }
        } else if ( key == 't' ) {
            test_register_dump();
#ifdef MPU_FT_MPL_INTEGRATION
        } else if (key=='f') {
            key = oldKey;
            MPL_LOGI("factory test...\n");
            MPL_LOGI("\tstopping MPL\n");
            CALL_CHECK_N_RETURN( MLDmpStop() );
            MLOSSleep(5);
            MPL_LOGI("\trunning test\n");
            CALL_CHECK_N_RETURN( FactoryCalibrate(MLSerialGetHandle()) );
            MLOSSleep(5);
            MPL_LOGI("\trestarting MPL\n");
            CALL_CHECK_N_RETURN( MLDmpStart() );
            continue;
        } else if (key=='l') {
            key = oldKey;
            MPL_LOGI("load calibration...\n");
            MPL_LOGI("\tstopping MPL\n");
            CALL_CHECK_N_RETURN( MLDmpStop() );
            MLOSSleep(5);
            MPL_LOGI("\tloading calibration\n");
            CALL_CHECK_N_RETURN( LoadCalibration() );
            MLOSSleep(5);
            MPL_LOGI("\trestarting MPL\n");
            CALL_CHECK_N_RETURN( MLDmpStart() );
            continue;
#endif
        } else if (key=='h') {
            printf(
                "\n\n"
                "0   -   turn all the features OFF\n"
                "1   -   read WHO_AM_I\n"
                "2   -   call MLUpdateData()\n"
                "4   -   call IntProcess()\n"
                "a   -   print Quaternion data\n"
                "b   -   Print raw accel and gyro data\n"
                "c   -   Heading & Euler Angle information\n"
                "t   -   dump register information\n"
            );
#ifdef MPU_FT_MPL_INTEGRATION
            printf(
                "f   -   interrupt execution and run functional test\n"
                "l   -   load calibration from file\n"
            );
#endif
            printf(
                "h   -   show this help\n"
                "\n\n"
            );
            PrintGestureMenu(&gestureMenuParams);
        } else {
            (void)GestureMenuProcessChar(&gestureMenuParams, key);
        }

#if 1
        if (flag & 1) {
            CALL_CHECK_N_RETURN( MLSLSerialRead(MLSerialGetHandle(), MLDLGetMPUSlaveAddr(),0,1,reg) );
            MPL_LOGI("\nreg[0]=%02x\n",reg[0]);
            flag &= ~1;
        }
#endif
#if 1
        if (flag & 2) {
            result = MLUpdateData();
            if (ML_SUCCESS != result) {
                MPL_LOGE("MLUpdateData returned %d\n", result);
            }
        }
#endif
#if 1
        if (flag & 4) {
            struct mpuirq_data data;
            memset(&data,0,sizeof(data));
            if (IntProcess(&data, 0, 500000) > 0) {
                if (data.interruptcount) {
                    MLDLIntHandler(INTSRC_MPU);
                }
            }
        }
#endif
#if 1
        if (flag & 0x20) {
            dumpData();
        }
#endif
#if 1
        if (flag & 0x40) {
            headingInfo();
        }
#endif
        //Adding Sleep of 5ms
        usleep(5000);
    }    // end of while(1)

    CALL_CHECK_N_RETURN( MLDisableOrientation() );

    // Close Motion Library
    printf("StoreCalibration\n");
    CALL_CHECK_N_RETURN( StoreCalibration() );

    CALL_CHECK_N_RETURN( MLDmpStop() );
    CALL_CHECK_N_RETURN( MLDmpClose() );
    CALL_CHECK_N_RETURN( MLSerialClose() );

    IntClose();

    return ML_SUCCESS;
}
