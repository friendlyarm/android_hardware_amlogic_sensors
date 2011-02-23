/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
 
/************************************************************
 * $Id: intdmp.cpp 2631 2010-06-12 01:11:11Z mcaramello $
 ************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef LINUX
#include <sys/select.h>
#include <sys/time.h>
#include <pthread.h>
#endif

// for InvenSense  
#include "ml.h"
#include "accel.h"
#include "compass.h"
#include "mldl.h"
#include "gesture.h"
#include "gesture_int.h"
#include "orientation.h"
#include "gestureMenu.h"

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPU-intapp"

#include "int.h"
#include "mlos.h"

#include "helper.h"
#include "mlsetup.h"


/*-------------- InvenSense Global Variables -----------------------*/

typedef struct {
    int startTime;
    gesture_t gesture;
} tFinalGesture;

static tFinalGesture finalGesture = {0};
static int sampleCount = 0;
static int fifoRate=4;
tGestureMenuParams gGestureParams;
int fullstream = FALSE;

/*-------------- InvenSense Application Code -----------------------*/

// Set this to 1 to enable polling the interrupt status register instead of
// using the mpuirq driver
#define IRQ_POLLING 0
// Interrupt Poll 
void interruptPoll(long tv_sec, long tv_usec)
{
    // MLDLIntHandler is the Motion Library Interrupt Handler.
    // It should be called when the system kernel/operating system has
    // detected the motion processing interrupt from the interrupt
    // output pin on the MPU.    
    if (IRQ_POLLING) {
        int res;
        unsigned char intStatus;
        // once it has been implemented.
    
        // Poll MPU interrupt status.
        res = MLSLSerialReadSingle(DEFAULT_MPU_SLAVEADDR, 0x1A, &intStatus);
        
        if (ML_SUCCESS == res && intStatus & 0x2) {
            MLDLIntHandler(INTSRC_MPU);
        }   
        //MPL_LOGI("MPU Interrupt Status register %02x\n",intStatus);
    } else {
        struct mpuirq_data data;
        memset(&data,0,sizeof(data));
        if (IntProcess(&data, tv_sec, tv_usec) > 0) {
            if (data.interruptcount) {
                MLDLIntHandler(INTSRC_MPU);
            }
        }
    }
}

// Keyboard hit function
static int kbhit(void)
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

static int ProcessKbhit(void)
{
    int exit = FALSE;

    /* Dynamic keyboard processing */
    char ch = getchar();

    
    if (ML_SUCCESS != GestureMenuProcessChar(&gGestureParams, ch)) {

        switch (ch) {
        case 'b':
        case 'B':
            fullstream = !fullstream;
            break;
        case 'q':
        case 'Q':
            exit = TRUE;
            break;
        default: 
            MPL_LOGE("_kbhit() %d, %c",(int)ch, ch);
            break;
                
        };
    }

    return exit;
}

//Orientation callback function
static void onOrientation(unsigned short orientation)
{	

    // Deteremine if it was a flip
    static int sLastOrientation = 0;
    int        flip = orientation | sLastOrientation;

    if ((ML_X_UP | ML_X_DOWN) == flip){
        MPL_LOGI("Flip about the X Axis: \n");
    } else if ((ML_Y_UP | ML_Y_DOWN) == flip) {
        MPL_LOGI("Flip about the Y axis: \n");
    } else if ((ML_Z_UP | ML_Z_DOWN) == flip) {
        MPL_LOGI("Flip about the Z axis: \n");
    }
    sLastOrientation = orientation;

	switch (orientation) 
	{
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

#define abs(x) ((x<0)?(-x):(x))

/** 
 * Returns the state of all engines state machines as either running
 * or idle.
 * 
 * @pre MLDmpOpen() Must be called with MLDmpDefaultOpen() and then 
 * MLDmpStart().
 *
 * @param[out]  state     Sets this value to:
 *                        - ML_STATE_IDLE
 *                        - ML_STATE_RUNNING if a recent event occured.
 * @return
 *    - ML_SUCCESS correctly called 
 *    - Non-zero error code otherwise
 */
static int MLGetEngineState(int *state) 
{
    int result;

    if (NULL == state) {
        return ML_ERROR_INVALID_PARAMETER;
    }
    
    result = MLGetGestureState(state);
    if ((ML_SUCCESS != result) || (ML_STATE_IDLE != *state)){
        return result;
    }

    result = MLGetOrientationState(state);
    if ((ML_SUCCESS != result) || (ML_STATE_IDLE != *state)){
        return result;
    }

    return ML_SUCCESS;
}

static void CheckGestures(void)
{
    int state;
    int result;

    // Check to see if there is a gesture pending
    if (0 == finalGesture.startTime) return;

    // Check to see if the gesture is done processing
    result = MLGetEngineState(&state);

    if (result != ML_SUCCESS) return;

    // If it is report it
    if (ML_STATE_IDLE == state) {
        PrintGesture(&finalGesture.gesture);
        memset(&finalGesture,0,sizeof(finalGesture));
    }
}

void dumpData(void)
{
    float accel[3], gyro[3];
    //MLGetFloatArray(ML_LINEAR_ACCELERATION, accel);
    MLGetFloatArray(ML_ACCELS, accel);
    MLGetFloatArray(ML_GYROS, gyro);
    MPL_LOGI(" %8d: A:%9.6f %9.6f %9.6f G:%11.6f %11.6f %11.6f\n",
             sampleCount,accel[0],accel[1],accel[2],
             gyro[0], gyro[1], gyro[2]);
}

void onProcessedData(void) 
{
    sampleCount++;
    CheckGestures();
    if (fullstream) {
        dumpData();
    }
}

//Gesture recognition callback function
void onGesture(gesture_t *gesture)
{
    MPL_LOGI( "%s\n" , __func__);
    if ((finalGesture.startTime) && 
        ((gesture->type != finalGesture.gesture.type) ||
         (gesture->num <= finalGesture.gesture.num))) {
        PrintGesture(&finalGesture.gesture);
        memset(&finalGesture,0,sizeof(finalGesture));
    }

    if (gesture->type == ML_YAW_IMAGE_ROTATE) {
        PrintGesture(gesture);
    } else {
        // remember for reporting later after the timer expires
        finalGesture.startTime = sampleCount;
        finalGesture.gesture = *gesture;
    }
}

/*-----------------------------------------------*/
/*		   			variables					 */
/*-----------------------------------------------*/
int main(int argc,char **argv)
{
    unsigned short accelId = ID_INVALID, 
                   compassId = ID_INVALID;
	unsigned char  *verStr;
    int exit = FALSE;
    int result;
    int index;

    MPL_LOGI("[MOTION DEVICE] Service Started..........\n"); 
		
    MLVersion(&verStr);		
	
    MPL_LOGI("MLVersion[%s]....................\n",verStr);

    result = MLSerialOpen("/dev/mpu");
    if (ML_SUCCESS != result) {
        return result;
    }

    if (ML_SUCCESS == MenuHwChoice(&accelId, &compassId)) {
        SetupCalibration(PLATFORM_ID_MSB,accelId,compassId);
    }

    result = IntOpen("/dev/mpuirq");
    if (ML_SUCCESS != result) {
        MPL_LOGE("IntOpen failed %d\n", result);
        goto exit;
    }

    result = IntSetTimeout(100);
    if(ML_SUCCESS != result) {
        MPL_LOGE("IntSetTimeout failed %d\n", result);
        goto exit;
    }

    // Open Motion Library
    result = MLDmpOpen();
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLDmpOpen returned %d\n",result);
        goto exit;
    }

    // Register processed data callback function
    result = MLSetProcessedDataCallback(onProcessedData);
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLSetProcessedDataCallback returned %d\n",result);
        goto exit;
    }

    // Set up orientation
    result = MLEnableOrientation();
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLEnableOrientation returned %d\n",result);
        goto exit;
    }
    result = MLSetOrientations(ML_ORIENTATION_ALL);  
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLSetOrientations returned %d\n",result);
        goto exit;
    }
    result = MLSetOrientationCallback(onOrientation);
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLSetOrientationCallback returned %d\n",result);
        goto exit;
    }
	  
	//Register gestures to be detected
    result = MLEnableGesture();
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLEnableGesture returned %d\n",result);
        goto exit;
    }
    result = MLSetGestureCallback(onGesture);
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLSetGestureCallback returned %d\n",result);
        goto exit;
    }
    result = MLSetGestures(ML_GESTURE_ALL);
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLSetGestures returned %d\n",result);
        goto exit;
    }
    GestureMenuSetDefaults(&gGestureParams);
    result = GestureMenuSetMpl(&gGestureParams);
    if (ML_SUCCESS != result) {
        goto exit;
    }

    // Setup FIFO
    MPL_LOGI("MLSetFIFORate\n");
	result = MLSetFIFORate(fifoRate);
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLSetFIFORate returned %d\n",result);
        goto exit;
    }
	result = MLSetDataMode(ML_DATA_FIFO);
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLSetDataMode returned %d\n",result);
        goto exit;
    }

    // Setup Interrupts
    result = MLSetTapInterrupt(TRUE);
    if (ML_SUCCESS != result) {
        printf("MLSetTapInterrupt returned :%d\n",result);
        goto exit;
    }
    result = MLSetOrientationInterrupt(TRUE);
    if (ML_SUCCESS != result) {
        printf("MLSetOrientationInterrupt returned :%d\n",result);
        goto exit;
    }
    result = MLSetShakePitchInterrupt(TRUE);
    if (ML_SUCCESS != result) {
        printf("MLSetShakePitchInterrupt returned :%d\n",result);
        goto exit;
    }
    result = MLSetShakeRollInterrupt(TRUE);
    if (ML_SUCCESS != result) {
        printf("MLSetShakeRollInterrupt returned :%d\n",result);
        goto exit;
    }
    result = MLSetShakeYawInterrupt(TRUE);
    if (ML_SUCCESS != result) {
        printf("MLSetShakeYawInterrupt returned :%d\n",result);
        goto exit;
    }

    result = MLDmpStart();
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLDmpStart returned %d\n",result);
        goto exit;
    }

    while (!exit){
        // Wait for an interrupt
        if (fullstream || MLDLGetIntTrigger(INTSRC_MPU)) {
            int startTime, finishTime;
            int state;
            if (!fullstream) {
                startTime = MLOSGetTickCount();
            
                MPL_LOGI("**************************************\n");
                MPL_LOGI("****          Interrupt           ****\n");
            }

            // Process any data currently in the fifo
            result = MLGestureIntUpdateData();
            if (ML_SUCCESS != result) {
                MPL_LOGI("MLGestureIntUpdateData returned :%d\n",result);
            }

            // Interrupts are generated before data is placed into the fifo.
            // Typically this is the packet that has the data that generated
            // the interrupt.  Process at least one more packet
            MLCheckFlag(ML_PROCESSED_DATA_READY);
            while (!MLCheckFlag(ML_PROCESSED_DATA_READY)) {
                interruptPoll(0,0);
                result = MLGestureIntUpdateData();
                if (ML_SUCCESS != result) {
                    MPL_LOGI("MLGestureIntUpdateData returned :%d\n",result);
                }
                MLOSSleep(1);
            }

            // By this time if there is a gesture to process we would have found
            // it by now. Continue processing until we are finished with the 
            // Gesture
            result = MLGetEngineState(&state);
            while( ML_STATE_IDLE != state) {
                interruptPoll(0,0);
                result = MLGestureIntUpdateData();
                if (ML_SUCCESS != result) {
                    MPL_LOGI("MLGestureIntUpdateData returned :%d\n",result);
                }
                result = MLGetEngineState(&state);
                MLOSSleep(1);
            }

            // Output any pending gestures now that all state machines are idle
            CheckGestures();
            finishTime = MLOSGetTickCount();
            if (!fullstream) {
                MPL_LOGI("**** Finished Gesture in %7.3fs ****\n", 
                         (float)(finishTime - startTime)/1000.0);
                MPL_LOGI("**************************************\n\n");
            }
        } else {
            MLOSSleep(8);
        }
        interruptPoll(1,0);

        if (kbhit()) {
            exit = ProcessKbhit();
        }

    }

exit:
    // Close Motion Library
    MLDmpClose();
    MLSerialClose();

    IntClose();
    
    return 0;
}
