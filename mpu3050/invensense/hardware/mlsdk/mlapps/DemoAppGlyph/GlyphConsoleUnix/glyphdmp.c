/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: glyphdmp.c 4256 2010-12-08 18:49:45Z prao $
 *
 *******************************************************************************/

#include <stdio.h>
#include <time.h>
#include <sys/select.h>
#include <unistd.h>
#include <string.h>

#include "math.h"

#include "ml.h"
#include "accel.h"
#include "compass.h"
#include "mlFIFO.h"
#include "mldl.h"
#include "mlcontrol.h"
#include "mlglyph.h"

#include "int.h"

#include "mlsetup.h"
#include "helper.h"

#define ESC       0x1b
#define SPACE     0x20
#define CR        0x0d
#define LF        0x0a

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

// Set Cursor Position Function - substitute for Win32 API
void SetCursorPos(signed short x, signed short y)
{

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

//Update mouse position
void updateMouse()
{
    int x;
    int y;
    unsigned short length;

    //Get latest data point in the glyph trajectory
    MLGetGlyphLength(&length);
    MLGetGlyph(length, &x, &y);

    //Move mouse to track glyph trajectory
    SetCursorPos((signed short)x+500,
                 -(signed short)y+500 );
}

//Train new glyph trajectories
void TrainGlyphs()
{
    unsigned short gotFile = 0;
    FILE *library;
    tMLError result;
    char fileName[256];    
    int status; 

    printf("\nPress ESC to exit.\n");
    printf("Press SPACE to turn gesture training\n");
    printf("on and off.\n");
    printf("Press any other key input the gesture\n");
    printf("being trained.\n");

    printf("\n Wait for No Motion to be detected \n");

    //Loop until a key is hit
    char key = 0;
    unsigned short GlyphID = 0;
    unsigned short GlyphOn = 0;

    MLDmpStart();

    while (key!=ESC) {
        while (!kbhit()) {
            //Track movement and display with the mouse
            status = MLUpdateData();
            if (ML_SUCCESS != result) {
                printf("MLUpdateData returned %d\n", status);
            }
            if (GlyphOn) {
                updateMouse();
            }
            
        }
        key = getchar(); //_getch();
        if ((key == CR) || (key == LF)) {
            continue;
        }

        if (key==SPACE) {
            if (GlyphOn) {
                //Stop movement tracking
                printf("Gesture off\n");
                GlyphOn=0;

                //Add latest trajectory to glyph library
                MLAddGlyph(GlyphID);
                printf("Training gesture: %c\n", (char)GlyphID);
                MLClearGlyph();
                MLStopGlyph();
            } else {
                //Start movement tracking
                printf("Gesture on\n");
                GlyphOn=1;
                MLStartGlyph();
            }
        } else if (key!=ESC) {
            //Switch to a new pattern for training
            GlyphID = (unsigned short)key;
            printf("Training gesture: %c\n", (char)GlyphID);
        }
    }
    //Store glyph library to file
    do {
        printf("Enter file name:\n");
        scanf("%s", fileName);
        library = fopen(fileName, "wb");
        if (library) {
            unsigned char libraryData[10000];
            unsigned short length;
            MLStoreGlyphs(libraryData, &length);
            fwrite(libraryData, 1, length, library);
            fclose(library);
            gotFile = 1;
        } else {
            printf("File error\n");
        }
    } while (!gotFile);
}


//Recognize glyph trajectories
void RecognizeGlyphs()
{
    unsigned short gotFile = 0;
    FILE *library;
    unsigned short finalGesture;
    tMLError result;
    char fileName[256];

    //Load glyph library from file
    do {
        printf("Enter file name [quit to exit]:\n");
        scanf("%s", fileName);
        if (0 == strcmp(fileName, "quit") )
            return;
        library = fopen(fileName, "r");
        if (library) {
            unsigned char libraryData[10000];
            int c;
            unsigned short fileLength=0;
            unsigned short ptr = 0;
            do {
                c = fgetc(library);
                libraryData[ptr++] = (unsigned char)c;
                fileLength++;
            } while (c != EOF);
            fclose(library);
            MLLoadGlyphs(libraryData);
            gotFile = 1;
        } else {
            printf("File not found\n");
        }
    } while (!gotFile);
    printf("\nPress ESC to exit.\n");
    printf("Press SPACE to turn glyph recognition\n");
    printf("on and off.\n");

    printf("\n Wait for No Motion to be detected \n");

    //Loop until a key is hit
    char key = 0;
    unsigned short GlyphOn = 0;

    MLDmpStart();

    while (key!=ESC) {
        while (!kbhit()) {
            //Track movement and display with the mouse
            result = MLUpdateData();
            if (ML_SUCCESS != result) {
                printf("MLUpdateData returned %d\n",result);
            }
            if (GlyphOn) {
                updateMouse();
            }
        }
        key = getchar(); //_getch();
        if ((key == CR) || (key == LF)) {
            continue;
        }

        if (key==SPACE) {
            if (GlyphOn) {
                //Stop movement tracking
                printf("Recognizing off\n");
                GlyphOn=0;

                //Find best match from the library
                MLBestGlyph(&finalGesture);
                printf("Recognized gesture: %c\n", (char)finalGesture);
                MLClearGlyph();
                MLStopGlyph();
            } else {
                //Start movement tracking
                printf("Recognizing on\n");
                GlyphOn=1;
                MLStartGlyph();
            }
        }
    }
}


int main(int argc, char *argv[])
{
    unsigned short accelId   = ID_INVALID;
    unsigned short compassId = ID_INVALID;
    int index;
    int result;
    int interror;
    unsigned char *verStr;

    MLVersion(&verStr);
    printf("%s\n",verStr);

    CALL_CHECK_N_RETURN( MLSerialOpen("/dev/mpu") );
    interror = IntOpen("/dev/mpuirq");

    result = MenuHwChoice(&accelId, &compassId);
    if (ML_SUCCESS == result) {
        CALL_N_CHECK( SetupPlatform(PLATFORM_ID_MSB, accelId, compassId) );
    }

    CALL_N_CHECK( MLDmpOpen() );

    CALL_N_CHECK( MLSetBiasUpdateFunc(ML_ALL) );

    // Register callback function to detect "motion" or "no motion"
    CALL_N_CHECK( MLSetMotionCallback(onMotion) );

    //Initialize control parameters
    MLSetControlSignals(ML_CONTROL_3 | ML_CONTROL_4);
    MLSetControlData(ML_CONTROL_3, ML_ANGULAR_VELOCITY, ML_PITCH);
    MLSetControlData(ML_CONTROL_4, ML_ANGULAR_VELOCITY, ML_YAW);
    MLSetControlSensitivity(ML_CONTROL_3, 150);
    MLSetControlSensitivity(ML_CONTROL_4, 150);
    MLSetGridThresh(ML_CONTROL_3, 1000);
    MLSetGridThresh(ML_CONTROL_4, 1000);
    MLSetGridMax(ML_CONTROL_3, 3);
    MLSetGridMax(ML_CONTROL_4, 3);
    MLSetControlFunc(  ML_HYSTERESIS | ML_GRID );

    //Enable motion sensing engines
    MLEnableControl();

    //Set up glyph recognition
    MLEnableGlyph();
    MLSetGlyphSpeedThresh(10);
    MLSetGlyphProbThresh(64);
  
    FIFOSendControlData(ML_ALL,ML_32_BIT);    
    FIFOSendQuaternion(ML_32_BIT);        
    FIFOSendGyro(ML_ALL,ML_32_BIT);        
    FIFOSendAccel(ML_ALL,ML_32_BIT);
    FIFOSendGravity(ML_ALL,ML_16_BIT);

    MLSetFIFORate(5);
    MLSetDataMode(ML_DATA_FIFO);

    if (ML_SUCCESS == interror) {
        result = MLSetFifoInterrupt(TRUE);
        result = MLSetMotionInterrupt(TRUE);
        result = IntSetTimeout(100);
        MPL_LOGI("Interrupts Configured\n");
    } 
    else {
        MPL_LOGI("Interrupts unavailable on this platform\n");
    }

    printf("Waiting for calibration...\n");

    //Select training mode or recognizing mode
    printf("1) Recognize glyphs or\n");
    printf("2) Train glyphs?\n");

    int option;
    scanf("%d", &option);

    if (option==1) {
        RecognizeGlyphs();
    } else {        
        TrainGlyphs();
    }

    MLDmpClose();
    MLSerialClose();
    IntClose();

    return 0;
}

