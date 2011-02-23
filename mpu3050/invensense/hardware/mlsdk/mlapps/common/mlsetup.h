/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mlsetup.h 3951 2010-10-30 00:29:23Z kkeal $
 *
 *******************************************************************************/

#ifndef MLSETUP_H
#define MLSETUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"

#define PLATFORM_ID_MSB               (0x0001) // Multi sensors testing board
#define PLATFORM_ID_ST_6AXIS          (0x0002) // 6 Axis board with ST accelerometer
#define PLATFORM_ID_MANTIS_PROTOTYPE  (0x0003) // Mantis prototype board
#define PLATFORM_ID_MANTIS_MSB        (0x0004) // Mantis on Multi sensor testing board
#define PLATFORM_ID_MANTIS_USB_DONGLE (0x0005) // Mantis and AKM on USB dongle.

    // Main entry APIs
tMLError SetupPlatform ( unsigned short platformId, 
                         unsigned short accelSelection, 
                         unsigned short compassSelection );
tMLError AccelSetup   ( unsigned short accelId,   unsigned char slaveAddr );
tMLError CompassSetup ( unsigned short compassId, unsigned char slaveAddr );

tMLError SetupGyroCalibration_MSB    ( void );
tMLError SetupAccelCalibration_MSB   ( unsigned short accelSelection );
tMLError SetupCompassCalibration_MSB ( unsigned short accelSelection );

tMLError SetupGyroCalibration_ST_6Axis  (void);
tMLError SetupAccelCalibration_ST_6Axis ( unsigned short accelId );

#ifdef __cplusplus
}
#endif

#endif // MLSETUP_H
