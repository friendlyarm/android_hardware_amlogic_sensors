/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: compass.h 4108 2010-11-20 01:34:54Z nroyer $
 *
 *******************************************************************************/

#ifndef COMPASS_H
#define COMPASS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"
#include "mpu.h"

    /* ------------ */
    /* - Defines. - */
    /* ------------ */

#define USE_COMPASS_AICHI                  0
#define USE_COMPASS_AKM                    0
#define USE_COMPASS_YAS529                 0
#define USE_COMPASS_HMC5883                0
#define USE_COMPASS_MMC314X                0
#define USE_COMPASS_HSCDTD002B             0


#define COMPASS_SLAVEADDR_INVALID          0x00
#define COMPASS_SLAVEADDR_AKM              0x0E
#define COMPASS_SLAVEADDR_AICHI            0x0E
#define COMPASS_SLAVEADDR_YAS529           0x2E
#define COMPASS_SLAVEADDR_HMC5883          0x1E
#define COMPASS_SLAVEADDR_MMC314X          0x30
#define COMPASS_SLAVEADDR_HSCDTD002B       0x0C

/*
    Define default compass to use if no selection is made
*/
 #if USE_COMPASS_AKM
 #define DEFAULT_COMPASS_TYPE              COMPASS_ID_AKM
 #endif

 #if USE_COMPASS_AICHI
 #define DEFAULT_COMPASS_TYPE              COMPASS_ID_AICHI
 #endif

 #if USE_COMPASS_YAS529
 #define DEFAULT_COMPASS_TYPE              COMPASS_ID_YAS529
 #endif

 #if USE_COMPASS_HMC5883
 #define DEFAULT_COMPASS_TYPE              COMPASS_ID_HMC5883
 #endif

#if USE_COMPASS_MMC314X
#define DEFAULT_COMPASS_TYPE              COMPASS_ID_MMC314X
#endif

#if USE_COMPASS_HSCDTD002B
#define DEFAULT_COMPASS_TYPE              COMPASS_ID_HSCDTD002B
#endif

#ifndef DEFAULT_COMPASS_TYPE                
#define DEFAULT_COMPASS_TYPE               ID_INVALID
#endif

    /* --------------- */
    /* - Structures. - */
    /* --------------- */

    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */

    unsigned char  CompassGetPresent    ( void );
    unsigned char  CompassGetSlaveAddr  ( void );
    tMLError       CompassSuspend       ( void );
    tMLError       CompassResume        ( void );
    tMLError       CompassGetData       ( long* data );
    tMLError       CompassSetBias       ( long *bias );
    unsigned short CompassGetId         ( void );

#ifdef __cplusplus
}
#endif

#endif // COMPASS_H
