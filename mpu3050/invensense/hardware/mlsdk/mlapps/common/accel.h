/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: accel.h 4108 2010-11-20 01:34:54Z nroyer $
 *
 *******************************************************************************/

#ifndef ACCEL_H
#define ACCEL_H

/**
 *  @addtogroup ACCELDL
 *
 *  @{
 *      @file     accel.h
 *      @brief    Top level descriptions for Accelerometer support
 *
**/

#include "mltypes.h"
#include "mpu.h"

    /* ------------ */
    /* - Defines. - */
    /* ------------ */

/*--- default accel support - selection ---*/
#define ACCEL_ST_LIS331                 0
#define ACCEL_KIONIX_KXTF9              1
#define ACCEL_BOSCH                     0
#define ACCEL_ADI                       0

#define ACCEL_SLAVEADDR_INVALID         0x00

#define ACCEL_SLAVEADDR_LIS331          0x18
#define ACCEL_SLAVEADDR_LSM303          0x18
#define ACCEL_SLAVEADDR_KXSD9           0x18
#define ACCEL_SLAVEADDR_KXTF9           0x0F
#define ACCEL_SLAVEADDR_BMA150          0x38
#define ACCEL_SLAVEADDR_BMA222          0x08
#define ACCEL_SLAVEADDR_ADI346          0x53
#define ACCEL_SLAVEADDR_ADI346_ALT      0x1D // alternative addr
#define ACCEL_SLAVEADDR_MMA8450         0x1C
#define ACCEL_SLAVEADDR_MMA8451         0x1C

#define ACCEL_SLAVEADDR_INVENSENSE      0x68
/*
    Define default accelerometer to use if no selection is made
*/
#if ACCEL_ST_LIS331
#define DEFAULT_ACCEL_SLAVEADDR           ACCEL_SLAVEADDR_LIS331
#define DEFAULT_ACCEL_ID                  ACCEL_ID_LIS331
#endif

#if ACCEL_ST_LSM303
#define DEFAULT_ACCEL_SLAVEADDR           ACCEL_SLAVEADDR_LSM303
#define DEFAULT_ACCEL_ID                  ACCEL_ID_LSM303
#endif

#if ACCEL_KIONIX_KXSD9
#define DEFAULT_ACCEL_SLAVEADDR           ACCEL_SLAVEADDR_KXSD9
#define DEFAULT_ACCEL_ID                  ACCEL_ID_KXSD9
#endif

#if ACCEL_KIONIX_KXTF9
#define DEFAULT_ACCEL_SLAVEADDR           ACCEL_SLAVEADDR_KXTF9
#define DEFAULT_ACCEL_ID                  ACCEL_ID_KXTF9
#endif

#if ACCEL_BOSCH
#define DEFAULT_ACCEL_SLAVEADDR           ACCEL_SLAVEADDR_BMA150
#define DEFAULT_ACCEL_ID                  ACCEL_ID_BMA150
#endif

#if ACCEL_BMA222
#define DEFAULT_ACCEL_SLAVEADDR           ACCEL_SLAVEADDR_BMA222
#define DEFAULT_ACCEL_ID                  ACCEL_ID_BMA222
#endif

#if ACCEL_ADI
#define DEFAULT_ACCEL_SLAVEADDR           ACCEL_SLAVEADDR_ADI346
#define DEFAULT_ACCEL_ID                  ACCEL_ID_ADI346
#endif

#if ACCEL_MMA8450
#define DEFAULT_ACCEL_SLAVEADDR           ACCEL_SLAVEADDR_MMA8450
#define DEFAULT_ACCEL_ID                  ACCEL_ID_MMA8450
#endif

#if ACCEL_MMA8451
#define DEFAULT_ACCEL_SLAVEADDR           ACCEL_SLAVEADDR_MMA8451
#define DEFAULT_ACCEL_ID                  ACCEL_ID_MMA8451
#endif

/*--- if no default accelerometer was selected ---*/
#ifndef DEFAULT_ACCEL_SLAVEADDR
#define DEFAULT_ACCEL_SLAVEADDR           ACCEL_SLAVEADDR_INVALID
#endif

#endif // ACCEL_H

/**
 *  @}
**/
