/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mlsetup.c 4175 2010-11-25 01:24:24Z nroyer $
 *
 *******************************************************************************/

/** 
 *  @defgroup MLSETUP
 *  @brief  The Motion Library external slaves setup override suite.
 *          
 *          Use these APIs to override the kernel/default settings in the
 *          corresponding data structures for gyros, accel, and compass.
 *
 *  @{
 *      @file mlsetup.c
 *      @brief The Motion Library external slaves setup override suite.
 */

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

/*
    Defines
*/
// these have to appear before inclusion of mpu.h
#define CONFIG_SENSORS_MPU3050       y     // MPU Gyroscopes

#define CONFIG_SENSORS_KXSD9         y     // Kionix accel
#define CONFIG_SENSORS_KXTF9         y     // Kionix accel
#define CONFIG_SENSORS_LIS331DLH     y     // ST accelerometer
#define CONFIG_SENSORS_LIS331DLH_LPP y     // ST accelerometer for lpp
#define CONFIG_SENSORS_LSM303DLHA    y     // ST accelerometer
#define CONFIG_SENSORS_BMA150        y     // Bosch 150 accelerometer
#define CONFIG_SENSORS_BMA222        y     // Bosch 222 accelerometer
#define CONFIG_SENSORS_ADXL346       y     // ADI accelerometer
#define CONFIG_SENSORS_MMA8450       y     // Freescale MMA8450 accelerometer
#define CONFIG_SENSORS_MMA8451       y     // Freescale MMA8451 accelerometer
#define CONFIG_SENSORS_MPU6000       y     // Mantis Accel

#define CONFIG_SENSORS_AK8975        y     // AKM compass
#define CONFIG_SENSORS_AMI304        y     // AICHI compass
#define CONFIG_SENSORS_HMC5883       y     // Honeywell compass
#define CONFIG_SENSORS_LSM303DLHM    y     // ST compass
#define CONFIG_SENSORS_YAS529        y     // Yamaha compass
#define CONFIG_SENSORS_MMC314X       y     // MEMSIC compass
#define CONFIG_SENSORS_HSCDTD002B    y     // ALPS compass

#include <stdio.h>
#include <string.h>

#include "mlsetup.h"

#include "accel.h"
#include "compass.h"
#include "ml.h"
#include "mldl.h"
#include "mlstates.h"

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-mlsetup"

#include "mpu.h"
#include "mldl_cfg.h"



/*
    Typedefs
*/
typedef void tSetupFuncAccel    (void);
typedef void tSetupFuncGyro     (void);
typedef void tSetupFuncCompass  (void);

/*
    Prototypes
*/
void Rotate180DegAroundZAxis ( signed char matrix[] );

tMLError SetupCalibration ( unsigned short platformId, 
                            unsigned short accelId, 
                            unsigned short compassId );



/*********************************************************************
              Multi sensor board - PLATFORM_ID_MSB
*********************************************************************/

static tSetupFuncGyro SetupMPUCalibration_MSB;
// accel
static tSetupFuncAccel SetupAccelKionixKXSD9Calibration_MSB;
static tSetupFuncAccel SetupAccelKionixKXTF9Calibration_MSB;
static tSetupFuncAccel SetupAccelSTLIS331Calibration_MSB;
static tSetupFuncAccel SetupAccelSTLIS331LPPCalibration_MSB;
static tSetupFuncAccel SetupAccelSTLSM303Calibration_MSB;
static tSetupFuncAccel SetupAccelBMA150Calibration_MSB;
static tSetupFuncAccel SetupAccelBMA222Calibration_MSB;
static tSetupFuncAccel SetupAccelADI346Calibration_MSB;
static tSetupFuncAccel SetupAccelMMA8450Calibration_MSB;
static tSetupFuncAccel SetupAccelMMA8451Calibration_MSB;
// compass
static tSetupFuncCompass   SetupCompassAKMCalibration_MSB;
static tSetupFuncCompass   SetupCompassAICHICalibration_MSB;
static tSetupFuncCompass   SetupCompassHMCCalibration_MSB;
static tSetupFuncCompass   SetupCompassLSM303Calibration_MSB;
static tSetupFuncCompass   SetupCompassYASCalibration_MSB;
static tSetupFuncCompass   SetupCompassMMCCalibration_MSB;
static tSetupFuncCompass   SetupCompassHSCDTD002BCalibration_MSB;

static tMLError AccelSetupSTLIS331 ( unsigned char slaveAddr );
static tMLError AccelSetupSTLSM303 ( unsigned char slaveAddr );
static tMLError AccelSetupKionixKXSD9( void );
static tMLError AccelSetupKionixKXTF9( void );
static tMLError AccelSetupBMA150   ( unsigned char slaveAddr );
static tMLError AccelSetupBMA222   ( unsigned char slaveAddr );
static tMLError AccelSetupADI346   ( unsigned char slaveAddr );
static tMLError AccelSetupMMA8450  ( unsigned char slaveAddr );
static tMLError AccelSetupMMA8451  ( unsigned char slaveAddr );

static tMLError CompassSetupAKM        ( unsigned char slaveAddr );
static tMLError CompassSetupMMC314X    ( unsigned char slaveAddr );
static tMLError CompassSetupAICHI      ( unsigned char slaveAddr );
static tMLError CompassSetupYAS529     ( void );
static tMLError CompassSetupHMC5883    ( void );
static tMLError CompassSetupHSCDTD002B ( unsigned char slaveAddr );

/*********************************************************************
              USB dongle with Mantis/AKM - PLATFORM_ID_MSB
*********************************************************************/
tMLError SetupMantisCalibration(int side);
static tSetupFuncCompass   SetupCompassAKMCalibration_Dongle;


/************************
        User API
*************************/

/**
 *  @brief  Allow the user to overide the hw setup for gyros, accel, and compass.
 *  @param  platfromId
 *              an user defined Id to distinguish the Hw platform in
 *              use from others.
 *  @param  accelId
 *              a combination of:
 *              - accelerometer specific id, as specified in the MPL, at the lower byte
 *              - accelerometer slave address, if different from the default, at the higher byte. Optional.
 *  @param  compassId
 *              a combination of:
 *              - compass specific id, as specified in the MPL, at the lower byte
 *              - compass slave address, if different from the default, at the higher byte. Optional.
 *  @return ML_SUCCESS or a non-zero error code.
**/
tMLError SetupPlatform( 
                unsigned short platformId, 
                unsigned short accelId,
                unsigned short compassId )
{
    tMLError result;

    if ( platformId != PLATFORM_ID_MANTIS_PROTOTYPE && platformId != PLATFORM_ID_MANTIS_MSB) {
        result = AccelSetup( accelId & 0xff, accelId>>8 );
        ERROR_CHECK(result);
    }
    result = CompassSetup( compassId & 0xff, compassId>>8 );
    ERROR_CHECK(result);
    SetupCalibration(platformId, accelId & 0xff, compassId & 0xff);
    ERROR_CHECK(result);

    return ML_SUCCESS;
}


/**
 *  @brief  Setup the Hw orientation and full scale.
 *  @param  platfromId
 *              an user defined Id to distinguish the Hw platform in
 *              use from others.
 *  @param  accelId
 *              the accelerometer specific id, as specified in the MPL.
 *  @param  compassId
 *              the compass specific id, as specified in the MPL.
 *  @return ML_SUCCESS or a non-zero error code.
**/
tMLError SetupCalibration( 
                unsigned short platformId, 
                unsigned short accelId,
                unsigned short compassId )
{
    tMLError result;

    /*----  setup the gyros ----*/
    switch(platformId) {
        case PLATFORM_ID_MSB:
            result = SetupGyroCalibration_MSB();                ERROR_CHECK(result);
            result = SetupAccelCalibration_MSB( accelId );      ERROR_CHECK(result);
            result = SetupCompassCalibration_MSB( compassId );  ERROR_CHECK(result);
#ifdef LINUX
            /* On Linux override the orientation, level shifter etc */
            {
                MPL_LOGI("Setting the MPU_SET_PLATFORM_DATA\n");
                struct mldl_cfg *mldl_cfg = MLDLGetCfg();
                ioctl((int)MLSerialGetHandle(), 
                      MPU_SET_PLATFORM_DATA, 
                      mldl_cfg->pdata);
            }
#endif
            break;
        case PLATFORM_ID_ST_6AXIS:
            result = SetupAccelCalibration_ST_6Axis( accelId ); ERROR_CHECK(result);
            result = SetupGyroCalibration_ST_6Axis();           ERROR_CHECK(result);
            break;
#ifdef M_HW
        case PLATFORM_ID_MANTIS_PROTOTYPE:
            result = SetupMantisCalibration(1);
            ERROR_CHECK( result );
            break;
        case PLATFORM_ID_MANTIS_MSB:
            result = SetupMantisCalibration(0);
            ERROR_CHECK(result);
            result = SetupCompassCalibration_MSB(compassId);
            ERROR_CHECK( result );
            break;
        case PLATFORM_ID_MANTIS_USB_DONGLE:
            result = SetupMantisCalibration(2);
            ERROR_CHECK(result);
            SetupCompassAKMCalibration_Dongle();
            ERROR_CHECK( result );
            break;
#endif
        /*
        case PLATFORM_ID_...:
            add here more setup function specific for your platform
            for gyro, accel, compass.
        */
        default:
            MPL_LOGE("Unrecognized platform ID %d\n", platformId);
            return ML_ERROR_INVALID_PARAMETER;
            break;
    }
    return ML_SUCCESS;
}


/**
 *  @brief  Setup a slave address for the specified accelerometer id
 *          different from the default one.
 *          Some accelerometers devices can be setup to respond to a
 *          slave address different from the default.  This is 
 *          typically achieved by pulling high the address pin on the 
 *          device, although some chip might implement this differently.
 *          If the calling function passes in a unsopported slave address 
 *          the default slave address is assumed and an error code is 
 *          returned.
 *
 *  @param  accelId  
 *              accelerometer slave address.
 *  @param  slaveAddr
 *              alternative accelerometer I2C slave address.
 *
 *  @return ML_SUCCESS or a non-zero error code.
 */
tMLError AccelSetup(unsigned short accelId, unsigned char slaveAddr)
{
    tMLError result;

    switch(accelId) {

        case ACCEL_ID_LIS331:
            result = AccelSetupSTLIS331(slaveAddr);
            break;
        case ACCEL_ID_LIS331_LPP:
            result = AccelSetupSTLIS331(slaveAddr);
            break;
        case ACCEL_ID_LSM303:
            result = AccelSetupSTLSM303(slaveAddr);
            break;
        case ACCEL_ID_KXSD9:
            result = AccelSetupKionixKXSD9();
            break;
        case ACCEL_ID_KXTF9:
            result = AccelSetupKionixKXTF9();
            break;
        case ACCEL_ID_BMA150:
            result = AccelSetupBMA150(slaveAddr);
            break;
        case ACCEL_ID_BMA222:
            result = AccelSetupBMA222(slaveAddr);
            break;
        case ACCEL_ID_ADI346:
            result = AccelSetupADI346(slaveAddr);
            break;
        case ACCEL_ID_MMA8450:            
            result = AccelSetupMMA8450(slaveAddr);
            break;
        case ACCEL_ID_MMA8451:
            result = AccelSetupMMA8451(slaveAddr);
            break;
        case ID_INVALID:
        default:
            result = ML_SUCCESS;
            break;
    }

    return result;
}


/**
 *  @brief  performs a 180' rotation around Z axis to reflect
 *          usage of the multi sensor board (MSB) with the
 *          beagleboard
 *  @note   assumes well formed mounting matrix, with only
 *          one 1 for each row.
**/
void Rotate180DegAroundZAxis(signed char matrix[])
{
    int i;
    for(i=0; i<6; i++) {
        matrix[i] = -matrix[i];
    }
}

/*******************************************************************************
        MULTI SENSOR BOARD - Selection and Setup APIs
*******************************************************************************/

tMLError SetupGyroCalibration_MSB(void)
{
    struct mldl_cfg* mldl_cfg = MLDLGetCfg();

    SetupMPUCalibration_MSB();

#ifndef WIN32
    Rotate180DegAroundZAxis(mldl_cfg->pdata->orientation);
#endif

    return ML_SUCCESS;
}


tMLError SetupAccelCalibration_MSB(unsigned short accelId)
{
    //tMLError result;
    struct mldl_cfg* mldl_cfg = MLDLGetCfg();

    /*----  setup the accels ----*/
    switch(accelId) {
        case ACCEL_ID_LSM303:
            SetupAccelSTLSM303Calibration_MSB ();
            break;
        case ACCEL_ID_LIS331:
            SetupAccelSTLIS331Calibration_MSB ();
            break;
        case ACCEL_ID_LIS331_LPP:
            SetupAccelSTLIS331LPPCalibration_MSB ();
            break;
        case ACCEL_ID_KXSD9:
            SetupAccelKionixKXSD9Calibration_MSB ();
            break;
        case ACCEL_ID_KXTF9:
            SetupAccelKionixKXTF9Calibration_MSB ();
            break;
        case ACCEL_ID_BMA150:
            SetupAccelBMA150Calibration_MSB ();
            break;
        case ACCEL_ID_BMA222:
            SetupAccelBMA222Calibration_MSB ();
            break;
        case ACCEL_ID_ADI346:
            SetupAccelADI346Calibration_MSB ();
            break;
        case ACCEL_ID_MMA8450:
            SetupAccelMMA8450Calibration_MSB ();
            break;
        case ACCEL_ID_MMA8451:
            SetupAccelMMA8451Calibration_MSB ();
            break;
        default:
            break;
    }

#ifndef WIN32
    if (accelId != ID_INVALID)
        Rotate180DegAroundZAxis(mldl_cfg->pdata->accel.orientation);
#endif

    return ML_SUCCESS;
}

tMLError SetupAccelCalibration_ST_6Axis(unsigned short accelId)
{
    struct ext_slave_descr* accel;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    float accelScale = 2.048f;

    if (accelId == ACCEL_ID_LIS331) {
        accel = lis331dlh_get_slave_descr();
    } else if (accelId == ACCEL_ID_LIS331_LPP) {
        accel = lis331dlh_lpp_get_slave_descr();
    } else {
        return ML_ERROR_INVALID_PARAMETER;
    }

    MPL_LOGI("Calibrating '%s'\n", __func__);

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(accel->range, accelScale);

    /* Orientation */
    memset(mldl_cfg->pdata->accel.orientation,0,
            sizeof(mldl_cfg->pdata->accel.orientation));
    mldl_cfg->pdata->accel.orientation[0] = 1;
    mldl_cfg->pdata->accel.orientation[4] = 1;
    mldl_cfg->pdata->accel.orientation[8] = 1;

    mldl_cfg->pdata->accel.adapt_num = 0;
    mldl_cfg->pdata->accel.bus       = EXT_SLAVE_BUS_SECONDARY;
    mldl_cfg->pdata->accel.get_slave_descr = lis331dlh_get_slave_descr;
    mldl_cfg->pdata->accel.address         = ACCEL_SLAVEADDR_LIS331;

    memcpy(mldl_cfg->accel,accel,sizeof(*accel));

    return ML_SUCCESS;
}

tMLError SetupCompassCalibration_MSB(unsigned short compassId)
{
    struct mldl_cfg* mldl_cfg = MLDLGetCfg();

    /*----  setup the compass ----*/
    switch(compassId) {
        case COMPASS_ID_AKM:
            SetupCompassAKMCalibration_MSB ();
            break;
        case COMPASS_ID_AICHI:
            SetupCompassAICHICalibration_MSB ();
            break;
        case COMPASS_ID_LSM303:
            SetupCompassLSM303Calibration_MSB ();
            break;
        case COMPASS_ID_HMC5883:
            SetupCompassHMCCalibration_MSB ();
            break;
        case COMPASS_ID_YAS529:
            SetupCompassYASCalibration_MSB ();
            break;
        case COMPASS_ID_MMC314X:
            SetupCompassMMCCalibration_MSB ();
            break;
        case COMPASS_ID_HSCDTD002B:
            SetupCompassHSCDTD002BCalibration_MSB ();
            break;
        default:
            break;
    }

#ifndef WIN32
    if (compassId != ID_INVALID)
        Rotate180DegAroundZAxis(mldl_cfg->pdata->compass.orientation);
#endif

    return ML_SUCCESS;
}


/***********************************************************************************
        WARNING : 
        all the mounting matrices for the Multi sensor board
        default on Windows.  A 180' rotation around Z has to be 
        applyed to correctly work with the Beagleboard mounting.
***********************************************************************************/


/**************************
    Gyro Setup Functions
***************************/

void SetupMPUCalibration_MSB (void) 
{
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    float gyroScale = 2000.f;

    MPL_LOGI("Calibrating '%s'\n", __func__);

    /* Full Scale */
    if (gyroScale == 250.f)
        mldl_cfg->full_scale = MPU_FS_250DPS;
    else if (gyroScale == 500.f)
        mldl_cfg->full_scale = MPU_FS_500DPS;
    else if (gyroScale == 1000.f)
        mldl_cfg->full_scale = MPU_FS_1000DPS;
    else if (gyroScale == 2000.f)
        mldl_cfg->full_scale = MPU_FS_2000DPS;

    /* Orientation */
    memset(mldl_cfg->pdata->orientation, 0,
           sizeof(mldl_cfg->pdata->accel.orientation));
    mldl_cfg->pdata->orientation[0] =  1;
    mldl_cfg->pdata->orientation[4] = -1;
    mldl_cfg->pdata->orientation[8] = -1;

    /* Interrupt */
    mldl_cfg->pdata->int_config = 0x10;  
}


/****************************
    Accel Setup Functions
*****************************/

static 
void SetupAccelKionixKXSD9Calibration_MSB (void)
{
    struct ext_slave_descr* accel;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    accel = kxsd9_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(accel->range, 2.5006f); // (11bit-fullscale + 1 / accel-sensitivity [g]) = (0x800/0x333) or (2048/819)

    /* Orientation */
    memset(mldl_cfg->pdata->accel.orientation,0,
            sizeof(mldl_cfg->pdata->accel.orientation));
    mldl_cfg->pdata->accel.orientation[1] =  1;   // X is  Y
    mldl_cfg->pdata->accel.orientation[3] =  1;   // Y is  X
    mldl_cfg->pdata->accel.orientation[8] = -1;   // Z is -Z

    mldl_cfg->pdata->accel.adapt_num       = 0;
    mldl_cfg->pdata->accel.bus             = EXT_SLAVE_BUS_SECONDARY;
    mldl_cfg->pdata->accel.get_slave_descr = kxsd9_get_slave_descr;
    mldl_cfg->pdata->accel.address         = ACCEL_SLAVEADDR_KXSD9;

    memcpy(mldl_cfg->accel,accel,sizeof(*accel));
}


static 
void SetupAccelKionixKXTF9Calibration_MSB (void)
{
    struct ext_slave_descr* accel;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    accel = kxtf9_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(accel->range, 2.f);

    /* Orientation */
    memset(mldl_cfg->pdata->accel.orientation,0,
            sizeof(mldl_cfg->pdata->accel.orientation));
    mldl_cfg->pdata->accel.orientation[1] =  1;   // X is  Y
    mldl_cfg->pdata->accel.orientation[3] =  1;   // Y is  X
    mldl_cfg->pdata->accel.orientation[8] = -1;   // Z is -Z

    mldl_cfg->pdata->accel.adapt_num       = 0;
    mldl_cfg->pdata->accel.bus             = EXT_SLAVE_BUS_SECONDARY;
    mldl_cfg->pdata->accel.get_slave_descr = kxtf9_get_slave_descr;
    mldl_cfg->pdata->accel.address         = ACCEL_SLAVEADDR_KXTF9;

    memcpy(mldl_cfg->accel,accel,sizeof(*accel));
}


static 
void SetupAccelSTLIS331Calibration_MSB (void) 
{
    struct ext_slave_descr* accel;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    accel = lis331dlh_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(accel->range, 2.048f);

    /* Orientation */
    memset(mldl_cfg->pdata->accel.orientation,0,
            sizeof(mldl_cfg->pdata->accel.orientation));
    mldl_cfg->pdata->accel.orientation[1] = -1;   // X is -Y
    mldl_cfg->pdata->accel.orientation[3] = -1;   // Y is -X
    mldl_cfg->pdata->accel.orientation[8] = -1;   // Z is -Z

    mldl_cfg->pdata->accel.adapt_num       = 0;
    mldl_cfg->pdata->accel.bus             = EXT_SLAVE_BUS_SECONDARY;
    mldl_cfg->pdata->accel.get_slave_descr = lis331dlh_get_slave_descr;
    mldl_cfg->pdata->accel.address         = ACCEL_SLAVEADDR_LIS331;

    memcpy(mldl_cfg->accel,accel,sizeof(*accel));
}

static 
void SetupAccelSTLIS331LPPCalibration_MSB (void) 
{
    struct ext_slave_descr* accel;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    accel = lis331dlh_lpp_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(accel->range, 2.048f);

    /* Orientation */
    memset(mldl_cfg->pdata->accel.orientation,0,
            sizeof(mldl_cfg->pdata->accel.orientation));
    mldl_cfg->pdata->accel.orientation[1] = -1;   // X is -Y
    mldl_cfg->pdata->accel.orientation[3] = -1;   // Y is -X
    mldl_cfg->pdata->accel.orientation[8] = -1;   // Z is -Z

    mldl_cfg->pdata->accel.adapt_num       = 0;
    mldl_cfg->pdata->accel.bus             = EXT_SLAVE_BUS_SECONDARY;
    mldl_cfg->pdata->accel.get_slave_descr = lis331dlh_get_slave_descr;
    mldl_cfg->pdata->accel.address         = ACCEL_SLAVEADDR_LIS331;

    memcpy(mldl_cfg->accel,accel,sizeof(*accel));
}

static 
void SetupAccelSTLSM303Calibration_MSB (void) 
{
    struct ext_slave_descr* accel;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    accel = lsm303dlha_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(accel->range, 2.048f);

    /* Orientation */
    memset(mldl_cfg->pdata->accel.orientation,0,
            sizeof(mldl_cfg->pdata->accel.orientation));
    mldl_cfg->pdata->accel.orientation[1] = -1;   // X is -Y
    mldl_cfg->pdata->accel.orientation[3] =  1;   // Y is X
    mldl_cfg->pdata->accel.orientation[8] =  1;   // Z is Z

    mldl_cfg->pdata->accel.adapt_num       = 0;
    mldl_cfg->pdata->accel.bus             = EXT_SLAVE_BUS_SECONDARY;
    mldl_cfg->pdata->accel.get_slave_descr = lsm303dlha_get_slave_descr;
    mldl_cfg->pdata->accel.address         = ACCEL_SLAVEADDR_LSM303;

    memcpy(mldl_cfg->accel,accel,sizeof(*accel));
}

static 
void SetupAccelBMA150Calibration_MSB (void) 
{
    struct ext_slave_descr* accel;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    accel = bma150_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(accel->range, 2.f);

    /* Orientation */
    memset(mldl_cfg->pdata->accel.orientation,0,
            sizeof(mldl_cfg->pdata->accel.orientation));
    mldl_cfg->pdata->accel.orientation[0] =  1;   // X is  X
    mldl_cfg->pdata->accel.orientation[4] = -1;   // Y is -Y
    mldl_cfg->pdata->accel.orientation[8] = -1;   // Z is -Z

    mldl_cfg->pdata->accel.adapt_num       = 0;
    mldl_cfg->pdata->accel.bus             = EXT_SLAVE_BUS_SECONDARY;
    mldl_cfg->pdata->accel.get_slave_descr = bma150_get_slave_descr;
    mldl_cfg->pdata->accel.address         = ACCEL_SLAVEADDR_BMA150;
    mldl_cfg->pdata->level_shifter   = 1;

    memcpy(mldl_cfg->accel,accel,sizeof(*accel));
}

static 
void SetupAccelBMA222Calibration_MSB (void) 
{
    struct ext_slave_descr* accel;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    accel = bma222_get_slave_descr();
    
    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(accel->range, 2.f);

    /* Orientation */
    memset(mldl_cfg->pdata->accel.orientation,0,
            sizeof(mldl_cfg->pdata->accel.orientation));
    mldl_cfg->pdata->accel.orientation[0] =  1;   // X is X 
    mldl_cfg->pdata->accel.orientation[4] =  1;   // Y is Y
    mldl_cfg->pdata->accel.orientation[8] =  1;   // Z is Z

    mldl_cfg->pdata->accel.adapt_num       = 0;
    mldl_cfg->pdata->accel.bus             = EXT_SLAVE_BUS_SECONDARY;
    mldl_cfg->pdata->accel.get_slave_descr = bma222_get_slave_descr;
    mldl_cfg->pdata->accel.address         = ACCEL_SLAVEADDR_BMA222;
    mldl_cfg->pdata->level_shifter   = 0;

    memcpy(mldl_cfg->accel,accel,sizeof(*accel));
}

static 
void SetupAccelADI346Calibration_MSB (void) 
{
    struct ext_slave_descr* accel;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    accel = adxl346_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(accel->range, 2.f);

    /* Orientation */
    memset(mldl_cfg->pdata->accel.orientation,0,
            sizeof(mldl_cfg->pdata->accel.orientation));
    mldl_cfg->pdata->accel.orientation[0] =  1;   // X is  X
    mldl_cfg->pdata->accel.orientation[4] = -1;   // Y is -Y
    mldl_cfg->pdata->accel.orientation[8] = -1;   // Z is -Z

    mldl_cfg->pdata->accel.adapt_num       = 0;
    mldl_cfg->pdata->accel.bus             = EXT_SLAVE_BUS_SECONDARY;
    mldl_cfg->pdata->accel.get_slave_descr = adxl346_get_slave_descr;
    mldl_cfg->pdata->accel.address         = ACCEL_SLAVEADDR_ADI346;

    memcpy(mldl_cfg->accel,accel,sizeof(*accel));
}


static 
void SetupAccelMMA8450Calibration_MSB (void) 
{
    struct ext_slave_descr* accel;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    accel = mma8450_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(accel->range, 2.f);

    /* Orientation */
    memset(mldl_cfg->pdata->accel.orientation,0,
            sizeof(mldl_cfg->pdata->accel.orientation));
    mldl_cfg->pdata->accel.orientation[1] = -1;   // X is -Y
    mldl_cfg->pdata->accel.orientation[3] = -1;   // Y is -X
    mldl_cfg->pdata->accel.orientation[8] = -1;   // Z is -Z

    mldl_cfg->pdata->accel.adapt_num       = 0;
    mldl_cfg->pdata->accel.bus             = EXT_SLAVE_BUS_SECONDARY;
    mldl_cfg->pdata->accel.get_slave_descr = mma8450_get_slave_descr;
    mldl_cfg->pdata->accel.address         = ACCEL_SLAVEADDR_MMA8450;

    memcpy(mldl_cfg->accel,accel,sizeof(*accel));
}


static 
void SetupAccelMMA8451Calibration_MSB (void) 
{
    struct ext_slave_descr* accel;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    accel = mma8451_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(accel->range, 2.f);

    /* Orientation */
    memset(mldl_cfg->pdata->accel.orientation,0,
            sizeof(mldl_cfg->pdata->accel.orientation));
    mldl_cfg->pdata->accel.orientation[1] = -1;   // X is -Y
    mldl_cfg->pdata->accel.orientation[3] = -1;   // Y is -X
    mldl_cfg->pdata->accel.orientation[8] = -1;   // Z is -Z

    mldl_cfg->pdata->accel.adapt_num       = 0;
    mldl_cfg->pdata->accel.bus             = EXT_SLAVE_BUS_SECONDARY;
    mldl_cfg->pdata->accel.get_slave_descr = mma8451_get_slave_descr;
    mldl_cfg->pdata->accel.address         = ACCEL_SLAVEADDR_MMA8451;

    memcpy(mldl_cfg->accel,accel,sizeof(*accel));
}


/*****************************
    Compass Setup Functions
******************************/
static 
void SetupCompassAKMCalibration_MSB (void) 
{
    struct ext_slave_descr* compass;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    compass = ak8975_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(compass->range, 9830.4f);

    /* Orientation */
    memset(mldl_cfg->pdata->compass.orientation,0,
            sizeof(mldl_cfg->pdata->compass.orientation));

#if 1 /* AKM8975 */  
    mldl_cfg->pdata->compass.orientation[0] = -1;   // X is -X
    mldl_cfg->pdata->compass.orientation[4] = -1;   // Y is -Y
    mldl_cfg->pdata->compass.orientation[8] =  1;   // Z is  Z
#else /* AKM8975C */  
    mldl_cfg->pdata->compass.orientation[0] =  1;   // X is -X
    mldl_cfg->pdata->compass.orientation[4] = -1;   // Y is -Y
    mldl_cfg->pdata->compass.orientation[8] = -1;   // Z is  Z
#endif

    mldl_cfg->pdata->compass.adapt_num       = 0;
#ifdef M_HW
    mldl_cfg->pdata->compass.bus             = EXT_SLAVE_BUS_SECONDARY; 
#else
    mldl_cfg->pdata->compass.bus             = EXT_SLAVE_BUS_PRIMARY;
#endif
    mldl_cfg->pdata->compass.get_slave_descr = ak8975_get_slave_descr;
    mldl_cfg->pdata->compass.address         = COMPASS_SLAVEADDR_AKM;

    memcpy(mldl_cfg->compass,compass,sizeof(*compass));
}

static 
void SetupCompassMMCCalibration_MSB (void) 
{
    struct ext_slave_descr* compass;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    compass = mmc314x_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(compass->range, 400.0f);

    /* Orientation */
    memset(mldl_cfg->pdata->compass.orientation,0,
            sizeof(mldl_cfg->pdata->compass.orientation));
    mldl_cfg->pdata->compass.orientation[1] = 1;   // X is Y
    mldl_cfg->pdata->compass.orientation[3] = 1;   // Y is X
    mldl_cfg->pdata->compass.orientation[8] = -1;   // Z is  Z

    mldl_cfg->pdata->compass.adapt_num       = 0;
    mldl_cfg->pdata->compass.bus             = EXT_SLAVE_BUS_PRIMARY;
    mldl_cfg->pdata->compass.get_slave_descr = mmc314x_get_slave_descr;
    mldl_cfg->pdata->compass.address         = COMPASS_SLAVEADDR_MMC314X;

    memcpy(mldl_cfg->compass,compass,sizeof(*compass));
}

static 
void SetupCompassAICHICalibration_MSB (void) 
{
    struct ext_slave_descr* compass;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    // @todo fixme michele
    compass = ami304_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(compass->range, 5461.33333f);

    /* Orientation */
    memset(mldl_cfg->pdata->compass.orientation,0,
            sizeof(mldl_cfg->pdata->compass.orientation));
    mldl_cfg->pdata->compass.orientation[0] = -1;   // X is -X
    mldl_cfg->pdata->compass.orientation[4] =  1;   // Y is  Y
    mldl_cfg->pdata->compass.orientation[8] = -1;   // Z is -Z

    mldl_cfg->pdata->compass.adapt_num       = 0;
    mldl_cfg->pdata->compass.bus             = EXT_SLAVE_BUS_PRIMARY;
    mldl_cfg->pdata->compass.get_slave_descr = ami304_get_slave_descr;
    mldl_cfg->pdata->compass.address         = COMPASS_SLAVEADDR_AICHI;

    memcpy(mldl_cfg->compass,compass,sizeof(*compass));
}

static 
void SetupCompassHMCCalibration_MSB (void) 
{
    struct ext_slave_descr* compass;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    // @todo fixme michele
    compass = hmc5883_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(compass->range, 10673.6156351f);

    /* Orientation */
    memset(mldl_cfg->pdata->compass.orientation,0,
            sizeof(mldl_cfg->pdata->compass.orientation));
    mldl_cfg->pdata->compass.orientation[0] =  1;   // X is -Y
    mldl_cfg->pdata->compass.orientation[4] = -1;   // Y is -X
    mldl_cfg->pdata->compass.orientation[8] = -1;   // Z is -Z

    mldl_cfg->pdata->compass.adapt_num       = 0;
    mldl_cfg->pdata->compass.bus             = EXT_SLAVE_BUS_PRIMARY;
    mldl_cfg->pdata->compass.get_slave_descr = hmc5883_get_slave_descr;
    mldl_cfg->pdata->compass.address         = COMPASS_SLAVEADDR_HMC5883;

    memcpy(mldl_cfg->compass,compass,sizeof(*compass));
}


static 
void SetupCompassLSM303Calibration_MSB (void) 
{
    struct ext_slave_descr* compass;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    compass = lsm303dlhm_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(compass->range, 10240.f);

    /* Orientation */
    memset(mldl_cfg->pdata->compass.orientation,0,
            sizeof(mldl_cfg->pdata->compass.orientation));
    mldl_cfg->pdata->compass.orientation[1] = -1;   // X is -Y
    mldl_cfg->pdata->compass.orientation[3] =  1;   // Y is X
    mldl_cfg->pdata->compass.orientation[8] =  1;   // Z is Z

    mldl_cfg->pdata->compass.adapt_num       = 0;
    mldl_cfg->pdata->compass.bus             = EXT_SLAVE_BUS_PRIMARY;
    mldl_cfg->pdata->compass.get_slave_descr = lsm303dlhm_get_slave_descr;
    mldl_cfg->pdata->compass.address         = COMPASS_SLAVEADDR_HMC5883;

    memcpy(mldl_cfg->compass,compass,sizeof(*compass));
}


static 
void SetupCompassYASCalibration_MSB (void) 
{
    struct ext_slave_descr* compass;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    // @todo fixme michele
    compass = yas529_get_slave_descr();

    /* Interrupt configuration */
    mldl_cfg->pdata->int_config = 0x10;

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(compass->range, 19660.8f);

    /* Orientation */
    memset(mldl_cfg->pdata->compass.orientation,0,
            sizeof(mldl_cfg->pdata->compass.orientation));
    mldl_cfg->pdata->compass.orientation[0] =  1;   // X is  X
    mldl_cfg->pdata->compass.orientation[4] = -1;   // Y is -Y
    mldl_cfg->pdata->compass.orientation[8] = -1;   // Z is -Z

    mldl_cfg->pdata->compass.adapt_num       = 0;
    mldl_cfg->pdata->compass.bus             = EXT_SLAVE_BUS_PRIMARY;
    mldl_cfg->pdata->compass.get_slave_descr = yas529_get_slave_descr;
    mldl_cfg->pdata->compass.address         = COMPASS_SLAVEADDR_YAS529;

    memcpy(mldl_cfg->compass,compass,sizeof(*compass));
}


static 
void SetupCompassHSCDTD002BCalibration_MSB (void) 
{
    struct ext_slave_descr* compass;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    compass = hscdtd002b_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(compass->range, 9830.4f);

    /* Orientation */
    memset(mldl_cfg->pdata->compass.orientation,0,
            sizeof(mldl_cfg->pdata->compass.orientation));
    mldl_cfg->pdata->compass.orientation[0] = -1;   // X is -X
    mldl_cfg->pdata->compass.orientation[4] = -1;   // Y is -Y
    mldl_cfg->pdata->compass.orientation[8] =  1;   // Z is Z

    mldl_cfg->pdata->compass.adapt_num       = 0;
    mldl_cfg->pdata->compass.bus             = EXT_SLAVE_BUS_PRIMARY;
    mldl_cfg->pdata->compass.get_slave_descr = hscdtd002b_get_slave_descr;
    mldl_cfg->pdata->compass.address         = COMPASS_SLAVEADDR_HSCDTD002B;

    memcpy(mldl_cfg->compass,compass,sizeof(*compass));
}


/*******************************************************************************
        6 axis EVB - Selection and Setup APIs
*******************************************************************************/

tMLError SetupGyroCalibration_ST_6Axis(void)
{
    struct mldl_cfg* mldl_cfg = MLDLGetCfg();
    float gyroScale = 2000.f;

    MPL_LOGI("Calibrating '%s'\n", __func__);

    /* Full Scale */
    if (gyroScale == 250.f)
        mldl_cfg->full_scale = MPU_FS_250DPS;
    else if (gyroScale == 500.f)
        mldl_cfg->full_scale = MPU_FS_500DPS;
    else if (gyroScale == 1000.f)
        mldl_cfg->full_scale = MPU_FS_1000DPS;
    else if (gyroScale == 2000.f)
        mldl_cfg->full_scale = MPU_FS_2000DPS;

    /* Orientation */
    memset(mldl_cfg->pdata->orientation, 0,
           sizeof(mldl_cfg->pdata->accel.orientation));
    mldl_cfg->pdata->orientation[0] =  1;
    mldl_cfg->pdata->orientation[4] =  1;
    mldl_cfg->pdata->orientation[8] =  1;

    return ML_SUCCESS;
}

/*******************************************************************************
        Mantis Generic - Selection and Setup APIs
*******************************************************************************/

#ifdef M_HW
/**
 * @param side 
 *          Selects the orientation for:
 *              0 = MSB,  
 *              1 = Mounted top of board, Y chip pointing along board (Prototype board)
 *              2 = on top of the USB dongle (demo board with AKM compass)
 */
tMLError SetupMantisCalibration(int side)
{
    struct mldl_cfg* mldl_cfg = MLDLGetCfg();
    struct ext_slave_descr* accel;
    float gyroScale = 2000.f;
    float accelScale = 2.f;

    MPL_LOGI("Calibrating '%s'\n", __func__);


      /*-----------------*/
     /*--- Gyroscope ---*/
    /*-----------------*/

    /* Full Scale */
    if (gyroScale == 250.f)
        mldl_cfg->full_scale = MPU_FS_250DPS;
    else if (gyroScale == 500.f)
        mldl_cfg->full_scale = MPU_FS_500DPS;
    else if (gyroScale == 1000.f)
        mldl_cfg->full_scale = MPU_FS_1000DPS;
    else if (gyroScale == 2000.f)
        mldl_cfg->full_scale = MPU_FS_2000DPS;

    /* Orientation */
    memset( mldl_cfg->pdata->orientation,  0,
           sizeof(mldl_cfg->pdata->orientation));
    if ( side == 0 ) {
        mldl_cfg->pdata->orientation[0] =   1;
        mldl_cfg->pdata->orientation[4] =  -1;
        mldl_cfg->pdata->orientation[8] =  -1;
    } else if ( side == 1) {
        mldl_cfg->pdata->orientation[0] =  1;
        mldl_cfg->pdata->orientation[4] =  1;
        mldl_cfg->pdata->orientation[8] =  1;
    } else if ( side == 2 ) {
        mldl_cfg->pdata->orientation[1] =   1;
        mldl_cfg->pdata->orientation[3] =  -1;
        mldl_cfg->pdata->orientation[8] =   1;
    } else {
        MPL_LOGE("Invalid side parameter %d passed to function '%s'\n", side, __func__);
        return ML_ERROR_INVALID_PARAMETER;
    }

    /* Interrupt */
    mldl_cfg->int_config = BIT_DMP_INT;
    mldl_cfg->pdata->int_config = BIT_INT_ANYRD_2CLEAR|BIT_BYPASS_EN;

      /*---------------------*/
     /*--- Accelerometer ---*/
    /*---------------------*/
    accel = mantis_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(accel->range, 2.f);

    /* Orientation */
    memset(mldl_cfg->pdata->accel.orientation,0,
            sizeof(mldl_cfg->pdata->accel.orientation));
    if ( side == 0 ) {
        mldl_cfg->pdata->accel.orientation[0] =   1;
        mldl_cfg->pdata->accel.orientation[4] =  -1;
        mldl_cfg->pdata->accel.orientation[8] =  -1;    
    } else if ( side == 1) {
        mldl_cfg->pdata->accel.orientation[0] =  1;
        mldl_cfg->pdata->accel.orientation[4] =  1;
        mldl_cfg->pdata->accel.orientation[8] =  1;
    } else if ( side == 2 ) {
        mldl_cfg->pdata->accel.orientation[1] =  1;
        mldl_cfg->pdata->accel.orientation[3] = -1;
        mldl_cfg->pdata->accel.orientation[8] =  1;
    }

    mldl_cfg->pdata->accel.adapt_num       = 0;
    mldl_cfg->pdata->accel.bus             = EXT_SLAVE_BUS_INVALID;
    mldl_cfg->pdata->accel.get_slave_descr = mantis_get_slave_descr;
    mldl_cfg->pdata->accel.address         = mldl_cfg->addr;

    memcpy(mldl_cfg->accel,accel,sizeof(*accel));

    return ML_SUCCESS;
}
#endif 


/*******************************************************************************
        Mantis USB dongle - Selection and Setup APIs
*******************************************************************************/
static 
void SetupCompassAKMCalibration_Dongle (void) 
{
    struct ext_slave_descr* compass;
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();

    MPL_LOGI("Calibrating '%s'\n", __func__);

    compass = ak8975_get_slave_descr();

    /* Full Scale */
    RANGE_FLOAT_TO_FIXEDPOINT(compass->range, 9830.4f);

    /* Orientation */
    memset(mldl_cfg->pdata->compass.orientation,0,
            sizeof(mldl_cfg->pdata->compass.orientation));
    mldl_cfg->pdata->compass.orientation[0] = -1;   // X is -X
    mldl_cfg->pdata->compass.orientation[4] =  1;   // Y is -Y
    mldl_cfg->pdata->compass.orientation[8] = -1;   // Z is  Z

    mldl_cfg->pdata->compass.adapt_num       = 0;
    mldl_cfg->pdata->compass.bus             = EXT_SLAVE_BUS_PRIMARY;
    mldl_cfg->pdata->compass.get_slave_descr = ak8975_get_slave_descr;
    mldl_cfg->pdata->compass.address         = COMPASS_SLAVEADDR_AKM;

    memcpy(mldl_cfg->compass,compass,sizeof(*compass));
}



/*******************************************************************************
    Override the accelerometer default slave address
********************************************************************************/

/**
 *  @brief 
 *  @param  slaveAddr
 *              the accelerometer slave address.
 *  @return ML_SUCCESS or a non-zero error code.
**/
static
tMLError AccelSetupSTLIS331(unsigned char slaveAddr)
{
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    unsigned char *accelSlaveAddr = &(mldl_cfg->pdata->accel.address);

    // slave address check
    if (slaveAddr == ACCEL_SLAVEADDR_INVALID) {
        *accelSlaveAddr = ACCEL_SLAVEADDR_LIS331;
    }
    else if ((slaveAddr & 0x7e) != ACCEL_SLAVEADDR_LIS331) {
        MPL_LOGE("Invalid slave address specification : 0x%02X\n", slaveAddr);
        MPL_LOGE("\tValid addresses are : 0x%02X, 0x%02X\n", 
                        ACCEL_SLAVEADDR_LIS331, ACCEL_SLAVEADDR_LIS331+1);
        return ML_ERROR_INVALID_PARAMETER;
    }
    else {
        *accelSlaveAddr = slaveAddr;
    }

    return ML_SUCCESS;
}


/**
 *  @brief 
 *  @param  slaveAddr
 *              the accelerometer slave address.
 *  @return ML_SUCCESS or a non-zero error code.
**/
static
tMLError AccelSetupSTLSM303(unsigned char slaveAddr)
{
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    unsigned char *accelSlaveAddr = &(mldl_cfg->pdata->accel.address);

    // slave address check
    if (slaveAddr == ACCEL_SLAVEADDR_INVALID) {
        *accelSlaveAddr = ACCEL_SLAVEADDR_LSM303;
    }
    else if ((slaveAddr & 0x7e) != ACCEL_SLAVEADDR_LSM303) {
        MPL_LOGE("Invalid slave address specification : 0x%02X\n", slaveAddr);
        MPL_LOGE("\tValid addresses are : 0x%02X, 0x%02X\n", 
                        ACCEL_SLAVEADDR_LSM303, ACCEL_SLAVEADDR_LSM303+1);
        return ML_ERROR_INVALID_PARAMETER;
    }
    else {
        *accelSlaveAddr = slaveAddr;
    }

    return ML_SUCCESS;
}


/**
 *  @brief
 *  @return ML_SUCCESS or a non-zero error code.
**/
static 
tMLError AccelSetupKionixKXSD9(void)
{
    return ML_SUCCESS;
}


/**
 *  @brief
 *  @return ML_SUCCESS or a non-zero error code.
**/
static 
tMLError AccelSetupKionixKXTF9(void)
{
    return ML_SUCCESS;
}


/**
 *  @brief
 *  @return ML_SUCCESS or a non-zero error code.
**/
static
tMLError AccelSetupBMA150(unsigned char slaveAddr)
{
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    unsigned char *accelSlaveAddr = &(mldl_cfg->pdata->accel.address);

    // slave address check
    if (slaveAddr == ACCEL_SLAVEADDR_INVALID) {
        *accelSlaveAddr = ACCEL_SLAVEADDR_BMA150;
    }
    else if ((slaveAddr & 0x7e) != ACCEL_SLAVEADDR_BMA150) {
        MPL_LOGE("Invalid slave address specification : 0x%02X\n", slaveAddr);
        MPL_LOGE("\tValid addresses are : 0x%02X, 0x%02X\n", 
                    ACCEL_SLAVEADDR_BMA150, ACCEL_SLAVEADDR_BMA150+1);
        return ML_ERROR_INVALID_PARAMETER;
    }
    else {
        *accelSlaveAddr = slaveAddr;
    }
    return ML_SUCCESS;
}
/**
 *  @brief 
 *  @param  slaveAddr
 *              the accelerometer slave address.
 *  @return ML_SUCCESS or a non-zero error code.
**/
static
tMLError AccelSetupBMA222(unsigned char slaveAddr)
{
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    unsigned char *accelSlaveAddr = &(mldl_cfg->pdata->accel.address);

    // slave address check
    if (slaveAddr == ACCEL_SLAVEADDR_INVALID) {
        *accelSlaveAddr = ACCEL_SLAVEADDR_BMA222;
    }
    else if ((slaveAddr & 0x7e) != ACCEL_SLAVEADDR_BMA222) {
        MPL_LOGE("Invalid slave address specification : 0x%02X\n", slaveAddr);
        MPL_LOGE("\tValid addresses are : 0x%02X, 0x%02X\n", 
                    ACCEL_SLAVEADDR_BMA222, ACCEL_SLAVEADDR_BMA222+1);
        return ML_ERROR_INVALID_PARAMETER;
    }
    else {
        *accelSlaveAddr = slaveAddr;
    }
    return ML_SUCCESS;
}

/**
 *  @brief
 *  @return ML_SUCCESS or a non-zero error code.
**/
static
tMLError AccelSetupADI346(unsigned char slaveAddr)
{
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    unsigned char *accelSlaveAddr = &(mldl_cfg->pdata->accel.address);

    // slave address check
    if (slaveAddr == ACCEL_SLAVEADDR_INVALID) {
        *accelSlaveAddr = ACCEL_SLAVEADDR_ADI346;
    }
    else if (slaveAddr != ACCEL_SLAVEADDR_ADI346    &&  
             slaveAddr != ACCEL_SLAVEADDR_ADI346_ALT) {
        MPL_LOGE("Invalid slave address specification : 0x%02X\n", slaveAddr);
        MPL_LOGE("\tValid addresses are : 0x%02X, 0x%02X\n", 
                    ACCEL_SLAVEADDR_ADI346, ACCEL_SLAVEADDR_ADI346_ALT);
        return ML_ERROR_INVALID_PARAMETER;
    }
    else {
        *accelSlaveAddr = slaveAddr;
    }
    return ML_SUCCESS;
}


/**
 *  @brief
 *  @return ML_SUCCESS or a non-zero error code.
**/
static
tMLError AccelSetupMMA8450(unsigned char slaveAddr)
{
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    unsigned char *accelSlaveAddr = &(mldl_cfg->pdata->accel.address);

    // slave address check
    if (slaveAddr == ACCEL_SLAVEADDR_INVALID) {
        *accelSlaveAddr = ACCEL_SLAVEADDR_MMA8450;
    }
    else if (slaveAddr != ACCEL_SLAVEADDR_MMA8450) {
        MPL_LOGE("Invalid slave address specification : 0x%02X\n", slaveAddr);
        MPL_LOGE("\tValid addresses are : 0x%02X\n", ACCEL_SLAVEADDR_MMA8450);
        return ML_ERROR_INVALID_PARAMETER;
    }
    else {
        *accelSlaveAddr = slaveAddr;
    }
    return ML_SUCCESS;
}


/**
 *  @brief
 *  @return ML_SUCCESS or a non-zero error code.
**/
static
tMLError AccelSetupMMA8451(unsigned char slaveAddr)
{
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    unsigned char *accelSlaveAddr = &(mldl_cfg->pdata->accel.address);

    // slave address check
    if (slaveAddr == ACCEL_SLAVEADDR_INVALID) {
        *accelSlaveAddr = ACCEL_SLAVEADDR_MMA8451;
    }
    else if (slaveAddr != ACCEL_SLAVEADDR_MMA8451) {
        MPL_LOGE("Invalid slave address specification : 0x%02X\n", slaveAddr);
        MPL_LOGE("\tValid addresses are : 0x%02X\n", ACCEL_SLAVEADDR_MMA8451);
        return ML_ERROR_INVALID_PARAMETER;
    }
    else {
        *accelSlaveAddr = slaveAddr;
    }
    return ML_SUCCESS;
}


///**
// *  @brief  re-initialize the accel orientation and scale.
// *          This is a simple wrapper for MLSetAccelCalibration() and 
// *          AccelSetRange().
// *  @return ML_SUCCESS or error code.
//**/
//tMLError AccelApplyOrientation(void)
//{
//    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
//    tMLError result;
//    float range = 0.f;
//    
//    RANGE_FIXEDPOINT_TO_FLOAT(mldl_cfg->accel->range, range);
//    //result = AccelSetRange();  
//    //ERROR_CHECK(result);
//    result = MLSetAccelCalibration(range, mldl_cfg->pdata->accel.orientation);
//    ERROR_CHECK(result);
//
//    return ML_SUCCESS;
//}





/*******************************************************************************
    Override the compass default slave address
********************************************************************************/


/**
 *  @internal
 *  @brief  Setup the compass using AICHI compass typical
 *          settings.
 *  @param  slaveAddr
 *              The compass slave address.
 *  @return ML_SUCCESS or a non-zero error code.
**/
static
tMLError CompassSetupAICHI(unsigned char slaveAddr)
{
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    unsigned char *compassSlaveAddr = &(mldl_cfg->pdata->compass.address);

    // slave address check
    if (slaveAddr == COMPASS_SLAVEADDR_INVALID) {
        *compassSlaveAddr = COMPASS_SLAVEADDR_AICHI;
    }
    else if ((slaveAddr & 0x7e) != COMPASS_SLAVEADDR_AICHI) {
        MPL_LOGE("Invalid slave address specification : 0x%02X\n", slaveAddr);
        MPL_LOGE("\tValid addresses are : 0x%02X, 0x%02X\n", 
                        COMPASS_SLAVEADDR_AICHI, COMPASS_SLAVEADDR_AICHI+1);
        return ML_ERROR_INVALID_PARAMETER;
    }
    else {
        *compassSlaveAddr = slaveAddr;
    }
    return ML_SUCCESS;
}

/**
 *  @internal
 *  @brief  Setup the compass using AKM compass typical
 *          settings.
 *  @param  slaveAddr
 *              compass slave address.
 *  @return ML_SUCCESS or a non-zero error code.
**/
static
tMLError CompassSetupAKM(unsigned char slaveAddr)
{
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    unsigned char *compassSlaveAddr = &(mldl_cfg->pdata->compass.address);

    // slave address check
    if (slaveAddr == COMPASS_SLAVEADDR_INVALID) {
        *compassSlaveAddr = COMPASS_SLAVEADDR_AKM;
    }
    else if ((slaveAddr & 0x7e) != COMPASS_SLAVEADDR_AKM) {
        MPL_LOGE("Invalid slave address specification : 0x%02X\n", slaveAddr);
        MPL_LOGE("\tValid addresses are : 0x%02X, 0x%02X\n", 
                        COMPASS_SLAVEADDR_AKM, COMPASS_SLAVEADDR_AKM+1);
        return ML_ERROR_INVALID_PARAMETER;
    }
    else {
        *compassSlaveAddr = slaveAddr;
    }
    return ML_SUCCESS;
}

/**
 *  @internal
 *  @brief  Setup the compass using AKM compass typical
 *          settings.
 *  @param  slaveAddr
 *              compass slave address.
 *  @return ML_SUCCESS or a non-zero error code.
**/
static
tMLError CompassSetupMMC314X(unsigned char slaveAddr)
{
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    unsigned char *compassSlaveAddr = &(mldl_cfg->pdata->compass.address);

    // slave address check
    if (slaveAddr == COMPASS_SLAVEADDR_INVALID) {
        *compassSlaveAddr = COMPASS_SLAVEADDR_MMC314X;
    }
    else if ((slaveAddr & 0x7e) != COMPASS_SLAVEADDR_MMC314X) {
        MPL_LOGE("Invalid slave address specification : 0x%02X\n", slaveAddr);
        MPL_LOGE("\tValid addresses are : 0x%02X, 0x%02X\n", 
                        COMPASS_SLAVEADDR_MMC314X, COMPASS_SLAVEADDR_MMC314X+1);
        return ML_ERROR_INVALID_PARAMETER;
    }
    else {
        *compassSlaveAddr = slaveAddr;
    }
    return ML_SUCCESS;
}

/**
 *  @internal
 *  @brief  Setup the compass using YAMAHA YAS529 compass typical
 *          settings.
 *  @return ML_SUCCESS or a non-zero error code.
**/
static
tMLError CompassSetupYAS529(void)
{
    return ML_SUCCESS;
}

/**
 *  @internal
 *  @brief  Setup the compass using HONEYWELL HMC5883 compass typical
 *          settings.
 *  @return ML_SUCCESS or a non-zero error code.
**/
static
tMLError CompassSetupHMC5883(void)
{
    return ML_SUCCESS;
}

/**
 *  @internal
 *  @brief  Setup the compass using ST LSM303 compass typical
 *          settings.
 *  @return ML_SUCCESS or a non-zero error code.
**/
static
tMLError CompassSetupLSM303(void)
{
    return ML_SUCCESS;
}

/**
 *  @internal
 *  @brief  Setup the compass using ALPS HSCDTD002B compass typical
 *          settings.
 *  @return ML_SUCCESS or a non-zero error code.
**/
static
tMLError CompassSetupHSCDTD002B(unsigned char slaveAddr)
{
    struct mldl_cfg * mldl_cfg = MLDLGetCfg();
    unsigned char *compassSlaveAddr = &(mldl_cfg->pdata->compass.address);

    // slave address check
    if (slaveAddr == COMPASS_SLAVEADDR_INVALID) {
        *compassSlaveAddr = COMPASS_SLAVEADDR_HSCDTD002B;
    }
    else if ((slaveAddr & 0x7c) != COMPASS_SLAVEADDR_HSCDTD002B) {
        MPL_LOGE("Invalid slave address specification : 0x%02X\n", slaveAddr);
        MPL_LOGE("\tValid addresses are : 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", 
                        COMPASS_SLAVEADDR_HSCDTD002B, COMPASS_SLAVEADDR_HSCDTD002B+1,
                        COMPASS_SLAVEADDR_HSCDTD002B+2, COMPASS_SLAVEADDR_HSCDTD002B+3);
        return ML_ERROR_INVALID_PARAMETER;
    }
    else {
        *compassSlaveAddr = slaveAddr;
    }
    return ML_SUCCESS;
}

/**
 *  @brief  Setup a slave address for the specified compass id
 *          different from the default one.
 *          Some compass devices can be setup to respond to a
 *          slave address different from the default.  This is 
 *          typically achieved by pulling high the address pin on the 
 *          device, although some chip might implement this differently.
 *          If the calling function passes in a unsopported slave address 
 *          the default slave address is assumed and an error code is 
 *          returned.
 *
 *  @param  compassId  
 *              compass slave address.
 *  @param  slaveAddr
 *              alternative compass I2C slave address.
 *
 *  @return ML_SUCCESS or a non-zero error code.
 */
tMLError CompassSetup(unsigned short compassId, unsigned char slaveAddr)
{
    tMLError result;

    switch(compassId) {

        case COMPASS_ID_AKM:
            result = CompassSetupAKM(slaveAddr);
            break;
        case COMPASS_ID_AICHI:
            result = CompassSetupAICHI(slaveAddr);
            break;
        case COMPASS_ID_YAS529:
            result = CompassSetupYAS529();
            break;
        case COMPASS_ID_HMC5883:
            result = CompassSetupHMC5883();
            break;
        case COMPASS_ID_LSM303:
            result = CompassSetupLSM303();
            break;
        case COMPASS_ID_MMC314X:
            result = CompassSetupMMC314X(slaveAddr);
            break;
        case COMPASS_ID_HSCDTD002B:
            result = CompassSetupHSCDTD002B(slaveAddr);
            break;

        case ID_INVALID:
        default:
            result = ML_SUCCESS;
            break;
    }
    
    return result;
}





/***********************/
/** @} */ /* defgroup */
/*********************/


