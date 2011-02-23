/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: helper.c 3698 2010-09-08 21:08:34Z yserlin $
 *
 *******************************************************************************/

#include <stdio.h>
#ifdef WIN32
#include <windows.h>
#endif

#include "ml.h"
#include "accel.h"
#include "mldl.h"
#include "mltypes.h"
#include "mlstates.h"
#include "compass.h"

#include "mlsl.h"

#include "helper.h"
#include "mlsetup.h"
#include "fopenCMake.h"

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-helper"



/** 
 *  Tries to find the serial port for the hardware
**/
int findComm(char * port,int size)
{
    return -1;
}

tMLError findHw(void *mlsl_handle, unsigned short* pAccelId, unsigned short* pCompassId)
{
    *pAccelId   =  ACCEL_KIONIX_KXTF9;
    *pCompassId = ID_INVALID;
    return ML_SUCCESS;
}


tMLError MenuHwChoice(unsigned short* pAccelId, unsigned short* pCompassId)
{
    char override = '\0';

    if (NULL == pAccelId || NULL == pCompassId) 
        return ML_ERROR_INVALID_PARAMETER;

    *pAccelId   = 0xffff;
    *pCompassId = 0xffff;

    while (override != 'y' && override != 'n' && override != '\n') {
        printf(
            "\n"
            "Override driver [y,N]:\n");
        scanf("%c",&override);
    }

    if (override == '\n' || override == 'n')
        return ML_ERROR_FEATURE_NOT_ENABLED;

    printf(
        "\n"
        "Accelerometers:\n"
        "\t0 - Default\n"
        "\t1 - ST LIS331\n"
        "\t2 - ST LSM303\n"
        "\t3 - Kionix KXSD9\n"
        "\t4 - Kionix KXTF9\n"
        "\t5 - Bosch BMA150\n"
        "\t6 - Bosch BMA222\n"
        "\t7 - ADI\n"
        "\t8 - FreeScale 8450 \n"
        "\t9 - FreeScale 8451 \n"
        "\n"
    );
    while (*pAccelId==0xffff || *pAccelId>9) {
        printf("Which accelerometer ? ");
        scanf("%hd", pAccelId);
    }

    printf(
        "\n"
        "Compasses:\n"
        "\t0  - Default\n"
        "\t10 - AKM\n"
        "\t11 - AICHI\n"
        "\t12 - Yamaha\n"
        "\t13 - Honeywell\n"
        "\t14 - ST LSM303\n"
        "\t15 - Memsic\n"
        "\t16 - Alps HSCDTD002B\n"
        "\n"
    );
    while (*pCompassId==0xffff 
           || (*pCompassId >COMPASS_ID_HSCDTD002B
           && (*pCompassId <COMPASS_ID_AKM) && *pCompassId != ID_INVALID)) {
        printf("Which compass ? ");
        scanf("%hd", pCompassId);
    }

    return ML_SUCCESS;
}

