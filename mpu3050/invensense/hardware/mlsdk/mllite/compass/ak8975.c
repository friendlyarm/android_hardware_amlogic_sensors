/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/**
 *  @defgroup   COMPASSDL (Motion Library - Accelerometer Driver Layer)
 *  @brief      Provides the interface to setup and handle an accelerometers
 *              connected to the secondary I2C interface of the gyroscope.
 *
 *  @{
 *      @file   AK8975.c
 *      @brief  Magnetometer setup and handling methods for AKM 8975 compass.
**/

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#ifdef __KERNEL__
#include <linux/module.h>
#endif

#include "mpu.h"
#include "mlsl.h"
#include "mlos.h"

#include <log.h>
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-compass"


#define AK8975_REG_ST1  (0x02)
#define AK8975_REG_HXL  (0x03)
#define AK8975_REG_ST2  (0x09)

#define AK8975_REG_CNTL (0x0A)

#define AK8975_CNTL_MODE_POWER_DOWN         (0x00)
#define AK8975_CNTL_MODE_SINGLE_MEASUREMENT (0x01)

int ak8975_suspend(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  AK8975_REG_CNTL,
				  AK8975_CNTL_MODE_POWER_DOWN);
	MLOSSleep(1);		/* wait at least 100us */
	ERROR_CHECK(result);
	return result;
}

int ak8975_resume(void *mlsl_handle,
		  struct ext_slave_descr *slave,
		  struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  AK8975_REG_CNTL,
				  AK8975_CNTL_MODE_SINGLE_MEASUREMENT);
	ERROR_CHECK(result);
	return result;
}

int ak8975_read(void *mlsl_handle,
		struct ext_slave_descr *slave,
		struct ext_slave_platform_data *pdata, unsigned char *data)
{
	unsigned char stat;
	unsigned char stat2;
	int result = ML_SUCCESS;

	result =
	    MLSLSerialRead(mlsl_handle, pdata->address, AK8975_REG_ST1, 1,
			   &stat);
	ERROR_CHECK(result);
	if (stat & 0x01) {
		result =
		    MLSLSerialRead(mlsl_handle, pdata->address,
				   AK8975_REG_HXL, 6,
				   (unsigned char *) data);
		ERROR_CHECK(result);
		result =
		    MLSLSerialRead(mlsl_handle, pdata->address,
				   AK8975_REG_ST2, 1, &stat2);
		ERROR_CHECK(result);
		if (stat2 & 0x04) /*data error */
			return ML_ERROR_COMPASS_DATA_NOT_READY;
		if (stat2 & 0x08)
			return ML_ERROR_COMPASS_DATA_OVERFLOW;

		result =
		    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					  AK8975_REG_CNTL,
					  AK8975_CNTL_MODE_SINGLE_MEASUREMENT);
		ERROR_CHECK(result);
		return ML_SUCCESS;
	} else if (stat & 0x02) {
		result =
		    MLSLSerialRead(mlsl_handle, pdata->address,
				   AK8975_REG_ST2, 1, &stat2);
		ERROR_CHECK(result);
		return ML_ERROR_COMPASS_DATA_OVERFLOW;
	} else {
		return ML_ERROR_COMPASS_DATA_NOT_READY;
	}
}

struct ext_slave_descr ak8975_descr = {
	/*.suspend          = */ ak8975_suspend,
	/*.resume           = */ ak8975_resume,
	/*.read             = */ ak8975_read,
	/*.name             = */ "ak8975",
	/*.type             = */ EXT_SLAVE_TYPE_COMPASS,
	/*.id               = */ COMPASS_ID_AKM,
	/*.reg              = */ 0x03,
	/*.len              = */ 6,
	/*.endian           = */ EXT_SLAVE_LITTLE_ENDIAN,
	/*.range            = */ {9830, 4000}
};

struct ext_slave_descr *ak8975_get_slave_descr(void)
{
	return &ak8975_descr;
}

#ifdef __KERNEL__
EXPORT_SYMBOL(ak8975_get_slave_descr);
#endif

/**
 *  @}
**/
