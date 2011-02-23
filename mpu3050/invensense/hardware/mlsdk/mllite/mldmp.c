/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 *
 * $Id: mldmp.c 4250 2010-12-08 01:29:04Z prao $
 *
 ******************************************************************************/

/**
 * @addtogroup MLDMP
 *
 * @{
 *      @file     mldmp.c
 *      @brief    Shared functions between all the different DMP versions
**/

#include <stdio.h>

#include "mltypes.h"
#include "mlinclude.h"
#include "mltypes.h"
#include "ml.h"
#include "mldl_cfg.h"
#include "mldl.h"
#include "compass.h"

#include "mlsl.h"
#include "mlFIFO.h"
#include "mldmp.h"
#include "mlstates.h"
#include "dmpDefault.h"
#include "mlFIFOHW.h"
#include "mlsupervisor.h"

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-dmp"

/**
 *  @brief  Open the default motion sensor engine.
 *          This function is used to open the default MPL engine, 
 *          featuring, for example, sensor fusion (6 axes and 9 axes), 
 *          sensor calibration, accelerometer data byte swapping, among 
 *          others.  
 *          Compare with the other provided engines.
 *
 *  @pre    MLSerialOpen() must have been called to instantiate the serial 
 *          communication.
 *  
 *  Example:
 *  @code
 *    result = MLDmpOpen( );
 *    if (ML_SUCCESS != result) {
 *        // Handle the error case
 *    }
 *  @endcode
 *
 *  @return Zero on success; Error Code on any failure.
 *
 */
tMLError MLDmpOpen(void)
{
    INVENSENSE_FUNC_START;
    tMLError result;

    /*************************************************************
     * Common operations before calling DMPOpen
     ************************************************************/
    if (MLStateTransition(ML_STATE_DMP_OPENED)) {
        MPL_LOGE("Error : Serial COM port closed\n");
        return ML_ERROR_SERIAL_CLOSED;
    }

#ifdef ML_USE_DMP_SIM
    do {
        void setup_univ();
        setup_univ();  /* hijack the read and write paths and re-direct them to the simulator */
    } while(0);
#endif

    result = dmpDefault();
    ERROR_CHECK(result);

    // Init vars.
    MLXInit();

    result = FIFOParamInit();
    ERROR_CHECK(result);
    FIFOHWInit();
    result = MLApplyCalibration();
    ERROR_CHECK(result);
    result = MLApplyAccelEndian();

    return result;
}

/**
 *  @brief  Start the DMP.
 *
 *  @pre    MLDmpOpen() must have been called.
 * 
 *  @code
 *     result = MLDmpStart();
 *     if (ML_SUCCESS != result) {
 *         // Handle the error case
 *     }
 *  @endcode
 *
 *  @return ML_SUCCESS if successful, or Non-zero error code otherwise.
 */
tMLError MLDmpStart(void)
{
    INVENSENSE_FUNC_START;
    tMLError result;

    result = MLStateTransition(ML_STATE_DMP_STARTED);
    ERROR_CHECK(result);
    FIFOHWInit();
    result = MLResetMotion();
    ERROR_CHECK(result);
    MLSensorFusionSupervisorInit();
    result = MLDLDmpStart();
    ERROR_CHECK(result);

    return result;
}

/**
 *  @brief  Stops the DMP and puts it in low power.
 *
 *  @pre    MLDmpStart() must have been called.
 * 
 *  @return ML_SUCCESS, Non-zero error code otherwise.
 */
tMLError MLDmpStop(void)
{
    INVENSENSE_FUNC_START;
    tMLError result;

    result = MLStateTransition(ML_STATE_DMP_OPENED);
    ERROR_CHECK(result);
    result = MLDLDmpStop();
    ERROR_CHECK(result);

    return result;
}

/**
 *  @brief  Closes the motion sensor engine.
 *          Does not close the serial communication. To do that,
 *          call MLSerialClose().
 *          After calling MLDmpClose() another DMP module can be
 *          loaded in the MPL with the corresponding necessary 
 *          intialization and configurations, via any of the 
 *          MLDmpOpenXXX functions.
 *
 *  @pre    MLDmpOpen() must have been called.
 * 
 *  @code
 *     result = MLDmpClose();
 *     if (ML_SUCCESS != result) {
 *         // Handle the error case
 *     }
 *  @endcode
 *
 *  @return ML_SUCCESS, Non-zero error code otherwise.
 */
tMLError MLDmpClose(void)
{
    INVENSENSE_FUNC_START;
    tMLError result;
    tMLError firstError = ML_SUCCESS;

    result = MLDLDmpStop();
    ERROR_CHECK_FIRST(firstError, result);
 
    result = FIFOClose();
    ERROR_CHECK_FIRST(firstError, result);

    result = MLStateTransition(ML_STATE_SERIAL_OPENED);
    ERROR_CHECK_FIRST(firstError, result);

    return result;
}

/**
 *  @}
**/


