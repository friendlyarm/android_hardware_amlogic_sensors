
/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: pedometerStandAlone.h 3863 2010-10-08 22:05:31Z nroyer $
 *
 ******************************************************************************/

#ifndef MLPEDOMETER_LOWPOWER_C_H
#define MLPEDOMETER_LOWPOWER_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"
#include "mlpedometer_lowpower.h"

    /* ------------ */
    /* - Defines. - */
    /* ------------ */

    tMLError MLEnablePedometerFullPower(void);
    tMLError MLDisablePedometerFullPower(void);
    tMLError MLSetPedometerFullPowerStepCount(unsigned long steps);
    tMLError MLGetPedometerFullPowerStepCount(unsigned long *steps);
    tMLError MLSetPedometerFullPowerStepCallback(
        void (*func) (unsigned long stepNum) );

    tMLError MLSetPedometerFullPowerParams(const struct stepParams *params);

#ifdef __cplusplus
}
#endif

#endif /* PEDOMETER_STAND_ALONE_H */
