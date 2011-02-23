/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: pedometer.h 3863 2010-10-08 22:05:31Z nroyer $
 *
 *******************************************************************************/

#ifndef PEDOMETER_H
#define PEDOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"

    /* ------------ */
    /* - Defines. - */
    /* ------------ */


    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */
    tMLError  MLEnablePedometer();
    tMLError  MLDisablePedometer();
    tMLError  MLSetStepCallback(void (*func) (unsigned short stepNum) );
    tMLError  MLGetNumOfSteps(unsigned int *steps);
    tMLError  MLClearNumOfSteps();
    tMLError  MLSetStrideLength(unsigned short strideLength);
    tMLError  MLPedometerSetDataRate( int dataRate );
    tMLError  MLPedometerSetDirection(int dir);

    float PedometerHeadingToVelocity();
    float PedometerHeading();

    /* Internal functions */
    void startPedometerLog();
    void endPedometerLog();
    long sqrti( long x );

#ifdef __cplusplus
}
#endif

#endif /* PEDOMETER_H */
