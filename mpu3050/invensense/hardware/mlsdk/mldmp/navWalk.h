/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: navWalk.h 3863 2010-10-08 22:05:31Z nroyer $
 *
 *******************************************************************************/

#ifndef NAVWALK_H
#define NAVWALK_H

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
    int  NavWalkEnable();
    int  NavWalkDisable();
    int  NavWalkSetStepCallback(void (*func) (unsigned short stepNum) );
    int  NavWalkGetNumOfSteps(unsigned int *steps);
    int  NavWalkClearNumOfSteps();
    int  NavWalkSetStrideLength(unsigned short strideLength);
    int  NavWalkSetDataRate( int dataRate );
    float NavWalkHeadingToVelocity();
    float NavWalkHeading();

    // Internal Debug Functions
    void NavWalkStartLog();
    void NavWalkEndLog();

#ifdef __cplusplus
}
#endif

#endif /* NAVWALK_H */
