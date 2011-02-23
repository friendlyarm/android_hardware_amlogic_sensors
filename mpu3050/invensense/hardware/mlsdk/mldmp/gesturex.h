/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: gesturex.h 3863 2010-10-08 22:05:31Z nroyer $
 *
 * $Date: 2010-10-08 15:05:31 -0700 (Fri, 08 Oct 2010) $
 *
 * $Author: nroyer $
 *
 * $Revision: 3863 $
 *
 *******************************************************************************/
/*******************************************************************************/
/** @defgroup ML_GESTUREX
    
    @{
        @file gesturex.h
        @brief Header file for the Gesture recognition algorithms.

*/
/******************************************************************************/
#ifndef GESTUREX_H
#define GESTUREX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"
#include "ml.h"
#include "gesture_tap_dmp.h"

    /* ------------ */
    /* - Defines. - */
    /* ------------ */

    /* - Module defines. - */


    /* --------------- */
    /* - Structures. - */
    /* --------------- */

    /**************************************************************************/
    /* MLGSTRX_Data_t Structure.                                              */
    /**************************************************************************/

    typedef struct {
        tGesture       gesture;
        int            gestureState;
        unsigned int   tapNumber;
        unsigned int   pulseNumber[3];
        unsigned int   pulseNumberReported[3];
        float          imageRotationIntegral;
        tMLGstrMultiTapHandle mlMultTapHandle;

    } tMLGstrxData,         // new type definition
      MLGSTRX_Data_t;       // backward-compatible type definition

    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */

    /**************************************************************************/
    /* ML Gesturex Functions.                                                 */
    /**************************************************************************/

    void MLGstrxInit(void);

    //Gesture recognition algorithm
    void GestureResetInterp(void);
    void TapResetInterp(void);
    tMLError  UpdateGestures( tMLXData *mlxData );
    void MLGestureReset(unsigned short param);
    void ShakeReset(int axis);

#ifdef __cplusplus
}
#endif

#endif /* GESTUREX_H */
