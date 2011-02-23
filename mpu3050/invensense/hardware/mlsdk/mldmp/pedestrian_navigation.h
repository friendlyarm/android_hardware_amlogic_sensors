/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
#ifndef ML_PEDESTRIAN_NAVIGATION_H__
#define ML_PEDESTRIAN_NAVIGATION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"

int MLEnablePedestrianNavigation();
int MLDisablePedestrianNavigation();
int MLPedestrianNavigationSetCallback( void (*func)(float x, float y, float heading) );
int MLPedestrianNavigationSetPosition( float x, float y );
int MLPedestrianNavigationSetHeading( );
int MLPedestrianNavigationSetStepSize( float stepsize );
void MLPedestrianNavigationGetUserLocation( float *x, float *y, float *heading );

#ifdef __cplusplus
}
#endif


#endif // ML_PEDESTRIAN_NAVIGATION_H__
