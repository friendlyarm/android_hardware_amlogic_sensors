/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/**
 * Version string setup for the sensor manager/gesture manager
 */

#define SM_VERSION_MAJOR                 3
#define SM_VERSION_MINOR                 2
#define SM_VERSION_SUB_MINOR             0

#define SM_VERSION_MAJOR_STR            "3"
#define SM_VERSION_MINOR_STR            "2"
#define SM_VERSION_SUB_MINOR_STR        "0"

#define SM_VERSION_ENGINEERING          "EngA"
#define SM_VERSION_PRE_ALPHA            "Pre-Alpha"
#define SM_VERSION_ALPHA                "Alpha"
#define SM_VERSION_BETA                 "Beta"
#define SM_VERSION_PRODUCTION           "Prod"

#ifndef SM_VERSION_TYPE
#define SM_VERSION_TYPE ML_VERSION_ALPHA
#endif

#define SM_VERSION  "InvenSense Android Sensor Integration" " " SM_VERSION_TYPE " "\
    "v" SM_VERSION_MAJOR_STR "." SM_VERSION_MINOR_STR "." SM_VERSION_SUB_MINOR_STR " " \
    __DATE__ " " __TIME__

