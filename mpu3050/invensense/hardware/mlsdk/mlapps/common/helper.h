/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
#ifndef HELPER_C_H
#define HELPER_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mpu.h"
#include "mlerrorcode.h"

#define CALL_N_CHECK(f) {\
    unsigned int result = f;\
    if(ML_SUCCESS != result) {\
        printf("Error : %s returned code #%d\n", #f, result);\
    } \
}

#define CALL_CHECK_N_RETURN(f) {\
    unsigned int result = f;\
    if(ML_SUCCESS != result) {\
        printf("Error : %s returned code #%d\n", #f, result);\
        return result; \
    } \
}

#define CALL_CHECK_N_EXIT(f, exitfunc) {\
    unsigned int result = f;\
    if(ML_SUCCESS != result) {\
        printf("Error : %s returned code #%d\n", #f, result);\
        exitfunc; \
    } \
}

#define CALL_CHECK_N_GOTO(f, label) {\
    unsigned int result = f;\
    if(ML_SUCCESS != result) {\
        printf("Error : %s returned code #%d\n", #f, result);\
        goto label; \
    } \
}

#define DataLoggerSelector(x)   //
#define DataLoggerCb(x)         NULL
int             findComm            ( char * port, int size );
tMLError        findHw              ( void *mlsl_handle, unsigned short* accelId, unsigned short* compassId );
tMLError        MenuHwChoice        ( unsigned short* accelId, unsigned short* compassId );

#ifdef __cplusplus
}
#endif

#endif // HELPER_C_H
