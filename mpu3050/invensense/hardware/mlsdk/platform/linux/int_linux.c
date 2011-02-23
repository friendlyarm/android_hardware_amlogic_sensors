/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: int_linux.c 3865 2010-10-08 23:09:22Z nroyer $
 *
 ******************************************************************************/

/******************************************************************************/
/** 
    @defgroup INT
    @brief  This file contains the routines used to detect the Invensense Motion
            Processing Interrupts.
    
    @{
        @file int.c
        @brief Diagnostics Interface Module

    THIS NEEDS TO BE IMPLEMENTED DIFFERENTLY FOR EACH PLATFORM

*/
/******************************************************************************/

#define INT_C

/* ------------- */
/* - Includes. - */
/* ------------- */

#ifndef LINUX
#error Trying to build __FILE__ without LINUX defined
#endif

#include <string.h>
#include <fcntl.h>
#include "mltypes.h"
#include "mlsl.h"
#include "mldl.h"
#include "int.h"
#include "log.h"
#include <errno.h>
#include <sys/select.h>
#include <sys/time.h>
#include "kernel/mpuirq.h"

/* --------------------- */
/* - Global Variables. - */
/* --------------------- */

/* --------------------- */
/* - Static Variables. - */
/* --------------------- */

static int imeIntHandle = -1;

/* --------------------- */
/* - Static Functions. - */
/* --------------------- */


/**
 *  @brief This routine opens the interrupt handling
 *
 *  @param dev Name of the character device to open support interrupts or 
 *             NULL to use the default of "/dev/mpu-0"

 *  @return error code.
 */
tMLError IntOpen(const char * dev)
{
    if (NULL == dev) {
        dev = "/dev/mpuirq";
    }
    
    imeIntHandle = open(dev,O_RDWR);
    if (imeIntHandle == -1) {
        MPL_LOGE("%s open error %d\n", dev, errno);
        return ML_ERROR_FEATURE_NOT_ENABLED;
    } else {
        MPL_LOGI("Opened %s: %d\n",dev, imeIntHandle);
        return ML_SUCCESS;
    }

    return ML_ERROR;
}



/**
 * @brief   This function should be called from the main event loop in systems 
 *          that support interrupt polling.
 * @param data Data read
 * @param tv_sec timeout value in seconds
 * @param tv_usec timeout value in micro seconds
 */
int IntProcess(struct mpuirq_data *data, long tv_sec, long tv_usec)
{
    int numRead = 0;
    int result;
    int size;
    fd_set read_fd;
    fd_set excep_fd;
    struct timeval timeout;

    if (data != NULL) {
        size = sizeof(*data) + data->data_size;
    } else {
        size = 0;
    }
    
    FD_ZERO(&read_fd);
    FD_ZERO(&excep_fd);
    FD_SET(imeIntHandle, &read_fd);
    FD_SET(imeIntHandle, &excep_fd);
    timeout.tv_sec = tv_sec;
    timeout.tv_usec = tv_usec;

    result = select((imeIntHandle+1), &read_fd, NULL, &excep_fd, &timeout);
    if (result < 0) 
        return result;
    
    /* Timeout */
    if (0 == result) {
        return ML_SUCCESS;
    }

    if (FD_ISSET(imeIntHandle, &read_fd)) {
        numRead = read(imeIntHandle,data,size);   // blocking read
    }
    
    return numRead;
}

tMLError IntDebugKernel(int on)
{
    MPL_LOGV("Calling ioctl with %d, %d\n",
             MPUIRQ_ENABLE_DEBUG,
             on);
    ioctl(imeIntHandle,MPUIRQ_ENABLE_DEBUG,on);
    return ML_SUCCESS;
}

tMLError IntSetTimeout(int timeout)
{
    MPL_LOGV("Calling ioctl with %d, %d\n",
             MPUIRQ_SET_TIMEOUT,
             timeout);
    ioctl(imeIntHandle,MPUIRQ_SET_TIMEOUT,timeout);
    return ML_SUCCESS;
}

/**
 *  @brief closes the interrupt handling.
 *  @return ML_SUCCESS or non-zero error code
 */
tMLError IntClose(void)
{
    close(imeIntHandle);

    return ML_SUCCESS;
}

  /**********************/
 /** @} */ /* defgroup */
/**********************/
