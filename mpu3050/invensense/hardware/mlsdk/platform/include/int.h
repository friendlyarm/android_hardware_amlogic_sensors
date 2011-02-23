/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: int.h 3865 2010-10-08 23:09:22Z nroyer $
 *
 *******************************************************************************/

#ifndef _INT_H
#define _INT_H

#include "mltypes.h"
#include "kernel/mpuirq.h"

#ifdef __cplusplus
extern "C" {
#endif

    /* ------------ */
    /* - Defines. - */
    /* ------------ */

    /* ---------- */
    /* - Enums. - */
    /* ---------- */

    /* --------------- */
    /* - Structures. - */
    /* --------------- */

    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */

    tMLError IntOpen(const char * dev);
    int IntProcess(struct mpuirq_data *data, long tv_sec, long tv_usec);
    tMLError IntClose(void);
    tMLError IntSetTimeout(int timeout);

    tMLError IntDebugKernel(int on);

#ifdef __cplusplus
}
#endif

#endif /* _TEMPLATE_H */
