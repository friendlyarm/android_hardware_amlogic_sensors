/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: log_printf_linux.c 4073 2010-11-16 01:09:35Z mcaramello $ 
 *
 ******************************************************************************/
 
/**
 * @addtogroup MPL_LOG
 *
 * @{
 *      @file   log_printf.c
 *      @brief  printf replacement for _MLWriteLog.
 */

#include <stdio.h>
#include "log.h"

int _MLWriteLog (const char * buf, int buflen)
{
    return puts(buf);
}

/**
 * @}
 */

