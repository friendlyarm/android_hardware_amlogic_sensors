/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

#ifndef __MPUIRQ__
#define __MPUIRQ__

#ifdef __KERNEL__
#include <linux/i2c-dev.h>
#endif

#define MPUIRQ_ENABLE_DEBUG          (1)
#define MPUIRQ_GET_INTERRUPT_CNT     (2)
#define MPUIRQ_GET_IRQ_TIME          (3)
#define MPUIRQ_GET_LED_VALUE         (4)
#define MPUIRQ_SET_TIMEOUT           (5)
#define MPUIRQ_SET_ACCEL_INFO        (6)
#define MPUIRQ_SET_FREQUENCY_DIVIDER (7)

struct mpuirq_data {
	int interruptcount;
	unsigned long long irqtime;
	int data_type;
	int data_size;
	void *data;
};

#ifdef __KERNEL__

void mpuirq_exit(void);
int mpuirq_init(struct i2c_client *mpu_client);

#endif

#endif
