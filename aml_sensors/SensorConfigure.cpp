
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/input.h>
#include <cutils/log.h>
#include "SensorConfigure.h"

#ifndef ALOGD
#define ALOGD	LOGD
#define ALOGE	LOGE
#define ALOGV	LOGV
#define ALOGE_IF	LOGE_IF
#define ALOGD_IF	LOGD_IF
#define ALOGV_IF	LOGV_IF
#endif

static int dither(int value, int old_val, int fuzz)
{
	if (value >= old_val - (fuzz>>1)  && value <= old_val + (fuzz>>1))
		return old_val;

	if (value > old_val - fuzz && value < old_val + fuzz)
		return (old_val * 3 + value) >> 2;

	if (value > old_val - fuzz * 2 && value < old_val + fuzz * 2)
		return (old_val + value) >> 1;

	return value;
}

static int average(int raw_val, int xyz)
{
#define FILTER_LEN	8	//Must be a power of 2, otherwise it'll consume to much cpu.
#define EN_ROUNDOFF	0 
        static int idx;
        static int raw_xyz[3][FILTER_LEN];
		static int prev_result[3];
        int i, raw_sum=0;
        int result, sum_below;
        if(idx >= FILTER_LEN)
                idx = 0;
        raw_xyz[xyz][idx] = raw_val;
        idx++;
        for(i = 0; i < FILTER_LEN; i++)
        {
                raw_sum += raw_xyz[xyz][i];
        }

	result = raw_sum/FILTER_LEN;

#if EN_ROUNDOFF
	sum_below = result * FILTER_LEN;


	if(raw_sum != sum_below)	//A round-off is needed	
	{//Round off
		int roundoff_result;
		int roundoff_sum;
		if(raw_sum <0)
		{
			roundoff_result = result -1;
			roundoff_sum = roundoff_result * FILTER_LEN;
		}
		else //raw_sum is greater than 0
		{
			roundoff_result = result + 1;
			roundoff_sum = roundoff_result * FILTER_LEN;
		}
		//roundoff_sum is closer to raw_sum, use roundoff_result instread
		if(abs(raw_sum - sum_below) > abs(raw_sum - roundoff_sum))
		{
			result = roundoff_result;

		}
		else if(abs(raw_sum - sum_below) == abs(raw_sum - roundoff_sum))
		{
			result = prev_result[xyz];
		}
	}
	prev_result[xyz] = result;
#endif	
	return result;
}

int mma7660_filter(unsigned int code, int val)
{
#define AMPLIFY_SHIFT	4
#define FUZZ		(1<<AMPLIFY_SHIFT)
	static int xyz_prev[3];
	int idx=0;
	switch(code)
	{
		case ABS_X:
			idx = 0;break;
		case ABS_Y:
			idx = 1;break;
		case ABS_Z:
			idx = 2;break;
	}
	val <<= AMPLIFY_SHIFT;
	val = dither(val, xyz_prev[idx]<<AMPLIFY_SHIFT, FUZZ);
	val = average(val, idx);

	val /= FUZZ;	//Since val could be negative, we can't use shift to replace division. Better way?
	xyz_prev[idx] = val;

	return val;
}
static char dummy_gsensor_name[80] = "dummy gravity sensor";
const static struct sensor_config supported_sensors[] =
{
		{"bma250", AML_SENSOR_TYPE_GRAVITY, {{0, 256.0f}}},
		{"bma222", AML_SENSOR_TYPE_GRAVITY, {{0, 64.0f}}},
		{"mc32x0", AML_SENSOR_TYPE_GRAVITY, {{0, 86.0f}}},
		{"mma7660", AML_SENSOR_TYPE_GRAVITY, {{mma7660_filter, 21.0f}}},
		{"mma8452", AML_SENSOR_TYPE_GRAVITY, {{0, 1024.0f}}},
		{"dmard06", AML_SENSOR_TYPE_GRAVITY, {{0, 32.0f}}},
		{"afa750", AML_SENSOR_TYPE_GRAVITY, {{0, 4096.0f}}},

		//This should  always be the last node of this array, don't touch.
		{NULL, AML_SENSOR_TYPE_NONE, {{0, 0}}},	
};	

const static struct sensor_config dummy_sensors[] =
{
		{(const char *)dummy_gsensor_name, AML_SENSOR_TYPE_GRAVITY, {{0, 9.8f}}},

};


const struct sensor_config *get_supported_sensor_cfg()
{
	return &supported_sensors[0];
}
const struct sensor_config *get_dummy_sensor_cfg(enum sensor_type s_type)
{
	unsigned int i = 0;
	for(; i < sizeof(dummy_sensors); i++)
	{
		if(dummy_sensors[i].type == s_type)
			return &dummy_sensors[i];
	}
	return NULL;
}


void set_dummy_sensor_name(enum sensor_type s_type, const char *name)
{
	const struct sensor_config *config = get_dummy_sensor_cfg(s_type);
	if(config)
	{
		char *name_stripped = const_cast<char *> (config->name);
		ALOGD("Coping dummy name %s\n",name);
		strncpy(name_stripped, name, 80);	
	}
}

void set_dummy_gsensor_cfg(const struct gsensor_config *gs_cfg)
{
	const struct sensor_config *config = get_dummy_sensor_cfg(AML_SENSOR_TYPE_GRAVITY);
	if(config)
	{
		struct gsensor_config *cfg = const_cast<struct gsensor_config *> (&config->config.gs_config);
		ALOGD("Configuring dummy gsensor\n");
		*cfg = *gs_cfg;
	}
}
