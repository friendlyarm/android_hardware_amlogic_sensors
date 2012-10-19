/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ANDROID_SENSOR_BASE_H
#define ANDROID_SENSOR_BASE_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include "SensorConfigure.h"
/*****************************************************************************/

struct sensors_event_t;

class SensorBase {
protected:
    const char* dev_name;
    const char* data_name;
    char        input_name[PATH_MAX];
    char        class_path[PATH_MAX];
    int         dev_fd;
    int         data_fd;

	const struct sensor_config *sensor_cfg;
	int using_dummy;

    int openInput(const char* inputName);
    int probeInput();
    static int64_t getTimestamp();


    static int64_t timevalToNano(timeval const& t) {
        return t.tv_sec*1000000000LL + t.tv_usec*1000;
    }

    int open_device();
    int close_device();

	/* ALl sensors must provide a method to prode its type */
	virtual bool probeSensorType(int fd) = 0;
	virtual enum sensor_type getSensorType() = 0;
	virtual void setDummySensorName(const char *name) = 0;
	virtual const struct sensor_config *getDummySensorConfig()= 0;
public:
            SensorBase(
                    const char* dev_name,
                    const char* data_name);

	SensorBase(const char* dev_name);

	static int getClassPath(char *des, const char *name);

	int initialize();
    virtual ~SensorBase();

    virtual int readEvents(sensors_event_t* data, int count) = 0;
    virtual bool hasPendingEvents() const;
    virtual int getFd();
    virtual int getPollTime() { return -1; }
    virtual int setDelay(int32_t handle, int64_t ns);
    virtual int enable(int32_t handle, int enabled) = 0;
};

/*****************************************************************************/

#endif  // ANDROID_SENSOR_BASE_H
