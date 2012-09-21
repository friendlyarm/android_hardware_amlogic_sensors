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

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>

#include <cutils/log.h>

#include <linux/input.h>

#include "SensorBase.h"
#include "SensorConfigure.h"
/*****************************************************************************/


#ifndef ALOGD
#define ALOGD	LOGD
#define ALOGE	LOGE
#define ALOGV	LOGV
#define ALOGE_IF	LOGE_IF
#define ALOGD_IF	LOGD_IF
#define ALOGV_IF	LOGV_IF
#endif

SensorBase::SensorBase(
        const char* dev_name,
        const char* data_name)
    : dev_name(dev_name), data_name(data_name),
      dev_fd(-1), data_fd(-1)
{
    if (data_name) {
        data_fd = openInput(data_name);
    }
}

SensorBase::SensorBase(
        const char* dev_name,
	enum sensor_type s_type)
    : dev_name(dev_name),
      dev_fd(-1), data_fd(-1), sensor_cfg(NULL), using_dummy(0)
{
	printf("Aml sensor hal, about to probeInput\n");
	data_fd = probeInput(s_type);	
}

SensorBase::~SensorBase() {
    if (data_fd >= 0) {
        close(data_fd);
    }
    if (dev_fd >= 0) {
        close(dev_fd);
    }
}

int SensorBase::open_device() {
    if (dev_fd<0 && dev_name) {
        dev_fd = open(dev_name, O_RDONLY);
        ALOGE_IF(dev_fd<0, "Couldn't open %s (%s)", dev_name, strerror(errno));
    }
    return 0;
}

int SensorBase::close_device() {
    if (dev_fd >= 0) {
        close(dev_fd);
        dev_fd = -1;
    }
    return 0;
}

int SensorBase::getFd() const {

	if(data_fd>=0)
		return data_fd;
	return dev_fd;
}

int SensorBase::setDelay(int32_t handle, int64_t ns) {
    return 0;
}

bool SensorBase::hasPendingEvents() const {
    return false;
}

int64_t SensorBase::getTimestamp() {
    struct timespec t;
    t.tv_sec = t.tv_nsec = 0;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return int64_t(t.tv_sec)*1000000000LL + t.tv_nsec;
}

int SensorBase::openInput(const char* inputName) {
    int fd = -1;
    const char *dirname = "/dev/input";
    char devname[PATH_MAX];
    char *filename;
    DIR *dir;
    struct dirent *de;
    dir = opendir(dirname);
    if(dir == NULL)
        return -1;
    strcpy(devname, dirname);
    filename = devname + strlen(devname);
    *filename++ = '/';
    while((de = readdir(dir))) {
        if(de->d_name[0] == '.' &&
                (de->d_name[1] == '\0' ||
                        (de->d_name[1] == '.' && de->d_name[2] == '\0')))
            continue;
        strcpy(filename, de->d_name);
        fd = open(devname, O_RDONLY);
        if (fd>=0) {
            char name[80];
            if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), &name) < 1) {
                name[0] = '\0';
            }
            if (!strcmp(name, inputName)) {
                strcpy(input_name, filename);
                ALOGD("openInput input_name:%s", input_name);
                break;
            } else {
                close(fd);
                fd = -1;
            }
        }
    }
    closedir(dir);
    if(fd<0)
        ALOGE("couldn't find '%s' input device", inputName);
    return fd;
}

int SensorBase::probeInput(enum sensor_type s_type)
{
    int fd = -1;
    const char *dirname = "/dev/input";
    char devname[PATH_MAX];
    char *filename;
    DIR *dir;
    struct dirent *de;

	
	int found_match = 0;
	int dummy_fd = -1;
	int found_dummy = 0;


    dir = opendir(dirname);
    if(dir == NULL)
        return -1;
    strcpy(devname, dirname);
    filename = devname + strlen(devname);
    *filename++ = '/';
    while((de = readdir(dir))) {
        if(de->d_name[0] == '.' &&
                (de->d_name[1] == '\0' ||
                        (de->d_name[1] == '.' && de->d_name[2] == '\0')))
            continue;
        strcpy(filename, de->d_name);
        fd = open(devname, O_RDONLY);
        if (fd>=0) {
		char name[80];
		const struct sensor_config *pconfig = get_supported_sensor_cfg();
		if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), &name) < 1) {
			name[0] = '\0';
		}
		ALOGD("Aml sensor hal, probing input : %s\n", name);
		while(pconfig->type != AML_SENSOR_TYPE_NONE)
		{
		    if ((s_type == pconfig->type) && (!strcmp(name, pconfig->name))) {
			strcpy(input_name, filename);
			found_match = 1;
			ALOGD("openInput input_name:%s", name);
			break;
		    }
		    pconfig++;
		}

		if(found_match)
		{
			sensor_cfg = pconfig;
			break;
		}
		else
		{
			unsigned long bits;
			if (ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(bits)), &bits) <= 0) 
			{
				ALOGE("Get ABS event bit failed. \n");
			}
			else
			{
				int type_match = 0;
				switch(s_type)
				{
					case AML_SENSOR_TYPE_GRAVITY :
					if(bits & (1UL<<ABS_X) && bits & (1UL<<ABS_Y) && (1UL<<ABS_Z))
						type_match = 1;
					break;

					case AML_SENSOR_TYPE_LIGHT :
					break;
					case AML_SENSOR_TYPE_COMPASS:
					break;
					default :
					break;
				}
				if(type_match)
				{
					found_dummy++;
					if(found_dummy == 1)
					{//Found a dummy sensor. Keep the file open.
					ALOGD("Found a dummy sensor %s\n", name);
						dummy_fd = fd;
						set_dummy_sensor_name(s_type, name);
						fd = -1;
					}
					else
					{//Found more than one dummy sensor, close them all.
						//close previously founded dummy sensor fd.
						close(dummy_fd);
						dummy_fd = -1;
						//close current fd
						close(fd);
						fd = -1;
					}
				}
				else
				{
					close(fd);
					fd = -1;	
				}
			}//if(ioctl)

		}//if(found_match)
	}//if(fd>=0)
    }//while

    if(!found_match && found_dummy==1)
    {//No matched sensor, but found one and only one dummy sensor.
	fd = dummy_fd;
	sensor_cfg = get_dummy_sensor_cfg(s_type);
	using_dummy = 1;
	ALOGD("Found no match, reckon %s as sensor and use dummy configuration\n", sensor_cfg->name);
    }
		    
    closedir(dir);
    if(fd<0)
        ALOGE("Sensor HAL failed to find a supported device");
    return fd;

}
