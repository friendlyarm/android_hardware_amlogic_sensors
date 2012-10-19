# Copyright (C) 2008 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


ifeq ($(BOARD_USES_AML_SENSOR_HAL),true)

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)


# HAL module implemenation, not prelinked, and stored in
# hw/<SENSORS_HARDWARE_MODULE_ID>.<ro.product.board>.so

#ifeq ($(BOARD_USES_LIGHT_SENSOR),true)
ifeq ($(BOARD_USES_LIGHT_SENSOR),true)
LOCAL_CFLAGS += -DENABLE_LIGHT_SENSOR 
endif #BOARD_USES_LIGHT_SENSOR

LOCAL_MODULE := sensors.amlogic

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

LOCAL_MODULE_TAGS := optional

LOCAL_SRC_FILES := 						\
				GSensor.cpp 			\
				sensors_aml.cpp          \
				SensorBase.cpp           \
				InputEventReader.cpp   \
				LightSensorAML.cpp	\
				SensorConfigure.cpp
				
LOCAL_SHARED_LIBRARIES := liblog libcutils
LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)

endif # !TARGET_SIMULATOR
