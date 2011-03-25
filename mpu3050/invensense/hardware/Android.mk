# invensense MPL daemon makefile
# Created on: 22 September 2010  Author: Kevin Powell

LOCAL_PATH:= $(call my-dir)

#
# mpld
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:=        \
 mpld.cpp      \
 Impld.cpp
   
LOCAL_C_INCLUDES += $(LOCAL_PATH)/mlsdk/platform/include
LOCAL_C_INCLUDES += $(LOCAL_PATH)/mlsdk/platform/linux
LOCAL_C_INCLUDES += $(LOCAL_PATH)/mlsdk/mllite
LOCAL_C_INCLUDES += $(LOCAL_PATH)/mlsdk/mldmp

LOCAL_PREBUILT_LIBS :=  libmpl.so libmllite.so libmlplatform.so
LOCAL_SHARED_LIBRARIES:= libcutils libutils libbinder libmpl libmllite libmlplatform
LOCAL_MODULE:= mpld
LOCAL_LDFLAGS += -g
LOCAL_CPPFLAGS+=-DLOG_TAG=\"mpld\"
LOCAL_CPPFLAGS+=-DLINUX=1
LOCAL_CPPFLAGS+=-g
LOCAL_CFLAGS+=-g
LOCAL_PRELINK_MODULE:=false
include $(BUILD_EXECUTABLE)
include $(BUILD_MULTI_PREBUILT)

#
# TestClient
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:=        \
 Impld.cpp      \
 TestClient.cpp \
   
LOCAL_SHARED_LIBRARIES:= libcutils libutils libbinder

LOCAL_MODULE:= TestClient

LOCAL_CFLAGS+=-DLOG_TAG=\"TestClient\"
LOCAL_PRELINK_MODULE:=false
include $(BUILD_EXECUTABLE)

#
# Sensors HAL
#
# HAL module implemenation, not prelinked, and stored in
# hw/<SENSORS_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)

ifneq ($(TARGET_SIMULATOR),true)

LOCAL_MODULE := sensors.amlogic

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

LOCAL_MODULE_TAGS := eng user debug
LOCAL_CFLAGS += -g
LOCAL_CPPFLAGS += -g
LOCAL_LDFLAGS += -g
ifeq ($(BOARD_USES_LIGHT_SENSOR),true)
LOCAL_SRC_FILES :=sensors2.cpp Impld.cpp
else
LOCAL_SRC_FILES :=sensors.cpp Impld.cpp
endif
LOCAL_SHARED_LIBRARIES := liblog libcutils libutils libbinder
LOCAL_PRELINK_MODULE := false
include $(BUILD_SHARED_LIBRARY)
endif # !TARGET_SIMULATOR

