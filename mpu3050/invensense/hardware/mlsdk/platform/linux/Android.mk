# Copyright 2011 @amlogic
LOCAL_PATH:= $(call my-dir)

#################################################################
# libubi shared library
#
include $(CLEAR_VARS)

LOCAL_PRELINK_MODULE := false

LOCAL_SRC_FILES := \
	int_linux.c \
	log_linux.c \
	log_printf_linux.c \
	mlos_linux.c \
	mlsl_linux.c

LOCAL_C_INCLUDES += \
	$(LOCAL_PATH)/../include/ \
	$(LOCAL_PATH)/../../mllite/
	
LOCAL_CFLAGS += -DLINUX
LOCAL_CFLAGS += -Wall

LOCAL_STATIC_LIBRARIES += libcutils libc

LOCAL_MODULE := libmlplatform


include $(BUILD_SHARED_LIBRARY)

