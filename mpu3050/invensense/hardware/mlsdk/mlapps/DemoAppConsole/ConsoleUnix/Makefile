# Use bash for additional echo fancyness
SHELL = /bin/bash

EXEC = consoledmp-static$(EXEC_SUFFIX)

TARGET = android

MLSDK_ROOT = ../../..
ML_COMMON = ../../common
ML_LITE_DIR = $(MLSDK_ROOT)/mllite
ML_FULL_DIR = $(MLSDK_ROOT)/mldmp
ML_PLATFORM_DIR = $(MLSDK_ROOT)/platform

OBJFOLDER=$(CURDIR)/obj

CFLAGS =  -static -Wall -fpic
CFLAGS += -D_REENTRANT -DLINUX 
CFLAGS += -DBB_DEMO -DBB_RS232 -DBB_I2C
CFLAGS += -I$(ML_LITE_DIR) -I$(ML_FULL_DIR)
CFLAGS += -I$(ML_PLATFORM_DIR)/include -I$(ML_PLATFORM_DIR)/linux
CFLAGS += -I$(ML_COMMON) -I$(MLSDK_ROOT)/mltools/debugsupport	

CROSS=arm-none-linux-gnueabi-
COMP=$(CROSS)gcc

VPATH += $(ML_COMMON) $(ML_LITE_DIR)

####################################################################################################
## sources

MK_NAME = Makefile

ML_LIBS = \
	$(ML_FULL_DIR)/mpl/$(TARGET)/libmpl.a \
	$(ML_LITE_DIR)/mpl/$(TARGET)/libmllite.a \
	$(ML_PLATFORM_DIR)/linux/libmlplatform.a \

ML_SRCS = \
	consoledmp.c \
    $(ML_COMMON)/mlsetup.c \
    $(ML_COMMON)/helper.c \
    $(ML_COMMON)/gestureMenu.c \
	$(ML_LITE_DIR)/mlmath.c

ML_OBJS = $(addsuffix .o, $(ML_SRCS))
ML_OBJS_DST = $(addprefix $(OBJFOLDER)/,$(notdir $(ML_OBJS)))

####################################################################################################
## macros

define echo_in_colors
@echo -ne "\e[1;32m"$(1)"\e[0m"
endef 

####################################################################################################
## rules

.PHONY: all clean cleanall install

all: $(EXEC) $(MK_NAME)

$(EXEC) : $(OBJFOLDER) $(ML_OBJS_DST) $(ML_LIBS) $(MK_NAME)
	@$(call echo_in_colors, "\n<linking $(EXEC) with objects $(ML_OBJS_DST) and libraries $(ML_LIBS)\n")
	$(COMP) -static $(ML_OBJS_DST) $(ML_LIBS) -lm -lpthread -o $(EXEC) 

$(OBJFOLDER) : 
	@$(call echo_in_colors, "\n<creating object's folder 'obj/'>\n")
	mkdir obj

$(ML_OBJS_DST) : $(OBJFOLDER)/%.c.o : %.c $(MK_NAME)
	@$(call echo_in_colors, "\n<compile $< to $(OBJFOLDER)/$(notdir $@)>\n")
	$(COMP) $(CFLAGS) -o $@ -c $<

clean : 
	rm -fR $(OBJFOLDER)

cleanall : 
	rm -fR $(EXEC) $(OBJFOLDER)

install : $(EXEC)
	cp -f $(EXEC) $(INSTALL_DIR)
