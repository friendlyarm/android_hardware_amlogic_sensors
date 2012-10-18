SHELL=/bin/bash

# Modify the following variables according to your product and source location.
PRODUCT      = g24ref
ANDROID_ROOT = /home/amlogic/work/jellybean
KERNEL_ROOT  = /home/amlogic/work/jellybean/common

#For android versions of Jellybean and later, use androideabi
CROSS		 = $(ANDROID_ROOT)/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.6/bin/arm-linux-androideabi-
#For android versions earlier than Jellybean, use arm-eabi
#CROSS		 = $(ANDROID_ROOT)/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-
 

# Verbose compile info for android target
VERBOSE		 = 1
TARGET 	     = android

ifeq ($(VERBOSE),1)
	DUMP=2>/dev/stderr
else
	DUMP=2>/dev/null
endif

include Android-common.mk

#############################################################################
## targets

LIB_FOLDERS = \
	platform/linux \
	mllite/mpl/$(TARGET) \
	mldmp/mpl/$(TARGET)

APP_FOLDERS = \
	mlapps/DemoAppConsole/ConsoleUnix \
	mlapps/DemoAppPedometer/PedometerStandAloneUnix \

#	mlapps/DemoAppEis/EisUnix

ifneq ($(HARDWARE),M_HW)
	APP_FOLDERS += mltools/driver_selftest
endif

INSTALL_DIR = $(CURDIR)

####################################################################################################
## macros

ifndef echo_in_colors
define echo_in_colors
	echo -ne "\e[1;34m"$(1)"\e[0m"
endef	
endif

define maker_libs
	echo "MLPLATFORM_LIB_NAME = $(MLPLATFORM_LIB_NAME)"
	echo "MLLITE_LIB_NAME     = $(MLLITE_LIB_NAME)"
	echo "MPL_LIB_NAME        = $(MPL_LIB_NAME)"

	$(call echo_in_colors, "\n<making '$(1)' in folder 'platform/linux'>\n"); \
	make MLPLATFORM_LIB_NAME=$(MLPLATFORM_LIB_NAME) ANDROID_ROOT=$(ANDROID_ROOT) KERNEL_ROOT=$(KERNEL_ROOT) PRODUCT=$(PRODUCT) CROSS=$(CROSS) -C platform/linux -f Android-shared.mk $@ $(DUMP)

	$(call echo_in_colors, "\n<making '$(1)' in folder 'mllite/mpl/$(TARGET)'>\n"); \
	make MLLITE_LIB_NAME=$(MLLITE_LIB_NAME) ANDROID_ROOT=$(ANDROID_ROOT) KERNEL_ROOT=$(KERNEL_ROOT) PRODUCT=$(PRODUCT) CROSS=$(CROSS) -C mllite/mpl/$(TARGET) -f Android-shared.mk $@  $(DUMP)

	if test -f mldmp/mpl/$(TARGET)/Android-shared.mk; then \
		$(call echo_in_colors, "\n<making '$(1)' in folder 'mldmp/mpl/$(TARGET)'>\n"); \
	        make MPL_LIB_NAME=$(MPL_LIB_NAME) ANDROID_ROOT=$(ANDROID_ROOT) KERNEL_ROOT=$(KERNEL_ROOT) PRODUCT=$(PRODUCT) CROSS=$(CROSS) -C mldmp/mpl/$(TARGET) -f Android-shared.mk $@ $(DUMP); \
	fi
endef

define maker_apps
	for dir in $(APP_FOLDERS); do \
		$(call echo_in_colors, "\n<making '$(1)' in folder $$dir>\n"); \
		make -C $$dir TARGET=$(TARGET) -f Android-shared.mk $@ $(DUMP); \
	done
endef 

#############################################################################
## rules

.PHONY : all libs $(LIB_FOLDERS) $(APP_FOLDERS) clean cleanall install

#Make only libs by default
libs :  
	@$(call maker_libs,$@)

all :  
	@$(call maker_libs,$@)
	@$(call maker_apps,$@)

clean : 
	@$(call maker_libs,$@)
	@$(call maker_apps,$@)

cleanall : 
	@$(call maker_libs,$@)
	@$(call maker_apps,$@)

