SHELL=/bin/bash

TARGET = android

ifeq ($(VERBOSE),1)
	DUMP=2>/dev/stderr
else
	DUMP=2>/dev/null
endif

#############################################################################
## targets

LIB_FOLDERS = \
	platform/linux \
	mllite/mpl/$(TARGET) \
	mldmp/mpl/$(TARGET)

APP_FOLDERS = \
	mlapps/DemoAppGlyph/GlyphConsoleUnix \
	mlapps/DemoAppConsole/ConsoleUnix \
	mlapps/DemoAppPoll/PollUnix \
	mlapps/DemoAppInt/IntUnix \
	mlapps/DemoAppPointer/PointerUnix \
	mltools/driver_selftest

INSTALL_DIR = $(CURDIR)/../mlsdk-rel/

####################################################################################################
## macros

define echo_in_colors
	echo -ne "\e[1;34m"$(1)"\e[0m"
endef 

define maker_libs
	$(call echo_in_colors, "\n<making '$(1)' in folder 'platform/linux'>\n"); \
	make -C platform/linux -f Makefile $@ $(DUMP)

	$(call echo_in_colors, "\n<making '$(1)' in folder 'mllite/mpl/$(TARGET)'>\n"); \
	make -C mllite/mpl/$(TARGET) -f static-mlmath.mk $@ $(DUMP)

	$(call echo_in_colors, "\n<making '$(1)' in folder 'mldmp/mpl/$(TARGET)'>\n"); \
    #make -C mldmp/mpl/(TARGET) -f static-mlmath.mk $@ $(DUMP)

endef


define maker_apps
	for dir in $(APP_FOLDERS); do \
		$(call echo_in_colors, "\n<making '$(1)' in folder $$dir>\n"); \
		make -C $$dir TARGET=$(TARGET) $@ $(DUMP); \
	done
endef 

#############################################################################
## rules

.PHONY : all $(LIB_FOLDERS) $(APP_FOLDERS) clean cleanall install

all : 
	@$(call maker_libs,$@)
	@$(call maker_apps,$@)

clean : 
	@$(call maker_libs,$@)
	@$(call maker_apps,$@)

cleanall : 
	@$(call maker_libs,$@)
	@$(call maker_apps,$@)

install:
	for dir in $(APP_FOLDERS); do \
		$(call echo_in_colors, "\n<making '$(1)' in folder $$dir>\n"); \
		make INSTALL_DIR=$(INSTALL_DIR) -C $$dir TARGET=$(TARGET) $@ $(DUMP); \
	done
