TOP = ..
include $(TOP)/configure/CONFIG

ioc_name := $(shell basename $(realpath ${TOP}))
# Get IOC parent directory
this_path := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))../../)

SCRIPTS_FOLDER = ../scripts
IOC_NAME ?= ${ioc_name}
INSTALL_PREFIX ?= ${this_path}
BPM_USER ?= bpm-epics-ioc
export IOC_NAME
export INSTALL_PREFIX
export BPM_USER

install: user_install scripts_install
	echo "INSTALL_PREFIX "${INSTALL_PREFIX}

uninstall: scripts_uninstall

scripts_install:
	$(MAKE) -C ${SCRIPTS_FOLDER} install

scripts_uninstall:
	$(MAKE) -C ${SCRIPTS_FOLDER} uninstall

user_install:
	@echo "Creating user/group $(BPM_USER)"
	-$(Q)useradd --system --no-create-home $(BPM_USER)

realuninstall: uninstall

clean:

realclean: clean

distclean: realclean realuninstall

mrproper: clean

