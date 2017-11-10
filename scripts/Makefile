TOP = ..
include $(TOP)/configure/CONFIG

CMDSEP = ;

SCRIPTS_PREFIX ?=
PWD = $(shell pwd)

# OS information
SERVICE_NAME = bpm-epics-ioc
INIT_SYSTEM := $(shell ./get-init-system.sh)
SCRIPTS_FULL_NAME := $(shell cd $(INIT_SYSTEM) && find . -type f)

# Strip off a leading ./
SCRIPTS=$(SCRIPTS_FULL_NAME:./%=%)

.PHONY: all clean mrproper install uninstall

all:

install:
	$(foreach script,$(SCRIPTS),mkdir -p $(dir ${SCRIPTS_PREFIX}/$(script)) $(CMDSEP))
	$(foreach script,$(SCRIPTS),cp --preserve=mode $(INIT_SYSTEM)/$(script) ${SCRIPTS_PREFIX}/$(script) $(CMDSEP))

uninstall:
	$(foreach script,$(SCRIPTS),rm -f ${SCRITS_PREFIX}/$(script) $(CMDSEP))

realuninstall: uninstall

clean:

realclean: clean

distclean: realclean realuninstall

mrproper: clean

