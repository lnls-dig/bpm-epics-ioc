TOP = ..
include $(TOP)/configure/CONFIG

all:

install:
	$(TOP)/install/install.sh /opt/epics/startup/ioc/bpm-epics-ioc $(TOP)

uninstall:
	$(TOP)/install/uninstall.sh /opt/epics/startup/ioc/bpm-epics-ioc $(TOP)

realuninstall: uninstall

clean:

realclean: clean

distclean: realclean realuninstall

mrproper: clean

