TOP = ..
include $(TOP)/configure/CONFIG

all:

install:
	# Install integration service files and environment files
	cp --preserve=mode ../cfg/bpm-sw-be-ioc@.service /etc/systemd/system
	cp --preserve=mode ../cfg/bpm-sw-fe-ioc@.service /etc/systemd/system
	cp --preserve=mode ../cfg/bpm-sw-ioc@.target /etc/systemd/system
	cp --preserve=mode ../cfg/bpm-epics-ioc /etc/sysconfig

uninstall:
	# Remove systemd files
	rm -f /etc/systemd/system/bpm-sw-be-ioc@.service
	rm -f /etc/systemd/system/bpm-sw-fe-ioc@.service
	rm -f /etc/systemd/system/bpm-sw-ioc@.service
	rm -f /etc/sysconfig/bpm-epics-ioc

clean:

mrproper: clean

