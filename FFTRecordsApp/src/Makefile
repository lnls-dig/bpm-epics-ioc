TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += FFTRecords

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
DBDINC += RFFTRecord
DBDINC += CFFTRecord
DBDINC += AveDevRecord

# install FFTRecords.dbd into <top>/dbd
DBD += FFTRecords.dbd

# specify all source files to be compiled and added to the library
#FFTRecords_SRCS += xxx
FFTRecords_SRCS += RFFTRecord.c
FFTRecords_SRCS += CFFTRecord.c
FFTRecords_SRCS += fftpack.c

FFTRecords_SRCS += AveDevRecord.c

FFTRecords_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

