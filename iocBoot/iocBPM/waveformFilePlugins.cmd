##################### GEN AMP File Saving ##########################

NDFileNetCDFConfigure("GEN_ABCD_FileNetCDF", $(QSIZE), 0, "$(PORT)", 4)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDFileNetCDF.template","P=${P},R=${R}GEN_ABCD_NETCDF,PORT=GEN_ABCD_FileNetCDF,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

NDFileHDF5Configure("GEN_ABCD_FileHDF5", $(QSIZE), 0, "$(PORT)", 4)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDFileHDF5.template",  "P=${P},R=${R}GEN_ABCD_HDF5,PORT=GEN_ABCD_FileHDF5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

##################### GEN POS File Saving ##########################

NDFileNetCDFConfigure("GEN_XYQS_FileNetCDF", $(QSIZE), 0, "$(PORT)", 10)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDFileNetCDF.template","P=${P},R=${R}GEN_XYQS_NETCDF,PORT=GEN_XYQS_FileNetCDF,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

NDFileHDF5Configure("GEN_XYQS_FileHDF5", $(QSIZE), 0, "$(PORT)", 10)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDFileHDF5.template",  "P=${P},R=${R}GEN_XYQS_HDF5,PORT=GEN_XYQS_FileHDF5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

##################### PM Amp File Saving ##########################

NDFileNetCDFConfigure("PM_ABCD_FileNetCDF", $(QSIZE), 0, "$(PORT)", 16)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDFileNetCDF.template","P=${P},R=${R}PM_ABCD_NETCDF,PORT=PM_ABCD_FileNetCDF,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

NDFileHDF5Configure("PM_ABCD_FileHDF5", $(QSIZE), 0, "$(PORT)", 16)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDFileHDF5.template",  "P=${P},R=${R}PM_ABCD_HDF5,PORT=PM_ABCD_FileHDF5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

##################### PM POS File Saving ##########################

NDFileNetCDFConfigure("PM_XYQS_FileNetCDF", $(QSIZE), 0, "$(PORT)", 22)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDFileNetCDF.template","P=${P},R=${R}PM_XYQS_NETCDF,PORT=PM_XYQS_FileNetCDF,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

NDFileHDF5Configure("PM_XYQS_FileHDF5", $(QSIZE), 0, "$(PORT)", 22)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDFileHDF5.template",  "P=${P},R=${R}PM_XYQS_HDF5,PORT=PM_XYQS_FileHDF5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

##################### SP File Saving ##########################

NDFileNetCDFConfigure("SP_ABCD_FileNetCDF", $(QSIZE), 0, "$(PORT)", 28)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDFileNetCDF.template","P=${P},R=${R}SP_ABCD_NETCDF,PORT=SP_ABCD_FileNetCDF,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

NDFileHDF5Configure("SP_ABCD_FileHDF5", $(QSIZE), 0, "$(PORT)", 28)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDFileHDF5.template",  "P=${P},R=${R}SP_ABCD_HDF5,PORT=SP_ABCD_FileHDF5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")
