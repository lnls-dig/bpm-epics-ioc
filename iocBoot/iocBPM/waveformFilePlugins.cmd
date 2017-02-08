##################### ADC File Saving ##########################

NDFileNetCDFConfigure("ADC_ABCD_FileNetCDF", $(QSIZE), 0, "$(PORT)", 4)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADC_ABCD_NETCDF,PORT=ADC_ABCD_FileNetCDF,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=4")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=${P},R=${R}ADC_ABCD_NETCDF,PORT=ADC_ABCD_FileNetCDF,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileNetCDF.template","P=${P},R=${R}ADC_ABCD_NETCDF,PORT=ADC_ABCD_FileNetCDF,ADDR=0,TIMEOUT=1")

NDFileHDF5Configure("ADC_ABCD_FileHDF5", $(QSIZE), 0, "$(PORT)", 4)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADC_ABCD_HDF5,PORT=ADC_ABCD_FileHDF5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=4")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=${P},R=${R}ADC_ABCD_HDF5,PORT=ADC_ABCD_FileHDF5,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileHDF5.template",  "P=${P},R=${R}ADC_ABCD_HDF5,PORT=ADC_ABCD_FileHDF5,ADDR=0,TIMEOUT=1")

##################### ADC SWAP File Saving ##########################

NDFileNetCDFConfigure("ADCSWAP_ABCD_FileNetCDF", $(QSIZE), 0, "$(PORT)", 10)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADCSWAP_ABCD_NETCDF,PORT=ADCSWAP_ABCD_FileNetCDF,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=10")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=${P},R=${R}ADCSWAP_ABCD_NETCDF,PORT=ADCSWAP_ABCD_FileNetCDF,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileNetCDF.template","P=${P},R=${R}ADCSWAP_ABCD_NETCDF,PORT=ADCSWAP_ABCD_FileNetCDF,ADDR=0,TIMEOUT=1")

NDFileHDF5Configure("ADCSWAP_ABCD_FileHDF5", $(QSIZE), 0, "$(PORT)", 10)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADCSWAP_ABCD_HDF5,PORT=ADCSWAP_ABCD_FileHDF5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=10")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=${P},R=${R}ADCSWAP_ABCD_HDF5,PORT=ADCSWAP_ABCD_FileHDF5,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileHDF5.template",  "P=${P},R=${R}ADCSWAP_ABCD_HDF5,PORT=ADCSWAP_ABCD_FileHDF5,ADDR=0,TIMEOUT=1")

##################### TBT AMP File Saving ##########################

NDFileNetCDFConfigure("TBT_ABCD_FileNetCDF", $(QSIZE), 0, "$(PORT)", 16)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_ABCD_NETCDF,PORT=TBT_ABCD_FileNetCDF,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=16")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=${P},R=${R}TBT_ABCD_NETCDF,PORT=TBT_ABCD_FileNetCDF,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileNetCDF.template","P=${P},R=${R}TBT_ABCD_NETCDF,PORT=TBT_ABCD_FileNetCDF,ADDR=0,TIMEOUT=1")

NDFileHDF5Configure("TBT_ABCD_FileHDF5", $(QSIZE), 0, "$(PORT)", 16)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_ABCD_HDF5,PORT=TBT_ABCD_FileHDF5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=16")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=${P},R=${R}TBT_ABCD_HDF5,PORT=TBT_ABCD_FileHDF5,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileHDF5.template",  "P=${P},R=${R}TBT_ABCD_HDF5,PORT=TBT_ABCD_FileHDF5,ADDR=0,TIMEOUT=1")

##################### TBT POS File Saving ##########################

NDFileNetCDFConfigure("TBT_XYQS_FileNetCDF", $(QSIZE), 0, "$(PORT)", 22)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_XYQS_NETCDF,PORT=TBT_XYQS_FileNetCDF,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=22")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=${P},R=${R}TBT_XYQS_NETCDF,PORT=TBT_XYQS_FileNetCDF,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileNetCDF.template","P=${P},R=${R}TBT_XYQS_NETCDF,PORT=TBT_XYQS_FileNetCDF,ADDR=0,TIMEOUT=1")

NDFileHDF5Configure("TBT_XYQS_FileHDF5", $(QSIZE), 0, "$(PORT)", 22)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_XYQS_HDF5,PORT=TBT_XYQS_FileHDF5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=22")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=${P},R=${R}TBT_XYQS_HDF5,PORT=TBT_XYQS_FileHDF5,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileHDF5.template",  "P=${P},R=${R}TBT_XYQS_HDF5,PORT=TBT_XYQS_FileHDF5,ADDR=0,TIMEOUT=1")

##################### FOFB AMP File Saving ##########################

NDFileNetCDFConfigure("FOFB_ABCD_FileNetCDF", $(QSIZE), 0, "$(PORT)", 34)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_ABCD_NETCDF,PORT=FOFB_ABCD_FileNetCDF,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=34")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=${P},R=${R}FOFB_ABCD_NETCDF,PORT=FOFB_ABCD_FileNetCDF,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileNetCDF.template","P=${P},R=${R}FOFB_ABCD_NETCDF,PORT=FOFB_ABCD_FileNetCDF,ADDR=0,TIMEOUT=1")

NDFileHDF5Configure("FOFB_ABCD_FileHDF5", $(QSIZE), 0, "$(PORT)", 34)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_ABCD_HDF5,PORT=FOFB_ABCD_FileHDF5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=34")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=${P},R=${R}FOFB_ABCD_HDF5,PORT=FOFB_ABCD_FileHDF5,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileHDF5.template",  "P=${P},R=${R}FOFB_ABCD_HDF5,PORT=FOFB_ABCD_FileHDF5,ADDR=0,TIMEOUT=1")

##################### FOFB POS File Saving ##########################

NDFileNetCDFConfigure("FOFB_XYQS_FileNetCDF", $(QSIZE), 0, "$(PORT)", 40)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_XYQS_NETCDF,PORT=FOFB_XYQS_FileNetCDF,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=40")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=${P},R=${R}FOFB_XYQS_NETCDF,PORT=FOFB_XYQS_FileNetCDF,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileNetCDF.template","P=${P},R=${R}FOFB_XYQS_NETCDF,PORT=FOFB_XYQS_FileNetCDF,ADDR=0,TIMEOUT=1")

NDFileHDF5Configure("FOFB_XYQS_FileHDF5", $(QSIZE), 0, "$(PORT)", 40)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_XYQS_HDF5,PORT=FOFB_XYQS_FileHDF5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=40")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=${P},R=${R}FOFB_XYQS_HDF5,PORT=FOFB_XYQS_FileHDF5,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileHDF5.template",  "P=${P},R=${R}FOFB_XYQS_HDF5,PORT=FOFB_XYQS_FileHDF5,ADDR=0,TIMEOUT=1")

