importPackage(Packages.org.csstudio.opibuilder.scriptUtil); 
var pv0 = PVUtil.getDouble(pvs[0]);
if(pv0 == 0)
	widget.setPropertyValue("axis_1_auto_scale",false);
else if(pv0 == 1)
	widget.setPropertyValue("axis_1_auto_scale",true);
else
	widget.setPropertyValue("axis_1_auto_scale",false);
