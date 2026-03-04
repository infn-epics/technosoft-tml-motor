from org.csstudio.display.builder.runtime.script import PVUtil
from org.csstudio.display.builder.runtime.script import ScriptUtil

logger = ScriptUtil.getLogger()
pv = PVUtil.getString(pvs[0])
#device_prefix = widget.getEffectiveMacros().getValue("P")
logger.info("SELECTION:"+pv)

widget.setPropertyValue("file","")
widget.getPropertyValue("macros").add("DEVICE",pv)
widget.setPropertyValue("file","TML_Main.bob")
# if device_prefix and pv:
# 	newdev=device_prefix+pv
# 	logger.info("PV Name " + str(pvs[0]) + " Val:"+pv +" DEVICE:"+newdev)
#  	widget.setPropertyValue("file","")
# 	widget.getPropertyValue("macros").add("DEVICE", newdev)
# 	widget.setPropertyValue("file","TML_Main.bob")


