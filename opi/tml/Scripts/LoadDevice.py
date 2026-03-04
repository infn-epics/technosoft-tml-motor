from org.csstudio.display.builder.runtime.script import ScriptUtil, PVUtil
import sys
import os

# Get the display model to construct the path to Scripts directory
display_model = widget.getDisplayModel()
display_path = display_model.getUserData(display_model.USER_DATA_INPUT_FILE)
scripts_dir = os.path.dirname(display_path) + "/../../../Scripts"
scripts_path = os.path.abspath(scripts_dir)
zone = PVUtil.getString(pvs[0])

# Add Scripts directory to Python path if not already there
if scripts_path not in sys.path:
    sys.path.insert(0, scripts_path)

import epik8sutil

logger = ScriptUtil.getLogger()
# Load devices using epik8sutil.conf_to_dev
devarray = epik8sutil.conf_to_dev(widget)

if not devarray:
    group = widget.getEffectiveMacros().getValue("GROUP")
    if group is None:
        group = "mot"
    ScriptUtil.showMessageDialog(widget, "No devices found for group: " + group)
else:
    # Find the combo box widget
    combo = ScriptUtil.findWidgetByName(widget, "DeviceCombo")
    combozone = ScriptUtil.findWidgetByName(widget, "DeviceZone")

    if combo is None:
        ScriptUtil.showMessageDialog(widget, "Cannot find DeviceCombo widget")
    else:
        # Build device list from devarray
        device_list = []
        print(widget.getName() + "] FOUND " + str(len(devarray)) + " devices in configuration file")

        for dev in devarray:
            # Use the device name
            
            devtype=dev.get('TYPE','')

            if devtype != 'tml':
                print(widget.getName() + "] SKIPPING " +dev.get('name', '') + " type: " + devtype )
                continue
            zoneadd = dev.get('ZONE', '')

            if 'zones_set' not in locals():
                zones_set = set()
            if zone:
                zones_set.add(zoneadd)

            if zone != "ALL" and zone not in (dev.get('ZONE','')):
                continue

            device_name = dev.get('NAME', '')
            deviceprefix = dev.get('P', '') + ":" + device_name
            
            if device_name:
                logger.info("Loading device: " + device_name)
                device_list.append(deviceprefix)
            
        # Populate the combo box
        combo.setItems(device_list)
        combozone.setItems(list(zones_set))
        
        # Set the first device as default if list is not empty
        if len(device_list) > 0:
            ScriptUtil.getPrimaryPV(combo).write(device_list[0])
            logger.info("Loaded " + str(len(device_list)) + " devices into combo")


