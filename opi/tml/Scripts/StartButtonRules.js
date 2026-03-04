/* Script for calculating the "START" button's Enabled property.
   It should only attached only to START button. */

importPackage(Packages.org.csstudio.opibuilder.scriptUtil);
importPackage(Packages.org.eclipse.jface.dialogs);
logger = ScriptUtil.getLogger()

/* Calculate if START button should be enabled or disabled. */
function calc_enabled_property() {
	var action = PVUtil.getString(pvs[0]); /* Motor action */
	var desired_pos = PVUtil.getDouble(pvs[1]); /* Desired position */
	var tml_sub_name = PVUtil.getString(pvs[2]); /* TML subroutine name */
	var current_abs_pos = PVUtil.getDouble(pvs[3]); /* Current absolute position */
	var axis_state = PVUtil.getString(pvs[4]); /* Axis status (ndsChannelStatus) */
	var axis_busy = PVUtil.getLong(pvs[5]); /* Axis busy performing some motion */
	
	logger.info("Enable properties: " +action + " pos: "+ desired_pos + " sub name:"+tml_sub_name+ " current abs pos: "+current_abs_pos+ " axis state: "+axis_state + " busy:"+axis_busy)

	/* Return if motor is not in processing (state machine) state. Don't want
	   to show button if it's in disabled, error, or any other state. */
	if (axis_state != "PROCESSING") {
		return false;
	}
	
	/* Return if motor is busy performing some motion */
	if (axis_busy == 1) {
		return false;
	}
	
	/* START button will be disabled by default */
	var enable = false;
	
	/* Note: I tried putting these conditions in a switch statement, but it
	   always executed the default clause and I'm not sure why. If-else-if
	   seems to work without any problems.
	
	/* For `Move Relative`, display START button if desired position
	   is not zero. */
	if (action == "MOVE REL") {
		if (desired_pos != 0.0) {
			enable = true;
		}
		
	/* When `Move Abstolute` is set, display START button only
	   if desired position is different from current position. */
	} else if (action == "MOVE ABS") {
		/* Convert both numbers to string representations
		   with same decimal precision,  */
		if (desired_pos.toFixed(10) != current_abs_pos.toFixed(10)) {
			enable = true;
		}
		
	/* No extra parameters need to be set for `Home` action,
	   START button can always be displayed. */
	} else if (action == "HOME") {
		enable = true;
	
	/* No extra parameters need to be set for `Jog Forward` action,
	   START button can always be displayed. */	
	} else if (action == "JOG FORWARD") {
		enable = true;
		
	/* No extra parameters need to be set for `Jog Reverse` action,
	   START button can always be displayed. */
	} else if (action == "JOG REVERSE") {
		enable = true;
	
	/* For `TML Subroutine` action, display START button only if a
	   TML subroutine name is present. */
	} else if (action == "TML SUB") {
		if (tml_sub_name.trim() != "") {
			enable = true;
		}
		
	/* Nothing to display for `NONE` action. */
	} else if (action == "NONE") {
	
	/* Just-in-case. */
	} else {
		MessageDialog.openWarning(null, "Unknown action", "Unknown motor action: '" + action + "'");
	}
	
	return enable;
}

/* Should button be enabled? */
var enable = calc_enabled_property();

/* Enable or disable START button */
widget.setPropertyValue("enabled", enable);