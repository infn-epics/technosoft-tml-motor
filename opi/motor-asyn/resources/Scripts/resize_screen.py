#
# Copyright (c) 2023 Cosylab d.d.
# Developed IP
# This software is distributed under the terms found
# in file LICENSE.txt that is included with this distribution.
#

from org.csstudio.display.builder.runtime.script import PVUtil, ConsoleUtil
from java import lang

x = PVUtil.getInt(pvs[0])
y = PVUtil.getInt(pvs[1])

macros = widget.getEffectiveMacros()
init_width = int(macros.getValue('IMGWIDTH'))
init_height = int(macros.getValue('IMGHEIGHT'))

if((float(x) / y) >= (float(init_width) / init_height)):
    new_height = float(y) / x * init_width
    new_width = init_width
else:
    new_width = float(x) / y * init_height
    new_height = init_height

pvs[2].setValue(new_width)

widget.setPropertyValue('width', new_width)
widget.setPropertyValue('height', new_height)

# The following is needed for the ROI (The coordinate system must be in pixels)
widget.setPropertyValue('x_axis.maximum', x)
widget.setPropertyValue('y_axis.minimum', y)

