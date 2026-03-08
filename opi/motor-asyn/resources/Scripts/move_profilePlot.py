#
# Copyright (c) 2023 Cosylab d.d.
# Developed IP
# This software is distributed under the terms found
# in file LICENSE.txt that is included with this distribution.
#

from org.csstudio.display.builder.runtime.script import PVUtil, ConsoleUtil
from java import lang

image_width = PVUtil.getInt(pvs[0])

macros = widget.getEffectiveMacros()
init_width = int(macros.getValue('IMGWIDTH'))

new_x = 400
if image_width > 400:
    new_x = image_width
elif image_width == 0:
    new_x = init_width

new_x = new_x + 20

widget.setPropertyValue('x', new_x) 

