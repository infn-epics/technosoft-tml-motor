#
# Copyright (c) 2023 Cosylab d.d.
# Developed IP
# This software is distributed under the terms found
# in file LICENSE.txt that is included with this distribution.
#

from org.csstudio.display.builder.runtime.script import PVUtil
from java import lang

pvStr = PVUtil.getString(pvs[0])
macros = widget.getPropertyValue('macros')

if pvStr == "Raw":
    macros.add("TIFF", "RawTiff")
    macros.add("IMG", "Raw")
else:
    macros.add("TIFF", "RoiTiff")
    macros.add("IMG", "ROI")

widget.setPropertyValue("file", "")
widget.setPropertyValue("file", "savefile.bob")

