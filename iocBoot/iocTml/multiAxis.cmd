#
# multiAxis.cmd — Example: multiple Technosoft axes on RS-485/CAN bus
#

dbLoadTemplate("multiAxis.substitutions")

## Create the controller for 3 axes on RS-232/RS-485
TmlControllerConfig("TML-MULTI", "/dev/ttyUSB0", 3, 3, 9600, 0.1, 1.0)

## Configure each axis individually
TmlAxisConfig("TML-MULTI", 0, 10, "$(TOP)/tml_lib/config/star_vat_phs.t.zip", "LSN")
TmlAxisConfig("TML-MULTI", 1, 11, "$(TOP)/tml_lib/config/star_vat_phs.t.zip", "LSN")
TmlAxisConfig("TML-MULTI", 2, 12, "$(TOP)/tml_lib/config/star_vat_phs.t.zip", "LSP")
