# technosoft-asyn — Technosoft TML Motor Driver (Pure Asyn)

EPICS asynMotorController / asynMotorAxis driver for **Technosoft**
intelligent servo and stepper drives, using the vendor's TML_lib
high-level C library.

This is a complete rewrite of the original NDS-based `motorTechnosoft`
driver, replacing the NDS/Cosylab layer with the standard EPICS motor
record + asyn model-3 architecture.

## Features

- Standard EPICS `motor` record support through `asynMotorController`/`asynMotorAxis`
- Direct use of TML_lib API (`TS_OpenChannel`, `TS_MoveAbsolute`, `TS_ReadStatus`, etc.)
- Thread-safe access to the non-reentrant TML library via `epicsMutex`
- Absolute and relative positioning via motor record
- Velocity (jog) mode
- Homing to positive or negative limit switch
- Position setting (`SET` field)
- Automatic power-on before motion, with SRL.15 readback
- Full status mapping: motion-complete, limits, faults, power state
- Extra PVs for raw TML registers (SRH, SRL, MER)
- Multi-axis support on RS-232, RS-485, CAN, and XPORT (IP) channels
- Configurable per-axis TML setup files (.t.zip from EasyMotion/EasySetup)
- Debug tracing via `var drvTmlDebug N` (0–4)

## Prerequisites

- EPICS Base 7.x
- Motor module (synApps motor)
- Asyn module
- TML_lib + tmlcomm shared libraries (from Technosoft or built from vendor SDK)

## Directory Structure

```
technosoft-asyn/
├── Makefile
├── README.md
├── configure/
│   ├── CONFIG
│   ├── CONFIG_SITE
│   ├── Makefile
│   ├── RELEASE          ← set EPICS_BASE, ASYN, MOTOR, TML_LIB here
│   ├── RULES
│   ├── RULES_DIRS
│   ├── RULES_TOP
│   └── RULES.ioc
├── technosoftApp/
│   ├── Makefile
│   ├── Db/
│   │   ├── Makefile
│   │   └── motor.db     ← motor record + extra TML status PVs
│   └── src/
│       ├── Makefile
│       ├── devTechnosoft.dbd
│       ├── drvTmlMotor.h
│       ├── drvTmlMotor.cpp
│       └── technosoftMain.cpp
├── tml_lib/              ← symlink or copy from motorTechnosoft/tml_lib
│   ├── config/           ← .t.zip setup files
│   ├── include/
│   │   ├── TML_lib.h
│   │   └── tmlcomm.h
│   └── lib/
│       ├── libTML_lib.so
│       └── libtmlcomm.so
└── iocBoot/
    ├── Makefile
    └── iocTml/
        ├── Makefile
        ├── st.cmd                    ← main startup script
        ├── singleAxis.cmd            ← single-axis RS-232 example
        ├── singleAxis.substitutions
        ├── multiAxis.cmd             ← multi-axis RS-485/CAN example
        └── multiAxis.substitutions
```

## Building

1. Copy or symlink the `tml_lib/` directory from the `motorTechnosoft` tree
   (or from the Technosoft SDK):

   ```bash
   ln -s ../motorTechnosoft/tml_lib .
   ```

2. Edit `configure/RELEASE` to match your EPICS installation:

   ```makefile
   EPICS_BASE = /epics/epics-base
   ASYN       = /epics/support/asyn
   MOTOR      = /epics/support/motor
   TML_LIB    = $(TOP)/tml_lib
   ```

3. Build:

   ```bash
   make
   ```

## Running

### Single Axis (RS-232)

```bash
cd iocBoot/iocTml
../../bin/linux-x86_64/technosoft st.cmd
```

The default `st.cmd` includes `singleAxis.cmd` which:
1. Creates a controller on `/dev/ttyUSB0` with host ID 255 at 9600 baud
2. Configures axis 0 with TML ID 1 and a Nanotec motor setup file
3. Loads the motor record database

Edit `singleAxis.substitutions` to set your PV prefix, motor resolution,
limits, and velocity.

### Multi-Axis (RS-485 / CAN)

Edit `st.cmd` to include `multiAxis.cmd` instead, then configure each
axis with its own TML axis ID and setup file.

## IOC Shell Commands

### TmlControllerConfig

```
TmlControllerConfig(portName, devicePath, numAxes, hostId, baudRate, movingPoll, idlePoll)
```

| Parameter    | Description |
|-------------|-------------|
| `portName`   | EPICS asyn port name (must match `PORT` in substitutions) |
| `devicePath` | `/dev/ttyUSBx` for serial, or `IP:port` for XPORT |
| `numAxes`    | Number of axes on this communication channel |
| `hostId`     | Host address on bus (1-255); for RS-232, usually the axis ID |
| `baudRate`   | 9600 (default), 19200, 38400, 115200 |
| `movingPoll` | Polling period (s) while moving |
| `idlePoll`   | Polling period (s) while idle |

### TmlAxisConfig

```
TmlAxisConfig(portName, axisNo, axisId, setupFile, homingSwitch)
```

| Parameter      | Description |
|---------------|-------------|
| `portName`     | Must match the controller's port name |
| `axisNo`       | 0-based motor record index (`ADDR` in substitutions) |
| `axisId`       | TML drive address on the bus (1-255) |
| `setupFile`    | Path to `.t.zip` or directory from EasyMotion/EasySetup |
| `homingSwitch` | `"LSN"` (negative limit) or `"LSP"` (positive limit) |

### Debug Level

```
var drvTmlDebug 2
```

| Level | Output |
|-------|--------|
| 0     | Off |
| 1     | Errors + configuration |
| 2     | Moves + poll |
| 3     | TML API calls |
| 4     | Raw register values |

## Motor Record Mapping

| Motor Record | TML Function |
|---|---|
| Move (absolute/relative) | `TS_MoveAbsolute` / `TS_MoveRelative` |
| Jog (velocity mode) | `TS_MoveVelocity` |
| Home | `TS_MoveVelocity` + `TS_SetEventOnLimitSwitch` |
| Stop | `TS_ABORT` + `TS_Stop` |
| Set position | `TS_SetPosition` |
| Position readback | `TS_GetLongVariable("TPOS")` |
| Status (MSTA) | SRH, SRL, MER register mapping |
| Power | `TS_Power(ON/OFF)` — auto-on before moves |

## Differences from motorTechnosoft (NDS version)

| Feature | motorTechnosoft (NDS) | technosoft-asyn |
|---|---|---|
| Device support layer | NDS (Cosylab) | Standard asynMotor |
| Dependency | epics-nds, boost | motor, asyn only |
| Motor record | Custom PV templates | Standard motor record |
| Thread model | NDS ThreadTask per axis | asynMotor poller thread |
| Configuration | NDS params in YAML | iocsh commands |
| Status registers | Custom PVs | Extra asyn params (SRH,SRL,MER) |
| Build complexity | Complex (NDS, boost, seq) | Simple (2 source files) |

## Author

Andrea Michelotti — <andrea.michelotti@infn.it>  
INFN Laboratori Nazionali di Frascati (LNF)
