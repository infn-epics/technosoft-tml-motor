# technosoft-asyn — EPICS Motor Driver for Technosoft TML Drives

EPICS asynMotor driver for Technosoft intelligent stepper/servo drives using
the native TML serial protocol (RS-232 / RS-485 / TCP via XPORT).  Replaces
the legacy NDS-based driver and proprietary TML_lib with a direct POSIX
implementation.

## Directory Structure

```
technosoft-asyn/
├── technosoftSup/src/         C++ driver source
│   ├── drvTmlMotor.cpp/h      asynMotorController / asynMotorAxis
│   ├── tmlSerial.cpp/h        Native TML serial protocol
│   └── devTechnosoft.dbd      DBD registration
├── db/
│   ├── motor.db               Motor record + TML-specific PVs
│   └── tml_compat.db          Legacy NDS naming compatibility bridge
├── opi/                       Phoebus BOB display files
│   ├── MotorAsyn.bob          Entry point (device selector)
│   ├── Motor_Main.bob         Main container (tabs + POI controls)
│   ├── index.bob              Simple launcher
│   └── resources/
│       ├── Motor_TabMain.bob          Tab 1: everyday motor controls
│       ├── Motor_TabExpert.bob        Tab 2: expert parameters
│       ├── Motor_WindowMSTADetail.bob MSTA bit detail pop-up
│       ├── widgets/                   Reusable FLAME widget templates
│       └── motorRecord/              Standard motor record OPI forms
├── docs/                      Technosoft TML Programming Manual (text)
├── configure/                 EPICS build configuration
├── iocBoot/                   IOC startup scripts
└── spark_values.yaml / btf_values.yaml   IBEK instance configs
```

## Prerequisites

- **EPICS Base** R7.0+
- **motor** module (asynMotor support)
- **asyn** module
- **calc** module (for `scalcout` records used by POI)
- **IBEK Framework** (optional, for container-based IOC generation)

---

## OPI Display Architecture

The operator interface is built in Phoebus BOB format using the FLAME widget
style.  Displays use macro substitution (`$(P)`, `$(M)`) to bind to any motor
axis at runtime.

### Display Hierarchy

```
MotorAsyn.bob                       Entry point, device/zone selector
 ├─ Scripts/LoadMotorDevice.py       Populates device combo box
 ├─ Scripts/MotorDeviceSelect.py     Sets $(P), $(R), macros on selection
 └─ Motor_Main.bob                   Main container
     ├─ Tab "Main"
     │   ├─ Motor_TabMain.bob        Everyday motor controls
     │   ├─ POI_SEL combo            Point-of-Interest selector (if POI records exist)
     │   ├─ POI_GO button            Move to selected POI
     │   └─ POI_CURR readback        Current POI name
     └─ Tab "Expert"
         └─ Motor_TabExpert.bob      Advanced motor record parameters
```

**Macros** (set by the device selector or passed from the parent):

| Macro | Example | Purpose |
|-------|---------|---------|
| `$(P)` | `BTF:MOT:TML` | PV prefix |
| `$(M)` | `:SLTTB001D` | Motor name (with leading colon) |
| `$(R)` | `SLTTB001D` | Motor name (without colon, used in title) |
| `$(DID)` | `1` | Display instance ID (for multiple windows) |

### Tab "Main" — Motor_TabMain.bob

Provides the everyday controls an operator needs:

| Widget | PV | Type | Description |
|--------|----|------|-------------|
| **Position readback** | `$(P)$(M).RBV` | textupdate | Current user position |
| **Setpoint** | `$(P)$(M).VAL` | textentry | Absolute move target |
| **Tweak value** | `$(P)$(M).TWV` | textentry | Step size for tweak buttons |
| **Tweak forward** | `$(P)$(M).TWF` | action_button | Nudge motor forward by TWV |
| **Tweak reverse** | `$(P)$(M).TWR` | action_button | Nudge motor reverse by TWV |
| **High limit LED** | `$(P)$(M).HLS` | led | Hardware high limit switch active |
| **Low limit LED** | `$(P)$(M).LLS` | led | Hardware low limit switch active |
| **Soft limit violation** | `$(P)$(M).LVIO` | led | Software limit violation indicator |
| **Speed** | `$(P)$(M).VELO` | setpoint | Motion velocity |
| **Soft high limit** | `$(P)$(M).HLM` | spinner | User coordinate high limit |
| **Soft low limit** | `$(P)$(M).LLM` | spinner | User coordinate low limit |
| **Dial high limit** | `$(P)$(M).DHLM` | spinner | Dial coordinate high limit |
| **Dial low limit** | `$(P)$(M).DLLM` | spinner | Dial coordinate low limit |
| **Mode** | `$(P)$(M).SPMG` | choice | Stop / Pause / Move / Go |
| **Enable/Disable** | `$(P)$(M)_able.VAL` | choice + led | Motor enable toggle |
| **Sync** | `$(P)$(M).SYNC` | button | Synchronize position |
| **MSTA** | `$(P)$(M).MSTA` | textupdate | Motor status word (hex) |
| **MSTA detail** | — | action_button | Opens Motor_WindowMSTADetail.bob |
| **Progress bar** | `$(P)$(M).RBV` | progressbar | Visual position indicator (HLM→LLM range) |

### Tab "Expert" — Motor_TabExpert.bob

Exposes all motor record fields for commissioning and diagnostics:

**Drive section:**

| Widget | PV | Description |
|--------|----|-------------|
| User readback | `$(P)$(M).RBV` | User coordinate readback |
| Dial readback | `$(P)$(M).DRBV` | Dial coordinate readback |
| Raw readback | `$(P)$(M).RRBV` | Raw encoder counts |
| User setpoint | `$(P)$(M).VAL` | Absolute position (user) |
| Dial setpoint | `$(P)$(M).DVAL` | Absolute position (dial) |
| Raw setpoint | `$(P)$(M).RVAL` | Raw position direct |
| High limit (user) | `$(P)$(M).HLM` | User high limit |
| High limit (dial) | `$(P)$(M).DHLM` | Dial high limit |
| Low limit (user) | `$(P)$(M).LLM` | User low limit |
| Low limit (dial) | `$(P)$(M).DLLM` | Dial low limit |
| Enable | `$(P)$(M)_able.VAL` | Motor enable/disable |
| Relative move | `$(P)$(M).RLV` | Relative move distance |
| Jog forward | `$(P)$(M).JOGF` | Start forward jog |
| Jog reverse | `$(P)$(M).JOGR` | Start reverse jog |
| Home forward | `$(P)$(M).HOMF` | Home toward positive LS |
| Home reverse | `$(P)$(M).HOMR` | Home toward negative LS |
| SPMG | `$(P)$(M).SPMG` | Stop/Pause/Move/Go selector |
| Limit LEDs | `$(P)$(M).HLS`, `.LLS` | Limit switch indicators |
| Moving / Comm status | `$(P)$(M).DMOV`, `.STAT` | Conditional status labels |

**Dynamics section:**

| Widget | PV | Description |
|--------|----|-------------|
| Max speed | `$(P)$(M).VMAX` | Maximum allowed velocity |
| Normal speed | `$(P)$(M).VELO` | Normal move velocity |
| Backlash speed | `$(P)$(M).BVEL` | Backlash correction velocity |
| Jog speed | `$(P)$(M).JVEL` | Jog velocity |
| Base speed | `$(P)$(M).VBAS` | Minimum starting velocity |
| Acceleration | `$(P)$(M).ACCL` | Move acceleration time (s) |
| Backlash accel | `$(P)$(M).BACC` | Backlash acceleration time (s) |
| Jog accel | `$(P)$(M).JAR` | Jog acceleration (EGU/s²) |
| Backlash distance | `$(P)$(M).BDST` | Backlash compensation distance |

**Resolution section:**

| Widget | PV | Description |
|--------|----|-------------|
| Motor resolution | `$(P)$(M).MRES` | EGU per motor step |
| Encoder resolution | `$(P)$(M).ERES` | EGU per encoder count |
| Readback resolution | `$(P)$(M).RRES` | Readback step size |
| Retry deadband | `$(P)$(M).RDBD` | Positioning tolerance |
| Max retries | `$(P)$(M).RTRY` | Position retry attempts |
| Retry count | `$(P)$(M).RCNT` | Current retry count (RO) |
| Use encoder | `$(P)$(M).UEIP` | Use encoder for position |
| Use readback link | `$(P)$(M).URIP` | Use external readback PV |
| Readback delay | `$(P)$(M).DLY` | Delay before reading position |
| Readback link | `$(P)$(M).RDBL` | External readback PV name |

**Status section:**

| Widget | PV | Description |
|--------|----|-------------|
| General status | `$(P)$(M).STAT` | Alarm status |
| Soft limit violation | `$(P)$(M).LVIO` | Limit violation indicator |
| Pre-move string | `$(P)$(M).PREM` | Command string run before move |
| Post-move string | `$(P)$(M).POST` | Command string run after move |

### MSTA Detail Window — Motor_WindowMSTADetail.bob

Pop-up showing individual bits of the MSTA (Motor Status) register:

| Bit | Name | Meaning |
|-----|------|---------|
| 0 | RA_DIRECTION | Last raw motion direction |
| 1 | RA_DONE | Motion complete |
| 2 | RA_PLUS_LS | Plus limit switch active |
| 3 | RA_HOME | Home switch active |
| 4 | EA_POSITION | Encoder position available |
| 5 | — | (unused) |
| 6 | EA_SLIP_STALL | Slip/stall detected |
| 7 | EA_HOME | At home position |
| 8 | RA_MINUS_LS | Minus limit switch active |
| 9 | RA_HOMED | Homing completed |
| 10 | RA_MOVING | Non-zero velocity |
| 11 | RA_PROBLEM | Driver stopped polling |
| 12 | RA_PRESENT | Encoder present |
| 13 | RA_COMM_ERR | Communication error |
| 14 | GAIN_SUPPORT | Closed-loop capable |
| 15 | RA_MOVING | Motor is moving |

---

## Coordinate Systems: Raw, Dial, and User

The EPICS motor record uses three coordinate systems related by simple
transformations:

```
Raw (RVAL)  ──×MRES──►  Dial (DVAL)  ──±offset──►  User (VAL)
 (counts)                 (EGU)                      (EGU)
```

**Raw** (`RVAL` / `RRBV`) — integer motor steps (encoder counts).  This is
what the driver actually sends to the hardware.  Unit-less.

**Dial** (`DVAL` / `DRBV`) — raw × `MRES` (motor resolution).  Converts
counts to engineering units (e.g., mm, degrees, µsteps) but with the origin
at the hardware's zero.  The `DIR` field (Pos/Neg) can flip the sign.

**User** (`VAL` / `RBV`) — dial + `OFF` (user offset).  Lets you define a
convenient coordinate system (e.g., setting the home position to 0 or to some
reference value) without changing the hardware calibration.

**Conversion formulas:**

| Direction | Formula |
|-----------|---------|
| User → Dial | `DVAL = (VAL - OFF) × DIR_sign` |
| Dial → Raw | `RVAL = DVAL / MRES` |
| Raw → Dial | `DRBV = RRBV × MRES` |
| Dial → User | `RBV = DRBV × DIR_sign + OFF` |

Where `DIR_sign` is +1 when `DIR=Pos`, −1 when `DIR=Neg`.

**Example:** A motor has `MRES=0.001` (1 µm/step), `OFF=100.0`, `DIR=Pos`:
- Raw = 50000 counts → Dial = 50.0 mm → User = 150.0 mm
- User writes `VAL=200` → Dial = 100.0 → Raw = 100000 steps sent to drive

In the Expert tab of the OPI, all three are shown side by side (User, Dial,
Raw columns) so you can verify the calibration chain at a glance.

---

## Calibration

The **Calibration** section (Expert tab, "Calibration" header) lets you
redefine the User coordinate system without physically moving the motor.

| Field | PV suffix | Description |
|-------|-----------|-------------|
| **SET** | `.SET` | Toggle between *Use* (normal) and *Set* (calibrate) modes. In *Set* mode, writing to `.VAL` redefines the current position without commanding motion — a yellow border appears around the position indicator as a visual warning. |
| **OFF** | `.OFF` | User offset: `User = Dial × DIR_sign + OFF`. Changing OFF shifts the entire User coordinate system. |
| **DIR** | `.DIR` | Direction: `Pos` (+1) or `Neg` (−1). Flips the sign in the Dial → User conversion. |
| **FOFF** | `.FOFF` | Freeze offset mode: *Variable* or *Frozen*. In *Variable* mode, when you write a new `.VAL` the motor moves and `.OFF` stays constant. In *Frozen* mode, `.OFF` is recalculated so that `.VAL` appears to stay at the written value (the hardware position changes but the offset absorbs it). |
| **SYNC** | `.SYNC` | Pressing SYNC forces the motor record to re-read the hardware position and recalculate User/Dial from the current Raw value. Useful after manually resetting the drive position or after a power cycle. |

**Typical calibration workflow:**

1. Home the motor (`.HOMF` or `.HOMR`) so the hardware knows its raw origin.
2. Set `.SET` = *Set*, then write the desired User value into `.VAL`.
   The motor record recalculates `.OFF` so that the current position reads
   as the value you entered.  No motion occurs.
3. Set `.SET` = *Use* to return to normal operation.

---

## Dynamics

The **Dynamics** section (Expert tab, "Dynamics" header) controls all
velocity and acceleration parameters.  There are three columns — *Normal*,
*Backlash*, and *Jog* — each with its own speed/acceleration pair, plus
home-specific dynamics:

| Field | PV suffix | Unit | Description |
|-------|-----------|------|-------------|
| **VMAX** | `.VMAX` | EGU/s | Maximum allowed velocity. The motor record clamps any requested speed to this value. |
| **VELO** | `.VELO` | EGU/s | Normal move velocity, used for absolute and relative moves. |
| **VBAS** | `.VBAS` | EGU/s | Base (minimum starting) velocity. Relevant for stepper drivers that need a minimum pulse rate. |
| **ACCL** | `.ACCL` | s | Acceleration time — the time to ramp from `VBAS` to `VELO`. The driver converts this to EGU/s² internally. |
| **BVEL** | `.BVEL` | EGU/s | Backlash correction velocity. |
| **BACC** | `.BACC` | s | Backlash acceleration time. |
| **BDST** | `.BDST` | EGU | Backlash distance. If non-zero, the motor overshoots by this amount and then reverses to eliminate mechanical backlash. |
| **JVEL** | `.JVEL` | EGU/s | Jog velocity (used by `.JOGF`/`.JOGR`). |
| **JAR** | `.JAR` | EGU/s² | Jog acceleration rate (note: in EGU/s², not seconds). |
| **HVEL** | `.HVEL` | EGU/s | Home-search velocity. |
| **HACC** | `.HACC` | s | Home-search acceleration time. |

**All velocity/acceleration fields are in EGU (engineering units) as defined
by the resolution parameters** — see the Resolution section below.  When you
change `MRES`, the meaning of 1 EGU changes, so VELO=1000 could mean
1000 µsteps/s or 1000 mm/s depending on how `MRES` maps counts to EGU.
**The resolution does not automatically rescale existing dynamics values** —
if you change `MRES` you must manually update velocities and accelerations
to match the new scale.

---

## Resolution

The **Resolution** section (Expert tab, "Resolution" header) defines how raw
hardware counts translate to engineering units and controls the positioning
feedback loop.

**Core resolution fields:**

| Field | PV suffix | Unit label | Description |
|-------|-----------|------------|-------------|
| **MRES** | `.MRES` | EGU/step | Motor resolution — engineering units per motor step. This is the fundamental conversion factor: `Dial = Raw × MRES`. |
| **ERES** | `.ERES` | EGU/count | Encoder resolution — engineering units per encoder count. Used when `UEIP=Yes` to convert the encoder readback. |
| **RRES** | `.RRES` | EGU/count | Readback resolution — used when an external readback link (`RDBL`) is configured. |

The EGU/step annotation on the OPI shows the physical meaning: the
numerator is the engineering unit (e.g., mm, degrees), the denominator is
one hardware step or encoder count.

**Feedback and retry fields:**

| Field | PV suffix | Description |
|-------|-----------|-------------|
| **UEIP** | `.UEIP` | Use Encoder If Present. When *Yes*, the motor record reads the encoder position (`REP`) to determine the actual position. When *No*, it uses the motor step count (`RMP`). |
| **URIP** | `.URIP` | Use Readback link. When *Yes*, the position comes from an external PV (`RDBL`) instead of the driver. |
| **RDBD** | `.RDBD` | Retry deadband (in EGU). After a move completes, if the position error exceeds `RDBD`, the motor record automatically retries the move. |
| **RTRY** | `.RTRY` | Maximum number of retry attempts. |
| **RCNT** | `.RCNT` | Current retry count (read-only). |
| **DLY** | `.DLY` | Readback delay (seconds). Time to wait after motion stops before reading the final position (allows mechanical settling). |
| **RDBL** | `.RDBL` | Readback input link — PV name for an external position source. |

**How resolution affects coordinate conversions:**

`MRES` is central to the entire coordinate chain:

```
Raw (counts) ──× MRES──► Dial (EGU) ──± OFF──► User (EGU)
```

- When `UEIP=No`: both setpoint and readback use `MRES`.
  `RVAL = nint(DVAL / MRES)` and `DRBV = RRBV × MRES`.
- When `UEIP=Yes`: the setpoint still uses `MRES` to convert Dial → Raw,
  but the readback uses `ERES` to convert encoder counts → Dial.
  If `ERES` is left at zero, the motor record defaults to using `MRES` for
  readback as well.

**Resolution and Dynamics:**

Dynamics fields (`VELO`, `VBAS`, `BVEL`, `JVEL`, `HVEL`) are expressed in
EGU/s, and acceleration fields (`ACCL`, `BACC`, `HACC`) are times in seconds
to ramp to the corresponding velocity.  **Resolution does not automatically
rescale dynamics.**  These fields are independent — changing `MRES` changes
what 1 EGU *means* physically, but does **not** update VELO, ACCL, etc.
If you halve `MRES`, the same `VELO=1000` now means half the physical speed
it meant before.  You must manually adjust velocities and accelerations
whenever you change the resolution.

---

## Servo (PID Closed-Loop)

The **Servo** section (Expert tab, "Servo" header) provides PID loop
coefficients for closed-loop positioning.  These fields are passed to the
motor controller via the asyn driver.

| Field | PV suffix | Description |
|-------|-----------|-------------|
| **CNEN** | `.CNEN` | Closed-loop enable (Torque enable). When *Enable*, the motor record sets the GAIN_SUPPORT bit and activates the drive's servo loop. |
| **PCOF** | `.PCOF` | Proportional coefficient (P gain). |
| **ICOF** | `.ICOF` | Integral coefficient (I gain). |
| **DCOF** | `.DCOF` | Derivative coefficient (D gain). |
| **FRAC** | `.FRAC` | Move fraction — fraction of the commanded move to execute during closed-loop corrections. Default 1.0 means full move; lower values provide more conservative positioning. |

For the Technosoft TML drives, the PID tuning is typically done in the TML
firmware (EasyMotion Studio) and stored in the drive's EEPROM.  The motor
record PID fields (`PCOF`, `ICOF`, `DCOF`) are available for pass-through to
the driver, but the TML drive uses its own internal servo loop parameters.
The `CNEN` field controls the drive's power stage (axis ON/OFF).

**Additional status/control fields** shown in the Expert tab Status section:

| Field | PV suffix | Description |
|-------|-----------|-------------|
| **FLNK** | `.FLNK` | Forward link — PV processed after motor record completes. |
| **PREM** | `.PREM` | Pre-move string — command executed before motion starts. |
| **POST** | `.POST` | Post-move string — command executed after motion ends. |

---

## TML Home (`CMD:FHOME`)

The standard motor record `.HOMF`/`.HOMR` fields have a built-in soft
protection: if the target limit switch is already active (e.g. `LLS=1` when
requesting `.HOMR`), the motor record silently drops the command — it never
reaches the driver.  This is a problem when axes power up sitting on their
home limit switch, which is the normal "parked" position.

`CMD:FHOME` is a smart homing command that bypasses this limitation.  It
uses the `homingSwitch` parameter from `TmlAxisConfig` to determine the
correct homing direction (`"LSN"` → reverse, `"LSP"` → forward) and handles
two cases:

| Situation | Behavior |
|-----------|----------|
| Home limit **active** (e.g. LSN=1 for `homingSwitch="LSN"`) | Instant: `SetPosition(0)` + `homed=1`, no motion |
| Home limit **not active** | Full home move in the configured direction using VELO/ACCL from the motor record |

**PV:** `$(P)$(M):CMD:FHOME` — write 1 to trigger.

**Usage from the IOC shell:**

```
# Home an axis that is already sitting on its LSN limit:
dbpf SPARC:MOT:TML:UTLFLG01:CMD:FHOME 1

# Home an axis that is away from the limit — starts a real home move:
dbpf SPARC:MOT:TML:GUNFLG01:CMD:FHOME 1
```

**Comparison with `.HOMR`/`.HOMF`:**

| Feature | `.HOMR`/`.HOMF` | `CMD:FHOME` |
|---------|-----------------|-------------|
| Direction | Explicit (R=reverse, F=forward) | Automatic from `TmlAxisConfig homingSwitch` |
| Works when already at limit | No — silently blocked | Yes — instant set-zero |
| Motor record integration | Full (soft limits, MSTA) | Bypasses motor record soft limit check |
| Typical use | Manual homing from OPI | Automated homing at startup, or when at limit |

---

## Database Records

### motor.db — Primary Motor Record

Instantiates one EPICS motor record per axis plus TML-specific status and
command records.

**Macros:**

| Macro | Description | Default |
|-------|-------------|---------|
| `P` | PV prefix | — |
| `M` | Motor name | — |
| `DESC` | Description | — |
| `PORT` | Asyn port name | — |
| `ADDR` | Axis number (0-based) | — |
| `DIR` | Direction (Pos/Neg) | Pos |
| `MRES` | Motor resolution (EGU/step) | — |
| `VELO` | Velocity | — |
| `DHLM` | Dial high limit | 0 |
| `DLLM` | Dial low limit | 0 |
| `EGU` | Engineering units | ustep |
| `UEIP` | Use encoder if present | Yes |

**Motor record:** `$(P)$(M)` — type `motor`, DTYP `asynMotor`, OUT `@asyn($(PORT),$(ADDR))`

**Enable/disable:** `$(P)$(M)_able` — type `bo`, controls `$(P)$(M).DISP`

**TML status registers (read-only, I/O Intr scan):**

| Record | Type | Asyn Param | Description |
|--------|------|-----------|-------------|
| `$(P)$(M):SRH` | longin | `TML_SRH` | Status Register High |
| `$(P)$(M):SRL` | longin | `TML_SRL` | Status Register Low |
| `$(P)$(M):MER` | longin | `TML_MER` | Motion Error Register |
| `$(P)$(M):MCR` | longin | `TML_MCR` | Motion Control Register |
| `$(P)$(M):MSR` | longin | `TML_MSR` | Motion Status Register |
| `$(P)$(M):ISR` | longin | `TML_ISR` | Interrupt Status Register |
| `$(P)$(M):ACTIVE` | bi | `TML_ACTIVE` | Axis active state (Inactive/Active) |
| `$(P)$(M):FAULT` | stringin | `TML_FAULT_TEXT` | Human-readable fault string |
| `$(P)$(M):SETUP` | stringin | `TML_SETUP_FILE` | TML setup file path |
| `$(P)$(M):APOS` | ai | `TML_APOS` | Actual encoder position |
| `$(P)$(M):CSPD` | ai | `TML_CSPD` | Commanded speed (IU/s) |
| `$(P)$(M):POTM` | ai | `TML_POTM` | Potentiometer/ADC readback |

**TML command records (write 1 to trigger):**

| Record | Asyn Param | Description |
|--------|-----------|-------------|
| `$(P)$(M):CMD:RSTFAULT` | `TML_RESET_FAULT` | Reset drive faults |
| `$(P)$(M):CMD:SAVE` | `TML_SAVE_EEPROM` | Save parameters to EEPROM |
| `$(P)$(M):CMD:RESET` | `TML_RESET_DRIVE` | Reset drive processor |
| `$(P)$(M):CMD:FHOME` | `TML_FORCE_HOME` | TML Home — smart homing (see below) |

**Bit-decoded status (calc records from raw registers):**

| Record | Source | Bit | Description |
|--------|--------|-----|-------------|
| `$(P)$(M):SRL:GFLT` | SRL | 8 | Global fault |
| `$(P)$(M):SRL:AFLT` | SRL | 9 | Axis fault |
| `$(P)$(M):SRL:MC` | SRL | 10 | Motion complete |
| `$(P)$(M):SRL:HOME` | SRL | 11 | Home found |
| `$(P)$(M):SRL:LSP` | SRL | 13 | Positive limit switch |
| `$(P)$(M):SRL:LSN` | SRL | 14 | Negative limit switch |
| `$(P)$(M):SRL:AXON` | SRL | 15 | Axis ON (power stage) |
| `$(P)$(M):SRH:FERR` | SRH | 0 | Following/position error |
| `$(P)$(M):SRH:OVSP` | SRH | 1 | Over-speed |
| `$(P)$(M):SRH:OCUR` | SRH | 2 | Over-current |
| `$(P)$(M):SRH:I2T` | SRH | 3 | I²T protection |
| `$(P)$(M):SRH:OTMP` | SRH | 4 | Over-temperature |
| `$(P)$(M):SRH:UVLT` | SRH | 5 | Under-voltage |
| `$(P)$(M):SRH:OVLT` | SRH | 6 | Over-voltage |
| `$(P)$(M):SRH:INPOS` | SRH | 9 | In-position |
| `$(P)$(M):SRH:MCOMPLETE` | SRH | 10 | Motion complete |
| `$(P)$(M):SRH:FLT` | SRH | 15 | General fault (OR) |
| `$(P)$(M):MER:FERR` | MER | 0 | Following error |
| `$(P)$(M):MER:OCUR` | MER | 1 | Over-current |
| `$(P)$(M):MER:I2T` | MER | 2 | I²T fault |
| `$(P)$(M):MER:OVSP` | MER | 3 | Over-speed |
| `$(P)$(M):MER:POVF` | MER | 4 | Position overflow |
| `$(P)$(M):MER:OVLT` | MER | 5 | Over-voltage |
| `$(P)$(M):MER:LSP` | MER | 6 | Positive limit switch |
| `$(P)$(M):MER:LSN` | MER | 7 | Negative limit switch |
| `$(P)$(M):MER:HOME` | MER | 8 | Home switch |
| `$(P)$(M):MER:UVLT` | MER | 9 | Under-voltage |
| `$(P)$(M):MER:OTMP` | MER | 10 | Over-temperature |
| `$(P)$(M):MCR:MODE` | MCR | 0-3 | Motion mode bits |
| `$(P)$(M):MCR:AXON` | MCR | 15 | Axis on bit |

**Direction/offset/resolution forwarding:**

| Record | Field | Description |
|--------|-------|-------------|
| `$(P)$(M)Direction` | DIR → `MOTOR_REC_DIRECTION` | Forwards direction to driver |
| `$(P)$(M)Offset` | OFF → `MOTOR_REC_OFFSET` | Forwards offset to driver |
| `$(P)$(M)Resolution` | MRES → `MOTOR_REC_RESOLUTION` | Forwards resolution to driver |

### tml_compat.db — Legacy NDS Naming Compatibility

Creates PVs matching the legacy NDS-based TML IOC naming convention so
existing OPI screens work unchanged with the new asyn motor driver.

**Key compatibility mappings:**

| Legacy PV | Maps To | Description |
|-----------|---------|-------------|
| `$(P)$(M):MSTA` | `$(P)$(M).MSTA` | Motor status bits (mbbiDirect) |
| `$(P)$(M):RSRH_RB` | `$(P)$(M):SRH` | SRH register bits (mbbiDirect) |
| `$(P)$(M):RSRL_RB` | `$(P)$(M):SRL` | SRL register bits (mbbiDirect) |
| `$(P)$(M):RMER_RB` | `$(P)$(M):MER` | MER register bits (mbbiDirect) |
| `$(P)$(M):STAT` | calc from `:ACTIVE` | State: 4=PROCESSING, 2=DISABLED |
| `$(P)$(M):RBV` | `$(P)$(M).RBV` | Position readback (ai) |
| `$(P)$(M):VAL_SP` | → `$(P)$(M).VAL` | Position setpoint (ao) |
| `$(P)$(M):VAL_RB` | `$(P)$(M).DVAL` | Dial value readback |
| `$(P)$(M):BUSY` | `$(P)$(M).MOVN` | Busy/moving flag |
| `$(P)$(M):POTM_RB` | `$(P)$(M):POTM` | Potentiometer readback |
| `$(P)$(M):VELO_SP/RB` | `$(P)$(M).VELO` | Velocity setpoint/readback |
| `$(P)$(M):ACCL_SP/RB` | `$(P)$(M).ACCL` | Acceleration setpoint/readback |
| `$(P)$(M):HVEL_SP/RB` | `$(P)$(M).HVEL` | Home velocity setpoint/readback |
| `$(P)$(M):JVEL_SP/RB` | `$(P)$(M).JVEL` | Jog velocity setpoint/readback |
| `$(P)$(M):JAR_SP/RB` | `$(P)$(M).JAR` | Jog accel setpoint/readback |
| `$(P)$(M):HAR_SP/RB` | (local) | Home accel (not in motor record) |
| `$(P)$(M):EGU2MM` | (local ao) | EGU-to-mm conversion factor |
| `$(P)$(M):MM_RBV` | calc(RBV×EGU2MM) | Position in mm |
| `$(P)$(M):VAL_MM_SP` | → VAL via calcout | Position setpoint in mm |

**Action dispatch (legacy state machine):**

The legacy OPI used an action selector (`ACT`) with an execute button (`ACTX_SP`):

| ACT value | Action | Motor record field |
|-----------|--------|--------------------|
| 0 | NONE | (no-op) |
| 1 | MOVE ABS | → `.VAL` |
| 2 | MOVE REL | → `.RLV` |
| 3 | HOME | → `.HOMR` or `.HOMF` (per `HOMEDIR` macro) |
| 4 | JOG FWD | → `.JOGF` |
| 5 | JOG REV | → `.JOGR` |

Write `ACTX_SP=1` to start, `ACTX_SP=2` to stop.

---

## Data Flow

```
OPI (Phoebus)                   EPICS IOC                      Hardware
─────────────                   ─────────                      ────────
                                                               
$(P)$(M).VAL  ──write──►  motor record      ──asyn──►  drvTmlMotor.cpp
                          │ DTYP=asynMotor             │ TmlAxis::move()
                          │ OUT=@asyn(PORT,ADDR)       │ TS_MoveAbsolute()
                          │                            ▼
                          │                       tmlSerial.cpp
                          │                       │ writeData32(CPOS)
                          │                       │ sendCommand(UPD)
                          │                       ▼
                          │                    ┌──────────┐
                          │                    │ TML Drive │
                          │                    │ (RS-485 / │
                          │                    │  TCP)     │
                          │                    └──────────┘
                          │                            │
$(P)$(M).RBV  ◄──poll───  motor record  ◄──poll──  readData32(TPOS/APOS)
$(P)$(M):SRL  ◄──I/O Intr callback──────────────  readData16(SRL)
$(P)$(M):MER  ◄──I/O Intr callback──────────────  readData16(MER)
$(P)$(M):APOS ◄──I/O Intr callback──────────────  readData32(APOS)
```

The driver polls each axis at the configured rate (moving poll / idle poll).
Status registers (SRL, SRH, MER, MCR, MSR, ISR) and auxiliary readbacks
(APOS, CSPD, POTM) are updated via asyn parameter callbacks on every poll
cycle (POTM and others on a slower 1-in-10 cycle to reduce bus traffic).

