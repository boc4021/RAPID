# CLAUDE.md — RAPID Project Context

**RAPID** is a capstone design project (Jack Parrack, Laurel Leuwerke, Matt Bracker, Chance Besancenez).
It streams GDS2 lithography patterns from a PC to an Arty Z7-20 FPGA over UART for motor-driven stage control (polar XY laser lithography).

---

## System Architecture

```
input.gds → inputParser → pcCommunication → UART → systemControl.c
             (XY→polar)    (frame+send)               (recv+ACK+motor control)
                                ↑                           │
                             gui.py ←── ACK log ────────────┘
                          (live XY scatter)

systemControl.c ── AXI GPIO (PS→PL) ──→ stepperDriver.vhd
                                     └──→ spindle.vhd (BLDC)
                                     └──→ LaserEn (GPIO pin P18)
                ── PS I2C0 (EMIO→PL) ──→ MLX90393 magnetometer (angle logging)
```

---

## Repository Layout

| Path | Description |
|------|-------------|
| `src/pcCommunication.c` | PC-side UART sender — compiled into `build/RAPID.exe` |
| `src/inputParser.c/h` | GDS2 text parser: XY coords → polar (r in µm, theta in degrees) |
| `src/platform.c/h` | Xilinx cache init wrappers |
| `src/gui.py` | PySide6 GUI — launches RAPID.exe, parses stdout, plots ACK'd points |
| `vitis_workspace/systemControl/systemControl.c` | **Active** FPGA app — packet receiver + motor/laser control |
| `vitis_workspace/systemControl/mlx90393.h/.c` | MLX90393 bare-metal I2C driver |
| `old/angleMeasure.ino` | Archived Arduino angle-tracking sketch (calibration reference) |
| `hardware/RAPID.xpr` | Vivado project file |
| `hardware/RAPID.srcs/sources_1/new/stepperDriver.vhd` | **Active** stepper FSM |
| `hardware/RAPID.srcs/sources_1/new/spindle.vhd` | **Active** BLDC 6-step commutation |
| `hardware/RAPID.srcs/sources_1/new/BLDC.vhd` | Superseded spindle draft — **must be disabled in Vivado** (see ISSUES.md H-0) |
| `hardware/RAPID.srcs/constrs_1/new/RAPID.xdc` | Pin constraints (Arty Z7-20) |
| `hardware/ip_repo/src/` | Archived early VHDL — not in block design |
| `Makefile` | PC-side build: `make`, `make run`, `make gui`, `make clean` |
| `docs/README.md` | User-facing project documentation |
| `docs/ISSUES.md` | Known issues, risks, and open TODOs |

---

## Hardware Platform

- **Board:** Digilent Arty Z7-20 (xc7z020clg400-1)
- **Toolchain:** Vivado 2025.1 + Vitis 2025.1
- **PS Clock:** 100 MHz (AXI interconnect) | **PL Clock:** 125 MHz (stepperDriver, spindle)
- **UART:** PS UART0, 115200 8N1, USB-JTAG/UART
- **I2C:** PS I2C0 via EMIO → Arduino header A4/A5 (SDA=P16, SCL=P15)
- **Vitis SDT note:** use `XPAR_<PERIPH>_0_BASEADDR` with `LookupConfig(u32 BaseAddress)` — `DEVICE_ID` macros do not exist in the 2025.1 SDT flow.

---

## Packet Wire Format

```
[ 0xAA | 0x55 | TYPE (1B) | LEN (1B) | PAYLOAD (LEN bytes) | CRC8 (1B) ]
```

| Direction | TYPE | LEN | Payload | Purpose |
|-----------|------|-----|---------|---------|
| PC → FPGA | `0x01` | 8 | `r_um` (int32 LE, µm) + `theta_deg` (float32 LE, deg) | Polar point |
| PC → FPGA | `0x03` | 0 | — | End of sequence |
| FPGA → PC | `0x81` | 8 or 0 | Echo of payload | ACK |
| FPGA → PC | `0xF0` | N | ASCII string | Debug/status |

**CRC8:** accumulating XOR over `[TYPE, LEN, PAYLOAD...]`.

**Flow control:** stop-and-wait. PC waits 32 s at startup (`FPGA_INIT_WAIT_MS`) for stepper zeroing, then up to 2 s per ACK.

---

## GPIO Control Word (27 bits, AXI GPIO Ch 1 output)

| Bits | `#define` | Description |
|------|-----------|-------------|
| `[0]` | `BIT_SPINDLE_EN` | Spindle enable |
| `[1]` | `BIT_STEPPER_DIR` | Stepper direction (0=inward, 1=outward) |
| `[2]` | `BIT_STEPPER_EN` | Stepper enable |
| `[3]` | `BIT_ZERO_REQ` | Home request — 100 ms pulse |
| `[24:4]` | `BIT_NUM_STEP` (21 bits) | Steps to move |
| `[25]` | `BIT_STEP_GO` | Move trigger — 100 ms pulse |
| `[26]` | `BIT_LASER_EN` | Laser enable |

`GPIO_MASK = 0x07FFFFFF`. Channel 2 is all-input, reserved for `step_total_out` readback.

### systemControl.c Automated Sequence

| Phase | Action |
|-------|--------|
| **0 — MLX init** | `mlx_init(&mlx, XPAR_XIICPS_0_BASEADDR)` |
| **1 — Zeroing** | `stepper_en=1` → VHDL ZEROING state; `usleep(ZERO_WAIT_US)` (30 s) |
| **2 — Spindle** | `spindle_en=1` |
| **3 — Point loop** | Compute `target_step`, move, laser on after first move, log `[THETA]`, ACK |
| **4 — End** | Clear laser/spindle/stepper, ACK, return |

**Step mapping:** `target_step = clamp(round(r_um / 33000 × 8500), 0, 8500)` — 8500 steps = 33 mm.

**Move wait:** `usleep(1200 + delta × 2000 + 10000)` µs after 100 ms `step_go` pulse.
(`run_freq=250000` @ 125 MHz → 500 Hz step rate = 2000 µs/step)

```c
#define ZERO_WAIT_US      30000000U   /* 30 s — increase if sled starts far from home */
#define FPGA_INIT_WAIT_MS 32000       /* PC-side: must be ≥ ZERO_WAIT_US + margin */
```

---

## VHDL Modules

### `stepperDriver.vhd`

125 MHz. Drives DRV8834. Key ports: `pwm_out_step`, `dir_out`, `en_out` (SLEEP pin), `prox_in`, `num_steps[20:0]`, `step_go`, `step_total_out[20:0]`.

**FSM:**
```
ZEROING → (prox_stable) → IDLE → (step_go↑) → WAKEUP → RUNNING → DONE
    ↑                                                                │
    └──────────────── zero_req↑ from any state ─────────────────────┘
```

| State | Behaviour |
|-------|-----------|
| `ZEROING` | Pulses at `zero_freq` (500 Hz) toward home; resets `step_total=0` on `prox_stable` |
| `IDLE` | Motor sleep (`en_out=0`). Waits for `step_go` or `zero_req` |
| `WAKEUP` | `en_out=1`, holds 150,000 cycles (1.2 ms) for DRV8834 wake |
| `RUNNING` | Outputs step pulses; tracks `step_total` |
| `DONE` | Motor sleep; waits for next `step_go` or `zero_req` |

`run_freq = 250,000` → 500 Hz step rate. Proximity debounce: 1,250,000 cycles (10 ms). 2FF synchroniser on `prox_in` (`ASYNC_REG`).

### `spindle.vhd`

125 MHz. Drives DRV8323. Fixed speed, fixed direction, on/off via `en`. Key ports: `en`, `en_spindle`, `INHA/INLA/INHB/INLB/INHC/INLC`.

- `INHC <= '0'`, `INLC <= '1'`, `en_spindle <= '1'` — hardcoded
- Commutation at `count_max = 6,410,256` → ~19.5 Hz commutation rate (~3.25 rev/s, ~195 RPM)
- PWM on `INHA` at ~10 kHz, 50% duty
- 1.5 s rotor alignment hold on `en` rising edge (187,500,000 cycles)

### `BLDC.vhd` (sources_1/new) — Superseded

Declares entity `Spindle` with abstract `PhA/PhB/PhC : STD_LOGIC_VECTOR(1 downto 0)` interface — incompatible with XDC and DRV8323. Replaced by `spindle.vhd`. **Must be disabled in Vivado** to avoid duplicate entity error. See ISSUES.md H-0.

### `ip_repo/src/` — Archived

`BLDC.vhd` and `stepperDriver.vhd` are early drafts with mismatched port interfaces. Not in the block design.

---

## Pin Constraints (RAPID.xdc)

All banks 3.3 V LVCMOS33.

| Signal | Pin | Function |
|--------|-----|----------|
| `En_Spindle` | R16 | DRV8323 enable |
| `INHA_0` | T14 | Phase A high-side |
| `INLA_0` | U12 | Phase A low-side |
| `INHB_0` | U13 | Phase B high-side |
| `INLB_0` | V13 | Phase B low-side |
| `INHC_0` | V15 | Phase C high-side |
| `INLC_0` | T15 | Phase C low-side |
| `dir_out_0` | V18 | Stepper direction |
| `pwm_out_step_0` | T16 | Stepper step pulse |
| `en_out_0` | V17 | Stepper enable (DRV8834 SLEEP) |
| `prox_in_0` | R17 | Proximity switch (PULLDOWN) |
| `LaserEn[0]` | P18 | Laser enable (PULLDOWN) |
| `IIC_0_0_scl_io` | P15 | I2C0 SCL (Arduino A5) |
| `IIC_0_0_sda_io` | P16 | I2C0 SDA (Arduino A4) |

---

## PC-side Build

```bash
make                         # builds build/RAPID.exe
make run PORT=COM25 FILE=input.gds
make gui                     # launches src/gui.py
make clean
```

Toolchain: MinGW-w64 / MSYS2 UCRT64 gcc, `-O2 -Wall -Wextra -std=c11 -lm`. Sources: `src/pcCommunication.c` + `src/inputParser.c`.

---

## GDS2 Input Format

```
XY <x0> : <y0>
<x1> : <y1>
...
ENDEL
```

Units: micrometres (integers). `convertToPolar()`: `r = sqrt(x²+y²)`, `theta = atan2(y,x)` normalized to [0, 360). Uses `PI = 3.14159265358979323846` (full double precision).

---

## GUI (`src/gui.py`)

PySide6 + pyqtgraph. Launches `RAPID.exe` as a `QProcess`, parses stdout with regex:
- `[ACK] r=<r> um, theta=<t> deg` → plots point
- `[FPGA] <msg>` → scrolling log
- `[RX] CRC mismatch` → error counter

Plot refreshes at 10 Hz. Default port: `COM25`, file: `input.gds`.

---

## MLX90393 Magnetometer Driver

`vitis_workspace/systemControl/mlx90393.h/.c` — bare-metal XIicPs driver for spindle angle logging.

**Protocol:** SM opcode `0x37` (T+X+Y); RM reads 7 bytes `[status, T_H, T_L, X_H, X_L, Y_H, Y_L]`. T is discarded; X/Y used for angle. Angle emitted as `[THETA] %.2f deg` after each stepper move.

**Config:** CONF1=`0x0050` (GAIN_SEL=5, 1.667x), CONF3=`0x000F` (DIG_FILT=3, OSR=3, RES=0). Conversion ~6 ms; driver waits 10 ms.

**Calibration** (from `old/angleMeasure.ino`):
```c
X_OFFSET = -47.5498,  Y_OFFSET = -23.2500
X_SCALE  =  0.00010226, Y_SCALE = 0.00010226
ANGLE_DIVISOR = 1.54   /* mechanical coupling */
```

**Angle algorithm:** cross/dot product of consecutive calibrated XY vectors → `atan2f` → accumulate. Wrap-around safe.

**Wiring:** I2C addr `0x0C` (A0/A1 to GND), 100 kHz. External 4.7 kΩ pull-ups on SCL/SDA required.

**Status:** driver complete, not yet hardware-tested. See `docs/ISSUES.md` I-1 through I-7.
