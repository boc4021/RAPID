# RAPID — Known Issues & Open Work

---

## Active Hardware Issues

### H-0: Duplicate entity `Spindle` in active source set
`sources_1/new/BLDC.vhd` and `sources_1/new/spindle.vhd` both declare `entity Spindle`. If both are active, Vivado will fail with a duplicate entity error. `BLDC.vhd` is a superseded draft with an abstract `PhA/PhB/PhC` interface incompatible with the XDC.
**Fix:** right-click `BLDC.vhd` in Vivado Sources → **Disable File**, or remove it from the project.

### H-1: `step_total_out` → GPIO channel 2 wiring unverified
The connection from `stepperDriver.vhd`'s `step_total_out` to AXI GPIO channel 2 in the block design has not been confirmed end-to-end. `XGpio_DiscreteRead(&gpio, 2)` may not return valid position data until verified.

---

## Active Software Issues

### S-1: Spindle runs open-loop
`theta_deg` from each `TYPE_POINT` packet is available in `systemControl.c` but not used to command the spindle. The spindle runs fixed speed; full theta control requires closed-loop MLX90393 feedback (not yet implemented).

---

## I2C / MLX90393 Integration Risks

### I-1: Bus hang on `XIicPs_BusIsBusy`
The `while (XIicPs_BusIsBusy(...)) {}` spin loop in `mlx90393.c` has no timeout. A missing pull-up, bad EMIO routing, or sensor lockup will hang the system silently.
**Fix (F-3):** add an iteration counter; return `XST_FAILURE` after N cycles.

### I-2: CONF1 direct write may clobber OTP bits
`mlx_init` writes `0x0050` directly to CONF1 without a read-modify-write. After RT reset the OTP defaults likely have bits [3:0] = 0, making this safe — but it has not been verified against the specific part's OTP.
**Mitigation:** if unexpected init behaviour is seen, add a read-modify-write for CONF1.

### I-3: Calibration constants are setup-specific
`MLX_X/Y_OFFSET` and `MLX_X/Y_SCALE` were measured in the original Arduino test rig (`old/angleMeasure.ino`). A different sensor mount position or magnet will produce wrong angles.
**Mitigation:** run a calibration pass before relying on angle values.

### I-4: SCL/SDA pin assignment unverified against board schematic
XDC uses `SCL=P15` (A5), `SDA=P16` (A4) from the Digilent master constraints file. Verify against the physical board schematic before generating a bitstream.

### I-5: EX and RT status not checked in `mlx_init`
`mlx_cmd(EX)` and `mlx_cmd(RT)` return values are discarded. A failed reset leaves the sensor in an unknown state before CONF writes.
**Fix (F-5):** check that `mlx_cmd(RT)` returns `0x01` (RESET status); return `XST_FAILURE` if not.

### I-6: Angle accumulator has no absolute reference
`accumulated_angle` integrates deltas from power-on. Noise accumulates without bound, and a re-init resets the counter to 0° regardless of spindle position. This is acceptable for logging-only use but must be addressed before closed-loop control.

### I-7: 10 ms conversion wait is tied to CONF3 settings
The `usleep(10000)` in `mlx_read_angle` assumes DIG_FILT=3, OSR=3 (~6 ms conversion). If CONF3 changes, this wait must be updated or stale data will be read. See also F-2.

---

## Future Work

### F-1: Closed-loop spindle theta control
Use MLX90393 angle feedback to command the spindle to the `theta_deg` target in each `TYPE_POINT` packet.

### F-2: DRDY polling instead of fixed wait
Replace the `usleep(10000)` in `mlx_read_angle` with polling the DRDY status bit to reduce per-sample latency. Resolves I-7 dependency on CONF3 settings.

### F-3: MLX90393 bus-hang timeout — see I-1

### F-4: Verify `step_total_out` → GPIO ch2 — see H-1

### F-5: Check RT/EX status in `mlx_init` — see I-5
