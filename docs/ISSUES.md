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

### S-2: `protocol.h` sync is manual — drift risk
`src/protocol.h` (PC), `vitis_workspace/systemControl/protocol.h` (FPGA copy), and the constants block at the top of `tests/fpga_sim.py` must be kept identical by hand. There is no automated check. If any one copy drifts, framing will silently misparse packets — CRC mismatches are likely but not guaranteed (e.g. a changed `POINT_LEN` would not affect the CRC).
**Mitigation:** run `make check-proto` before each Vitis build — it diffs the `#define` lines between both copies and fails if they diverge.

### S-3: `FPGA_INIT_WAIT_MS` and `ZERO_WAIT_US` are not compile-time coupled
The PC-side `FPGA_INIT_WAIT_MS` (32 000 ms, in `pcCommunication.c`) must always exceed the FPGA-side `ZERO_WAIT_US` (30 000 000 µs = 30 s, in `systemControl.c`). The two values live in different files and different build environments with no shared assertion. If `ZERO_WAIT_US` is increased without also updating `FPGA_INIT_WAIT_MS`, the PC will start sending packets before the FPGA finishes zeroing, causing the first points to be silently dropped.
**Mitigation:** whenever `ZERO_WAIT_US` changes, update `FPGA_INIT_WAIT_MS` to `(ZERO_WAIT_US / 1000) + 2000`.

### S-4: E2E test only counts the first XY block in a GDS file
`count_gds_points()` in `tests/e2e_test.py` stops at the first `ENDEL` token, mirroring the behaviour of `inputParser.c`. A GDS file with multiple boundary elements will silently have only its first block processed. This is consistent with the parser but limits test coverage of multi-element patterns.

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

### I-5: Angle accumulator has no absolute reference
`accumulated_angle` integrates deltas from power-on. Noise accumulates without bound, and a re-init resets the counter to 0° regardless of spindle position. This is acceptable for logging-only use but must be addressed before closed-loop control.

### I-6: 10 ms conversion wait is tied to CONF3 settings
The `usleep(10000)` in `mlx_read_angle` assumes DIG_FILT=3, OSR=3 (~6 ms conversion). If CONF3 changes, this wait must be updated or stale data will be read. See also F-2.

---

## Future Work

### F-1: Closed-loop spindle theta control
Use MLX90393 angle feedback to command the spindle to the `theta_deg` target in each `TYPE_POINT` packet.

### F-2: DRDY polling instead of fixed wait
Replace the `usleep(10000)` in `mlx_read_angle` with polling the DRDY status bit to reduce per-sample latency. Resolves I-7 dependency on CONF3 settings.

### F-3: MLX90393 bus-hang timeout — see I-1

### F-4: Verify `step_total_out` → GPIO ch2 — see H-1
