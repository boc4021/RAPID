# RAPID — Known Issues & Open Work

This file tracks hardware bugs, software risks, and planned future work.
It is separate from `README.md` (which covers project summary, setup, and usage only).

---

## Active Hardware Issues

### H-1: `step_total_out` → GPIO channel 2 wiring unverified
The `step_total_out` port of `stepperDriver.vhd` is intended to feed back the
absolute stepper position to the PS via AXI GPIO channel 2.  The connection in
the Vivado block design has not been verified end-to-end.  Until confirmed,
`XGpio_DiscreteRead(&gpio, 2)` may not return meaningful position data.

---

## Active Software Issues

### S-1: Spindle runs open-loop
`theta_deg` in each `TYPE_POINT` packet is received and available in
`systemControl.c` but is not yet used to command the spindle to a specific
angular position.  The spindle runs at a fixed speed and the angle is assumed
from timing only.  Full theta control requires closed-loop feedback.

---

## I2C / MLX90393 Integration Risks

### I-1: Bus hang on `XIicPs_BusIsBusy`
`mlx90393.c` uses a bare spin loop (`while (XIicPs_BusIsBusy(...)) {}`) between
I2C send and receive.  If the bus never releases — due to missing pull-up
resistors, incorrect EMIO routing, or a sensor lockup — the system hangs
silently with no timeout or error recovery.
**Mitigation:** add a counter and return `XST_FAILURE` after N iterations if
the bus remains busy.

### I-2: CONF1 bit layout assumption
The CONF1 register value `0x5F` is computed assuming:
`GAIN_SEL[6:4] = 5`, `DIG_FILT[3:2] = 3`, `OSR[1:0] = 3`.
This matches the Adafruit_MLX90393 library and the Melexis MLX90393EF datasheet
rev 1.0.  Some earlier or later silicon revisions define bits differently.
**Mitigation:** verify against the datasheet for the specific part on the PCB.

### I-3: Calibration constants are setup-specific
`MLX_X_OFFSET`, `MLX_Y_OFFSET`, `MLX_X_SCALE`, `MLX_Y_SCALE` were measured in
the original Arduino test setup (`tests/old/anglemeasure.ino`).  If the sensor
is mounted in a different position relative to the spindle magnet, or a
different magnet is used, these values will produce incorrect angles.
**Mitigation:** run a calibration pass before relying on absolute angle values.

### I-4: SCL/SDA pin assignment assumes Digilent master XDC mapping
The XDC constraints use `SCL = P15` (Arduino A5) and `SDA = P16` (Arduino A4)
taken from the Digilent Arty Z7-20 master constraints file.  Verify these pin
numbers against the board schematic before generating a new bitstream.

---

## Future Work

### F-1: Closed-loop spindle theta control
Once the MLX90393 angle logging is verified, the next step is a feedback loop:
compare measured angle to the `theta_deg` target in each `TYPE_POINT` packet
and adjust spindle timing or speed accordingly.

### F-2: DRDY polling instead of fixed wait
`mlx_read_angle()` uses `usleep(10000)` after the SM command.  Replacing this
with polling the DRDY bit in the status byte would reduce latency (current
conversion is ~6 ms; worst case with margin is ~10 ms as coded).

### F-3: MLX90393 bus-hang timeout
See I-1 above.  Add a maximum iteration count to the `XIicPs_BusIsBusy` loops
in `mlx90393.c` to prevent silent hangs.

### F-4: Stepper position readback verification
See H-1 above.  Verify the `step_total_out` → GPIO ch2 connection in the Vivado
block design and confirm that `XGpio_DiscreteRead(&gpio, 2)` returns the correct
stepper position after each move.
