# KiCad Starter Exercise

This is a blank-but-valid KiCad 10 training project with a 60 mm × 40 mm board
outline. It teaches the workflow without pretending to be the production base PCB.
WIP!! ( don't do quite yet :P )

Open `hand_training.kicad_pro` in KiCad 10.

## Target circuit

| Reference | Part | Pins or purpose |
| --- | --- | --- |
| `J1` | 1×2 power connector | `5V_ACTUATOR`, `GND` |
| `J2` | 1×3 servo connector | `5V_ACTUATOR`, `GND`, `SERVO_PWM` |
| `J3` | 1×3 force-sensor connector | `3V3`, `GND`, `FORCE_SENSE` |
| `J4` | 1×4 host/controller connector | `3V3`, `GND`, `UART_TX`, `UART_RX` |
| `R1` | 1 kΩ resistor | In series with `D1` |
| `D1` | LED | `3V3` indicator through `R1` to `GND` |
| `TP1`–`TP5` | Test points | One each for both rails, ground, PWM, and force signal |

The same information is available in `connection_spec.csv` for checking work.

## Exercise

1. **Schematic** place and annotate the connectors, resistor,
   LED, and test points. Wire or label the named nets. Add `PWR_FLAG` symbols
   only where you understand why ERC needs them.
2. **Footprints** assign generic 2.54 mm pin-header footprints,
   an axial or through-hole resistor, a through-hole LED, and test-point
   footprints.
3. **ERC** run Inspect → Electrical Rules Checker. Resolve or
   explain every warning; do not simply exclude all violations.
4. **PCB** update the PCB from the schematic, place everything
   inside the supplied outline, keep the force-signal path away from the
   actuator rail, and route the board.
5. **DRC and 3D view** run the Design Rules Checker and open the
   3D Viewer. Save one screenshot of each.

## Completion check

- The schematic and PCB references match.
- ERC and DRC have no unexplained violations.
- `5V_ACTUATOR` is not accidentally shorted to `3V3`.
- Every connector has a ground return.
- The member can identify the actuator-power, logic, signal, and communication
  portions of the design.

## Boundaries

This board is not intended for fabrication. It contains no regulator,
protection design, microcontroller, calibrated analog front end, or validated
connector choice. It is not an electrotactile circuit. Those omissions are
deliberate discussion prompts for the real base-PCB requirements phase.
