# Precision Expression Controller

Programmable expression/CV controller firmware and hardware notes for the Raspberry Pi Pico H prototype.

The current build uses:

- Raspberry Pi Pico H
- MCP4725 I2C DAC
- 0.91 inch 128x32 I2C OLED
- Rotary encoder with push switch
- Two TRS jacks: expression pedal input and Therevox CV output
- USB power bank or USB wall power

## Start Here

| Need | File |
| --- | --- |
| Arduino firmware | `pico/PrecisionExpressionControllerPico/PrecisionExpressionControllerPico.ino` |
| Exact wiring and bench test | `docs/pico-prototype.md` |
| Exact perfboard/enclosure layout | `docs/pico-physical-layout.md` |
| BOM | `docs/bom.md` |
| Product notes | `docs/product-plan.md` |
| Console simulator | `sim/pico_console_sim.cpp` |
| 3D-print enclosure files | `hardware/enclosure/` |

## Voltage Model

The firmware now uses one bend direction at a time. This gives the selected direction the full DAC voltage range instead of splitting the range around a center point.

```text
UP mode:
  heel/no-bend: 0.000 V
  toe major 6th: 3.214 V with the current response fit

DOWN mode:
  heel/no-bend: 3.300 V
  toe major 6th: 0.086 V with the current response fit
```

Tune the Therevox with the expression pedal heel-down in the active direction. Double-click the encoder to toggle `UP`/`DOWN`.

This prototype is currently an active `0-3.3V` CV source. For a standard Therevox external-CV test, that voltage must appear on the physical `Tip` of the plug going into the Therevox, with physical `Sleeve` as ground. Do not trust breakout lug labels until you have checked the actual plug with a meter.

For a real 1V/oct path, `cv 1000` should be one octave above `cv 0`. If `cv 1000` is not one octave, re-check that the DAC voltage is on the physical plug `Tip` to physical `Sleeve`, not on the expression reference path or a switched jack lug.

For precise intervals, use `response <cents>` in `docs/pico-prototype.md`. Standard 1V/oct is `response 3960` because full DAC scale is about `3.3V`. Your current compressed bench path measured about `924` cents at full DAC scale, so the default bench map uses `response 924` and caps the selectable interval at a major 6th.

If an interval is wildly short, first run the hard voltage test:

```text
cv 0
cv 1000
cv off
```

Tune the Therevox at `cv 0`, then send `cv 1000` and check the tuner. If the wiring is on the real CV input, that should be one octave.

## Build And Upload

Use Arduino IDE.

1. Install the board package `Raspberry Pi Pico/RP2040/RP2350` by Earle F. Philhower.
2. Open `pico/PrecisionExpressionControllerPico/PrecisionExpressionControllerPico.ino`.
3. Select `Tools > Board > Raspberry Pi Pico/RP2040/RP2350 > Raspberry Pi Pico`.
4. Plug in the Pico H over USB.
5. Select the Pico under `Tools > Port`.
6. Click Upload.
7. Open Serial Monitor at `115200`.

If the Pico does not show as a port, unplug it, hold `BOOTSEL`, plug USB back in, release `BOOTSEL`, then upload again.

## Smoke Tests

Mac-side simulator:

```bash
g++ -std=c++17 sim/pico_console_sim.cpp -o /tmp/expctrl_pico_console_sim
/tmp/expctrl_pico_console_sim
```

Host math test:

```bash
g++ -std=c++17 test/test_pico_core.cpp -o /tmp/expctrl_pico_core_test
/tmp/expctrl_pico_core_test
```

Arduino compile:

```bash
arduino-cli compile --fqbn rp2040:rp2040:rpipico pico/PrecisionExpressionControllerPico
```

Expected simulator result:

```text
SIM RESULT: PASS
```

Expected hardware boot once the DAC and OLED are wired:

```text
MCP4725 PASS at 0x62
SSD1306 PASS at 0x3C
```

## Controls

| Action | Result |
| --- | --- |
| Turn encoder clockwise | Increase interval size by one semitone, up to major 6th |
| Turn encoder counterclockwise | Decrease interval size by one semitone |
| Short press encoder | Reset interval to unison |
| Double-click encoder | Toggle `UP` / `DOWN` direction |
| Hold encoder about `2s` | Open menu |
| In menu, turn encoder | Select `CURVE`, `CAL`, `DIR`, or `DONE` |
| In menu, short press encoder | Change/select the shown item |
| In menu, hold encoder about `2s` | Exit menu |
| Serial `sleep` / `wake` | Manual sleep test only; automatic idle sleep is disabled for bench testing |

Settings autosave about `3s` after the last change.

Curve options:

```text
curve linear     current/default feel
curve easeout    smoother near toe; try this first
curve square     slower early, faster near toe
curve smooth     softer heel/toe, faster middle
```

For live tuning, send `tune on`. In tune mode, turning the encoder adjusts the current interval's toe voltage instead of changing intervals; short press saves and exits.

## Project Layout

```text
docs/
  bom.md
  pico-physical-layout.md
  pico-prototype.md
  product-plan.md
hardware/enclosure/
  generate_enclosure_stl.py
  stl/
pico/PrecisionExpressionControllerPico/
  PrecisionExpressionControllerPico.ino
  PicoFirmwareCore.h
sim/
  pico_console_sim.cpp
test/
  test_pico_core.cpp
```
