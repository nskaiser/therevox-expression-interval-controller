# Therevox Expression Interval Controller

Turns an expression pedal into a precise pitch-bend interval controller for the Therevox. Pick an interval with the rotary encoder (semitone up to an octave, in either direction), and the pedal bends smoothly from no-bend to exactly that interval.

Built with:

- Raspberry Pi Pico H
- MCP4725 I2C DAC
- 0.91 inch 128x32 I2C OLED
- Rotary encoder with push switch
- Two TRS jacks: expression pedal in, CV out
- USB power

## Start Here

| Need | File |
| --- | --- |
| Arduino firmware | `pico/PrecisionExpressionControllerPico/PrecisionExpressionControllerPico.ino` |
| Wiring and bench test | `docs/pico-prototype.md` |
| Perfboard/enclosure layout | `docs/pico-physical-layout.md` |
| BOM | `docs/bom.md` |
| Console simulator | `sim/pico_console_sim.cpp` |
| 3D-print enclosure files (untested, never printed) | `hardware/enclosure/` |

## How It Works

One bend direction is active at a time; double-click the encoder to toggle `UP`/`DOWN`.

```text
UP:   heel 0 V, toe rises to the selected interval's voltage
DOWN: heel 3.3 V, toe falls to the selected interval's voltage
```

Tune the instrument with the pedal at heel.

Interval accuracy comes from a per-interval voltage map. Set the overall scale with `response <cents>` (the pitch change your instrument produces at full DAC scale — `3960` for a true 1V/oct input), then fine-tune any interval against a tuner with `tune on`. See `docs/pico-prototype.md` for the full serial command reference.

Note the DAC output tops out at 3.3 V. If your instrument needs more than that for an octave, the largest selectable interval shrinks accordingly.

## Build And Upload

1. In Arduino IDE, install the board package `Raspberry Pi Pico/RP2040/RP2350` by Earle F. Philhower.
2. Open `pico/PrecisionExpressionControllerPico/PrecisionExpressionControllerPico.ino`.
3. Select `Tools > Board > Raspberry Pi Pico/RP2040/RP2350 > Raspberry Pi Pico`.
4. Plug in the Pico over USB, select its port, and upload.
5. Open Serial Monitor at `115200`.

If the Pico does not show as a port, unplug it, hold `BOOTSEL`, plug USB back in, release `BOOTSEL`, then upload again.

## Tests

```bash
# Host math test
g++ -std=c++17 test/test_pico_core.cpp -o /tmp/expctrl_test && /tmp/expctrl_test

# Console simulator (expects SIM RESULT: PASS)
g++ -std=c++17 sim/pico_console_sim.cpp -o /tmp/expctrl_sim && /tmp/expctrl_sim

# Firmware compile check
arduino-cli compile --fqbn rp2040:rp2040:rpipico pico/PrecisionExpressionControllerPico
```

## Controls

| Action | Result |
| --- | --- |
| Turn encoder | Change interval size by one semitone |
| Short press | Reset interval to unison |
| Double-click | Toggle `UP` / `DOWN` direction |
| Hold ~2s | Open menu (`CURVE`, `CAL`, `DIR`, `DONE`) |
| In menu: turn / press / hold ~2s | Select / choose / exit |

Settings autosave about 3s after the last change.

Pedal feel: `curve linear|easeout|square|smooth` over serial.

## Project Layout

```text
docs/                  wiring, layout, BOM, product notes
hardware/enclosure/    STL generator and files (untested)
pico/                  Arduino firmware
sim/                   host-side console simulator
test/                  host-side math tests
```
