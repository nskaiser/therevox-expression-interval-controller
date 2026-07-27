# Therevox Expression Controller

Turns an expression pedal input into a programmable Therevox modulation source. In `PED` mode, pick a pitch-bend interval with the rotary encoder and the pedal bends smoothly from no-bend to that interval; in `LO`/`FM` modes, the same output jack becomes a selectable-waveform LFO source.

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

LFO modes ignore the pedal interval map and output a unipolar `0-3.3V` modulation signal from the DAC. `LO` covers slow LFO rates (`0.05-20 Hz`); `FM` covers faster modulation rates (`8-160 Hz`). The pedal sweeps LFO speed from slow to fast, and the encoder adjusts LFO depth/attenuation from `50-100%` in `5%` steps.

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
| Turn encoder in `PED` | Change interval size by one semitone |
| Move pedal in `LO`/`FM` | Sweep LFO speed within that mode's range |
| Turn encoder in `LO`/`FM` | Change LFO depth/attenuation in `5%` steps |
| Short press in `PED` | Reset interval to unison |
| Short press in `LO`/`FM` | Reset/sync LFO phase |
| Double-click | Toggle `UP` / `DOWN`; in LFO modes this flips polarity |
| Hold ~2s | Open menu (`MODE`, `WAVE`, `DEPTH`, `CURVE`, `CAL`, `DIR/POL`, `DONE`) |
| In menu: turn / press / hold ~2s | Scroll items / edit or choose / exit |
| Editing `MODE`, `WAVE`, `DEPTH`, or `CURVE` | Turn to the value you want, then press to save |

Settings autosave about 3s after the last change.

PED-mode pedal feel: `curve linear|easeout|square|smooth` over serial.

LFO serial shortcuts:

```text
mode ped|lo|fm
wave sine|tri|sawup|sawdown|square|pulse
rate 1.5      set toe/max speed; pedal sweeps from mode minimum to this
depth 75
sync
```

## Project Layout

```text
docs/                  wiring, layout, BOM, product notes
hardware/enclosure/    STL generator and files (untested)
pico/                  Arduino firmware
sim/                   host-side console simulator
test/                  host-side math tests
```
