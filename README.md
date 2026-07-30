# Therevox Expression Controller

Turns an expression pedal input into a programmable Therevox modulation source. In `PED` mode, pick a pitch-bend interval with the rotary encoder and the pedal bends smoothly from no-bend to that interval; in `LO`/`FM` modes, the expression output can become an LFO source. The MCP4728 build also adds two independent 3.5mm LFO outputs for the Therevox patch panel.

Built with:

- Raspberry Pi Pico H
- MCP4728 quad I2C DAC
- 0.91 inch 128x32 I2C OLED
- Rotary encoder with push switch
- Two 1/4 inch TRS jacks: expression pedal in, Therevox expression/CV out
- Two 3.5mm TS jacks: independent LFO 1 and LFO 2 outputs
- USB power

## Start Here

| Need | File |
| --- | --- |
| Arduino firmware | `pico/PrecisionExpressionControllerPico/PrecisionExpressionControllerPico.ino` |
| User/menu manual | `docs/user-manual.md` |
| Therevox sample patches | `docs/sample-patches.md` |
| Wiring and bench test | `docs/pico-prototype.md` |
| Perfboard/enclosure layout | `docs/pico-physical-layout.md` |
| BOM | `docs/bom.md` |
| Console simulator | `sim/pico_console_sim.cpp` |
| 3D-print enclosure files (untested, never printed) | `hardware/enclosure/` |

## How It Works

One bend direction is active at a time for the expression output. Use the menu or serial `direction up|down` to change it.

```text
UP:   heel 0 V, toe rises to the selected interval's voltage
DOWN: heel 3.3 V, toe falls to the selected interval's voltage
```

Tune the instrument with the pedal at heel.

Interval accuracy comes from a per-interval voltage map. Set the overall scale with `response <cents>` (the pitch change your instrument produces at full DAC scale; `3960` for a true 1V/oct input), then fine-tune any interval against a tuner with `tune on`. See `docs/pico-prototype.md` for the full serial command reference.

Note the DAC output tops out at 3.3 V. If your instrument needs more than that for an octave, the largest selectable interval shrinks accordingly.

LFO modes ignore the pedal interval map and output a unipolar `0-3.3V` modulation signal from the DAC. `LO` covers slow LFO rates (`0.05-20 Hz`); `FM` covers faster modulation rates (`8-160 Hz`). On the expression output, the pedal sweeps LFO speed from slow to fast and the encoder adjusts depth. On `LFO1`/`LFO2`, the encoder adjusts speed directly.

LFO features:

- Eight waveforms: sine, triangle, saw up, saw down, square, pulse (adjustable width `5-95%`), sample+hold random, and drift (smoothed random).
- Depth `0-100%` around the midpoint, plus a `-50..+50%` center offset on `LFO1`/`LFO2` (depth `0` + offset = fixed CV).
- `LFO2` can lock to `LFO1` at a rate ratio (`1:1`, `1:2`, `1:4`, `3:2`, `2:1`, `4:1`) with a `0/90/180/270` degree phase offset — quadrature and polyrhythm pairs that never drift.
- Tap tempo: two short presses on a focused LFO sync it and set its rate to the tapped interval.
- `sync` / `sync all` phase resets, per-output polarity.
- The DAC's fourth channel can emit a full-swing clock square at `LFO1` or `LFO2`'s rate (`clock lfo1|lfo2`).

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

## Control Summary

| Action | Result |
| --- | --- |
| Turn encoder with `EXP` focused in `PED` | Change interval size by one semitone |
| Move pedal in expression `LO`/`FM` | Sweep expression-output LFO speed within that mode's range |
| Turn encoder with `EXP` focused in `LO`/`FM` | Change expression-output LFO depth/attenuation in `5%` steps |
| Turn encoder with `LFO1` or `LFO2` focused | Change that dedicated LFO output speed (or phase offset when `LFO2` is linked) |
| Short press in `PED` | Reset interval to unison |
| Short press in `LO`/`FM`, `LFO1`, or `LFO2` | Sync the focused LFO; two presses at tempo tap its rate |
| Double-click | Cycle edit focus: `EXP` -> `LFO1` -> `LFO2` |
| Hold ~2s | Open menu |
| In menu: turn / press / hold ~2s | Scroll items / edit or choose / exit |
| Editing a menu value | Turn to the value you want, then press to save |

Settings autosave about 3s after the last change.

See `docs/user-manual.md` for the full OLED menu reference.

LFO serial shortcuts:

```text
mode ped|lo|fm
focus exp|lfo1|lfo2
wave sine|tri|sawup|sawdown|square|pulse|sh|drift
rate 1.5      set focused LFO speed; EXP LFO uses this as toe/max speed
depth 75      set focused LFO depth, 0..100
pw 60         pulse-wave width, 5..95
offset -25    shift focused LFO1/LFO2 center, -50..50
link 1:2      lock LFO2 rate to LFO1; link phase 90 for quadrature
clock lfo1    full-swing clock square on the DAC's spare channel
sync          sync focused LFO; sync all resets every phase
tap           send twice at tempo to set the focused LFO rate
```

## Project Layout

```text
docs/                  wiring, layout, BOM, product notes
hardware/enclosure/    STL generator and files (untested)
pico/                  Arduino firmware
sim/                   host-side console simulator
test/                  host-side math tests
```
