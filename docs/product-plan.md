# Product Plan

## Goal

Build a programmable expression controller that lets a musician choose musical bend ranges instead of trimming an expression pedal by percentage.

The current target is a Raspberry Pi Pico H prototype with:

- TRS expression pedal input
- Rotary encoder with push switch
- 128x32 OLED
- MCP4728 quad DAC CV output
- One expression/CV output plus two dedicated 3.5mm LFO outputs
- USB power
- 3D-printed enclosure

## Prototype Scope

Features implemented:

- Pico H ADC read from expression pedal on `GP26/ADC0`.
- Heel/toe calibration from the encoder menu.
- One-direction CV model capped at a major 6th for this bench build. `UP` uses `0.000 V` heel/no-bend to about `3.214 V` toe with the current response fit; `DOWN` uses `3.300 V` heel/no-bend to about `0.086 V` toe.
- Output modes: `PED` expression interval mode, `LO` slow LFO mode, and `FM` faster modulation mode.
- LFO waveforms: sine, triangle, saw up, saw down, square, pulse (adjustable `5-95%` width), sample+hold random, drift (smoothed random).
- LFO `LO` range: `0.05-20 Hz`; LFO `FM` range: `8-160 Hz`.
- LFO depth `0-100%` around the midpoint, plus `-50..+50%` center offset on `LFO1`/`LFO2` (depth `0` + offset acts as a fixed CV source).
- `LFO2` can link to `LFO1` at `1:1`, `1:2`, `1:4`, `3:2`, `2:1`, or `4:1` with a `0/90/180/270` degree phase offset, drift-free.
- Tap tempo (two presses on a focused LFO), per-LFO sync, and `sync all`.
- MCP4728 DAC CV output over I2C on `GP4/SDA` and `GP5/SCL`, using latch-safe multi-write commands that assert `VREF=VDD`/`gain=1` on every update; `dac eeprom` programs silent 0V power-on defaults.
- DAC channel A drives the expression/CV output; channels B and C drive dedicated `LFO1` and `LFO2`; channel D can emit a full-swing clock square at `LFO1`/`LFO2` rate.
- Stored-settings migration: upgrading from the previous settings layout preserves calibration and the tuned toe map.
- 128x32 SSD1306 OLED on the same I2C bus.
- Rotary encoder on `GP14`/`GP15`; push switch on `GP13`.
- Short encoder press resets interval to unison.
- Double-click encoder cycles edit focus: `EXP`, `LFO1`, `LFO2`.
- Encoder hold about `2s` opens an OLED menu.
- OLED menu can choose output focus, change mode, waveform, depth, curve, start calibration, toggle direction/polarity, or close. LFO-only items are hidden when the expression output is in normal `PED` mode.
- Pedal curve modes: linear, easeout, square, smooth.
- Automatic idle sleep is disabled during bench testing to avoid OLED blank/wake flicker.
- Autosave about `3s` after interval/calibration changes.
- Serial UI for setup, calibration, monitoring, and I2C probes.
- Mac-side simulator for OLED/Serial/DAC behavior.

Acceptance criteria:

- Arduino sketch compiles for `rp2040:rp2040:rpipico`.
- Simulator exits with `SIM RESULT: PASS`.
- Serial boot detects MCP4728 at `0x60` and SSD1306 at `0x3C` once wired.
- Pedal reads smoothly from `0.0000` to `1.0000` after calibration.
- `interval 9` measures about `0.000 V` at heel and `3.214 V` at toe.
- `interval -9` measures about `3.300 V` at heel and `0.086 V` at toe.
- Rotary encoder steps exactly one semitone of interval size per detent.
- Short press resets to unison or syncs the focused LFO; double-click cycles edit focus; long press opens the OLED menu.
- In `LO`/`FM`, the expression pedal sweeps LFO speed and the rotary encoder changes LFO depth.
- `LO`/`FM` outputs remain `0-3.3V`, support all six LFO waveforms, and support `50-100%` depth/attenuation.
- Dedicated `LFO1` and `LFO2` outputs can run independently while the expression output stays in `PED` mode.

## Later Product Questions

- Whether MCP4728 resolution is enough musically, or whether a higher-precision DAC is needed.
- Whether a future true audio-rate modulation output needs a faster DAC or PWM/filter output instead of MCP4728 I2C.
- Bench result: the active output reaches about a sixth even when firmware requests more and the output measures about `3.2V`. This build accepts that limit for now and remaps intervals so `6` is accurate.
- Decide whether the prototype should use true active 1V/oct CV on physical Tip/Sleeve or a passive expression-pedal emulator that matches the Nektar NX-P behavior.
- Whether to keep the OLED or switch to a simpler segment display after debug needs are gone.
- Whether to add true hardware power-off instead of firmware sleep.
- Whether to support passive expression emulation or MIDI in a separate product variant.

## Commercialization Checklist

- Reverse-polarity and overvoltage protection.
- ESD protection on external jacks.
- Output current limiting.
- Power-up and brownout behavior verified on scope.
- Factory reset path.
- Firmware update path.
- Calibration procedure written for production.
- Enclosure layout with jack/control spacing validated.
- Clear compatibility matrix by device type and expression standard.
