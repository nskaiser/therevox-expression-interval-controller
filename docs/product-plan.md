# Product Plan

## Goal

Build a programmable expression controller that lets a musician choose musical bend ranges instead of trimming an expression pedal by percentage.

The current target is a Raspberry Pi Pico H prototype with:

- TRS expression pedal input
- Rotary encoder with push switch
- 128x32 OLED
- MCP4725 DAC CV output
- USB power
- 3D-printed enclosure

## Prototype Scope

Features implemented:

- Pico H ADC read from expression pedal on `GP26/ADC0`.
- Heel/toe calibration from the encoder menu.
- One-direction CV model capped at a major 6th for this bench build. `UP` uses `0.000 V` heel/no-bend to about `3.214 V` toe with the current response fit; `DOWN` uses `3.300 V` heel/no-bend to about `0.086 V` toe.
- MCP4725 DAC CV output over I2C on `GP4/SDA` and `GP5/SCL`.
- 128x32 SSD1306 OLED on the same I2C bus.
- Rotary encoder on `GP14`/`GP15`; push switch on `GP13`.
- Short encoder press resets interval to unison.
- Double-click encoder toggles `UP` / `DOWN` direction.
- Encoder hold about `2s` opens an OLED menu.
- OLED menu can change curve, start calibration, toggle direction, or close.
- Pedal curve modes: linear, easeout, square, smooth.
- Automatic idle sleep is disabled during bench testing to avoid OLED blank/wake flicker.
- Autosave about `3s` after interval/calibration changes.
- Serial UI for setup, calibration, monitoring, and I2C probes.
- Mac-side simulator for OLED/Serial/DAC behavior.

Acceptance criteria:

- Arduino sketch compiles for `rp2040:rp2040:rpipico`.
- Simulator exits with `SIM RESULT: PASS`.
- Serial boot detects MCP4725 at `0x62` and SSD1306 at `0x3C` once wired.
- Pedal reads smoothly from `0.0000` to `1.0000` after calibration.
- `interval 9` measures about `0.000 V` at heel and `3.214 V` at toe.
- `interval -9` measures about `3.300 V` at heel and `0.086 V` at toe.
- Rotary encoder steps exactly one semitone of interval size per detent.
- Short press resets to unison; double-click toggles direction; long press opens the OLED menu.

## Later Product Questions

- Whether MCP4725 resolution is enough musically, or whether a higher-precision DAC is needed.
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
