# Pico H Prototype

This is the cheaper soldered prototype build: Raspberry Pi Pico H, MCP4728 quad DAC, 128x32 OLED, one rotary encoder with push, two 1/4 inch TRS jacks, and two 3.5mm dedicated LFO output jacks.

## Important Voltage Model

This firmware uses one bend direction at a time for the expression output. That gives the active direction the full MCP4728 channel-A output range.

```text
UP mode:
  heel/no-bend: 0.000 V
  toe major 6th: 3.214 V with the current response fit

DOWN mode:
  heel/no-bend: 3.300 V
  toe major 6th: 0.086 V with the current response fit
```

Tune the Therevox with the expression pedal heel-down in the active direction. Use the OLED menu `DIR/POL` item or serial `direction up|down` to change `UP`/`DOWN`.

Important hardware limitation: the current output is an active `0-3.3V` DAC signal. For a standard Therevox external-CV test, that voltage should be on the physical plug `Tip` relative to physical plug `Sleeve`. If your jack breakout only bends when you swap its `Tip` and `Ring` lugs, verify with a meter at the actual plug contacts; do not rely only on the breakout silkscreen.

## Parts

| Item | Qty | Notes |
| --- | ---: | --- |
| Raspberry Pi Pico H | 1 | Adafruit PID 5525 |
| MCP4728 quad DAC breakout | 1 | DIYmall GY-MCP4728 or equivalent, I2C address `0x60` |
| Adafruit 0.91 inch 128x32 I2C OLED | 1 | PID 4440, I2C address `0x3C` |
| STEMMA QT / Qwiic JST-SH to male headers cable | 1 | PID 4209, for OLED |
| Rotary Encoder + Extras | 1 | Adafruit PID 377 |
| 1/4 inch TRS jack | 2 | Expression pedal input and Therevox expression/CV output |
| 3.5mm TS or TRS jack | 2 | Dedicated `LFO1` and `LFO2` patch-panel outputs |
| `1k` resistor | 4 | One pedal input series resistor, plus one series resistor per DAC output |
| `100nF` capacitor | 1 | Pedal input noise filter |
| Perfboard | 1 | Better than breadboard for Therevox testing |
| Hookup wire | as needed | `22-24 AWG` solid or stranded |
| USB micro cable / USB power bank | 1 | Do not feed 9V into USB |

## Pico Pin Wiring

Use the Pico H labels printed on the board.

For exact perfboard row/column placement that matches the printed enclosure, use `docs/pico-physical-layout.md`.

### Expression Pedal Input TRS Jack

| Jack lug | Pico H pin | Notes |
| --- | --- | --- |
| Tip | `GP26/ADC0`, physical pin `31` | Connect through a `1k` resistor |
| Ring | `3V3(OUT)`, physical pin `36` | Pedal reference voltage |
| Sleeve | `GND`, physical pin `38` | Common ground |

```text
Input TRS Tip ---- 1k ---- Pico GP26/ADC0
                           |
                         100nF
                           |
Input TRS Sleeve --------- GND

Input TRS Ring ----------- Pico 3V3(OUT)
```

### I2C Bus Shared by DAC and OLED

The OLED STEMMA cable and the MCP4728 header pins connect to the same Pico I2C pins.

| Signal / OLED STEMMA wire | Pico H pin |
| --- | --- |
| Red / VCC | `3V3(OUT)`, physical pin `36` |
| Black / GND | `GND`, physical pin `38` |
| Blue / SDA | `GP4`, physical pin `6` |
| Yellow / SCL | `GP5`, physical pin `7` |

On the MCP4728 board, use the pins labeled `VCC`, `GND`, `SDA`, and `SCL`.

**Also wire the MCP4728 `LDAC` pin to `GND`.** If `LDAC` floats high, the chip
accepts every I2C write but can hold the analog outputs frozen — the firmware
uses latch-safe multi-write commands as a backstop, but grounding `LDAC` removes
the failure mode entirely.

This is normal I2C sharing. The MCP4728 DAC is `0x60`; the OLED is `0x3C`.

### MCP4728 DAC Outputs

The I2C cable/header wiring does not carry the DAC outputs. Add regular hookup wires from the DAC `VOUT` pins to the output jacks:

```text
MCP4728 VOUTA ---- 1k ---- Therevox expression output plug physical Tip
Output TRS Sleeve -------- GND
Output plug physical Ring - not connected

MCP4728 VOUTB ---- 1k ---- LFO1 3.5mm jack Tip
LFO1 Sleeve --------------- GND

MCP4728 VOUTC ---- 1k ---- LFO2 3.5mm jack Tip
LFO2 Sleeve --------------- GND

MCP4728 VOUTD ---- 1k ---- optional clock 3.5mm jack Tip (see `clock` command)
Clock jack Sleeve --------- GND
Leave VOUTD unconnected if you do not need the clock output.

Do not use the old optional `100k` pulldown or any temporary `10k` to `11k`
load resistor on any output jack for this active-CV test.
```

If the 3.5mm jacks are TRS instead of TS, use `Tip` and `Sleeve` only. Leave `Ring` unconnected.

Your Nektar NX-P resistance measurements showed:

```text
Tip-Sleeve: about 11k at heel and toe
Ring-Sleeve: about 0.574k to 11.6k
Tip-Ring: about 11.6k to 0.578k
```

That means `Tip` and `Sleeve` look like the fixed pot ends, and `Ring` looks like the wiper when the NX-P is measured by itself. For active external CV, the Therevox manual identifies the expression jack as `Tip = CV`, `Ring = Reference`, `Sleeve = Ground`. So the active DAC voltage should land on the physical `Tip`.

If swapping `Tip` and `Ring` on your jack breakout is the only way to get bending, that likely means the breakout lug naming, cable, or jack contact path is not what we assumed. The meter test that matters is at the actual plug inserted into the Therevox: `cv 1000` should put about `1.000V` on physical `Tip` to physical `Sleeve`.

### Nektar NX-P Pedal Notes

The Nektar NX-P has a fixed TRS cable, a polarity switch, and a sensitivity pot. Do all measurements with the polarity switch and sensitivity knob in the exact positions that give a clean octave when plugged directly into the Therevox.

With the NX-P unplugged, measure resistance at the pedal plug:

```text
Tip-Sleeve, heel and toe
Ring-Sleeve, heel and toe
Tip-Ring, heel and toe
```

The pair that stays roughly constant is the total pedal pot/load. The two measurements that change identify the wiper. For your measured NX-P setup, `Ring` is the wiper.

### Rotary Encoder

The PEC11 encoder has three different kinds of metal connections:

```text
3 small pins on one side      = encoder rotation pins: A, C, B
2 small pins on opposite side = pushbutton switch
1 large lug on each side      = mechanical mounting tabs
```

Looking at the 3-pin side, the middle pin is `C` / common. The two outside pins are `A` and `B`.

| Encoder lug | Pico H pin |
| --- | --- |
| `C` / common | `GND` |
| `A` / CLK | `GP14`, physical pin `19` |
| `B` / DT | `GP15`, physical pin `20` |
| Switch lug | `GP13`, physical pin `17` |
| Other switch lug | `GND` |
| Large side lugs | no electrical connection |

No resistors are needed. The firmware uses internal pullups. If turning clockwise moves down instead of up, swap encoder `A` and `B`.

## Firmware

Open this sketch in Arduino IDE:

```text
pico/PrecisionExpressionControllerPico/PrecisionExpressionControllerPico.ino
```

Install/select:

```text
Board package: Raspberry Pi Pico/RP2040/RP2350 by Earle F. Philhower
Board: Raspberry Pi Pico
Port: the Pico serial port
Serial Monitor: 115200 baud
```

The local smoke compile passed with:

```text
arduino-cli compile --fqbn rp2040:rp2040:rpipico pico/PrecisionExpressionControllerPico
```

Flashing over the previous release keeps your pedal calibration and tuned toe
map: the firmware migrates stored settings from the last layout (version 11)
and only falls back to defaults from anything older. New settings (pulse width,
offset, link, clock) start at their defaults after migration.

## Bench Smoke Test

Do this before connecting the Therevox.

Before the hardware arrives, run the Mac-side simulator:

```text
g++ -std=c++17 sim/pico_console_sim.cpp -o /tmp/expctrl_pico_console_sim
/tmp/expctrl_pico_console_sim
```

It prints simulated Serial Monitor output, OLED text, DAC millivolts, and DAC codes. It exits with `SIM RESULT: PASS` only after verifying one-direction UP/DOWN rails, MCP4728-style `EXP`/`LFO1`/`LFO2` DAC state, encoder steps, calibration, autosave, and bench idle-sleep disabled.

1. Power the Pico from USB.
2. Open Serial Monitor at `115200`.
3. Confirm boot says:

```text
MCP4728 PASS at 0x60
SSD1306 PASS at 0x3C
```

4. Put a multimeter black probe on the Therevox output plug physical Sleeve and red probe on the output plug physical Tip.
5. Send `interval 9`; heel should be about `0.000V`, toe about `3.214V`.
6. Send `interval -9`; heel should be about `3.300V`, toe about `0.086V`.
7. Send `center`; output should stay at the active direction's heel/no-bend rail.
8. Move the red probe to the `LFO1` 3.5mm Tip, with black on Sleeve. You should see voltage moving between about `0V` and `3.3V`.
9. Move the red probe to the `LFO2` 3.5mm Tip. You should also see an independent moving voltage.

Voltage tuning commands:

```text
direction up     heel/no-bend 0.000V, toe bends upward
direction down   heel/no-bend 3.300V, toe bends downward
range 3300       old linear voltage map; disables response fit
mode ped         expression pedal interval mode
mode lo          slow LFO mode, 0.05-20Hz
mode fm          fast LFO/FM mode, 8-160Hz
focus exp        encoder/menu edits the expression output
focus lfo1       encoder/menu edits dedicated LFO1
focus lfo2       encoder/menu edits dedicated LFO2
wave sine        LFO waveform: sine, tri, sawup, sawdown, square, pulse, sh, drift
rate 1.5         set focused LFO speed; EXP LFO uses this as toe/max speed
depth 75         set focused LFO depth/attenuation, 0-100% in 5% steps
pw 60            pulse-wave width, 5-95%
offset -25       shift focused LFO1/LFO2 center voltage, -50..50%
link 1:2         lock LFO2 rate to LFO1; also link phase 0|90|180|270
clock lfo1       full-swing clock square on VOUTD; clock off disables
sync             reset focused LFO phase; sync all resets every LFO
tap              send twice at tempo to set the focused LFO rate
response 924     compressed bench response; rebuilds the global map
dac eeprom       program 0V power-on defaults into the MCP4728 (run once)
save             persist the settings
```

Run `dac eeprom` once per MCP4728 board. It stores 0V power-on values in the
DAC's own EEPROM so the jacks stay silent between power-on and firmware boot.

The default is `direction up` and `response 924`, based on your measured compressed active-DAC response. Selectable intervals now stop at `interval 9`, labeled `6` for a major 6th. `response 3960` means standard `1V/oct` behavior because full DAC scale is about `3.3V`, or `3960` cents. `response off` returns to the older linear voltage map.

If all intervals are proportionally small or large, adjust `response`. This build intentionally stops at `6` because your current active path reaches roughly that range.

For precise tuning, start with the global response map. This is one scale factor, not individual note tuning.

```text
response 924      full DAC scale measured as 924 cents; current bench default
response 3960     standard 1V/oct scale for later wiring diagnosis
response off      disable response fit and use old linear voltage range
map show          print all interval toe voltages
map reset         rebuild the map from response/range
map 2 2050        set major 2nd toe voltage to 2.050V
map -2 2750       set downward major 2nd toe voltage to 2.750V
toe 2050          set toe voltage for the currently selected interval
nudge 25          raise current interval toe voltage by 25mV
nudge -25         lower current interval toe voltage by 25mV
tune on           encoder adjusts current interval toe voltage
tune step 25      set encoder tuning step to 25mV
cv 1500           force fixed DAC output for voltage/range testing
cv off            return to normal pedal control
```

Suggested tuner workflow:

1. Send `direction up`, put the pedal heel-down, and tune the Therevox to unison.
2. Send `response 924`, then test `b2`, `2`, `b3`, and `3` with the pedal toe-down.
3. If all intervals are still proportionally flat, lower `response`; if they are sharp, raise `response`.
4. Send `save`.
5. Send `map show` and keep the printed commands. On board cores that print `settings kept in RAM`, `save` does not survive unplugging.

### Hard Therevox Range Test

Do this when intervals are way off, such as `+b3` only moving about one semitone.

This bypasses the expression pedal, pedal calibration, and interval map. It proves whether the active DAC is reaching the Therevox's actual 1V/oct CV input.

1. Connect the output jack to the Therevox.
2. Open Serial Monitor at `115200`.
3. Send:

```text
cv 0
```

4. Tune the Therevox to your base note.
5. Send:

```text
cv 1000
```

6. Read the tuner.

If physical plug `Tip` really carries `1.000V` above physical `Sleeve`, `cv 1000` should be one octave up on a 1V/oct input. If it is not, the output is not reaching the actual CV input path.

Then send:

```text
cv 3300
```

If `cv 3300` is only around a sixth above `cv 0`, the active DAC is not connected to a plain 1V/oct CV input. Re-check the physical plug `Tip`/`Sleeve`, jack lugs `T` versus `TN`, and the Therevox Exp 1 function setting.

When done:

```text
cv off
```

If `cv 1000` is a clean octave, then use `response 3960` for normal 1V/oct behavior. If the active output still behaves like the compressed measurements you sent, use `response 924` while we fix the wiring/output path.

### Live Interval Tune Mode

Use this after the hard range test.

Example for `+b3`:

```text
interval 3
tune step 25
tune on
```

Then:

1. Hold the expression pedal fully toe-down.
2. Watch a tuner.
3. Turn the encoder clockwise for a larger musical interval, counterclockwise for a smaller one. In DOWN mode this lowers the toe voltage.
4. Stop when the tuner says the interval is correct.
5. Short-press the encoder to save and exit tune mode.

For fine tuning:

```text
tune step 5
tune on
```

If you hit `TOE 3300MV` in UP mode and the interval is still flat, that interval is beyond the current hardware's upward voltage range. In DOWN mode, the equivalent lower limit is `TOE 0MV`.

### OLED Main Screen

The normal OLED screen is intentionally stable. It does not show live pedal percent
or live output voltage, because tiny ADC changes can otherwise redraw the OLED and
look like flicker.

```text
6       TOE 3214MV
        RSP 924C
        CRV LIN
UP SAVED
```

Use Serial Monitor `status` or `monitor on` for live `raw`, `ped`, and output
millivolts while debugging.

### OLED Shows Or Flickers `WAKE`

Current bench firmware disables automatic idle sleep. If you still see `WAKE`
without manually sending `sleep`, you are running an older build. Upload the
current sketch again.

In older builds, `WAKE` meant the firmware had gone into sleep and then woke up.
During bench wiring, that usually meant one of these:

- Encoder switch pin `GP13`, physical pin `17`, is accidentally touching `GND`.
- The encoder switch wiring is noisy or on the wrong pins.
- The expression input `GP26/ADC0` is floating because the expression pedal is not plugged in yet.

Quick test:

1. Unplug power.
2. Remove the wire from Pico `GP13`, physical pin `17`.
3. Power back up.
4. Open Serial Monitor at `115200` and send `status`.

Expected button state:

```text
encBtn13=open
```

If it says:

```text
encBtn13=PRESSED
```

then `GP13` is being held to `GND`; fix the encoder switch wiring before continuing.

For a bench test without the expression pedal plugged in, temporarily connect the input node on `GP26/ADC0` to `GND` through the normal `1k` input resistor path so it does not float.

### Rotary Encoder Does Nothing

The encoder rotation pins are only the three small pins on one side of the encoder. The middle pin is `C` / common, and it must go to `GND`.

Correct rotation wiring:

```text
Encoder C/common  -> Pico GND
Encoder A/outside -> Pico GP14, physical pin 19
Encoder B/outside -> Pico GP15, physical pin 20
```

The two small pins on the opposite side are only the pushbutton. They do not detect rotation.

Serial test:

1. Open Serial Monitor at `115200`.
2. Send `enc debug on`.
3. Turn the encoder slowly.

Expected when the Pico sees rotation:

```text
ENC transitions=...
OK interval ...
```

If there are no `ENC transitions` lines while turning, the Pico is not seeing the encoder pins change. Check that `C` is on `GND`, not `3V3`, and that the two outside rotation pins go to `GP14` and `GP15`. If the interval moves the wrong direction, swap `A` and `B`.

Button hold test:

1. Open Serial Monitor at `115200`.
2. Send `btn debug on`.
3. Press and hold the encoder for about `2s`.

Expected:

```text
BTN press
BTN long -> menu open
OK menu open; turn encoder to select, short press to choose, hold to exit
```

If you see `BTN press` and then `BTN release` before two seconds even though you are still holding the encoder down, the switch wiring is loose or bouncing open. If you see no `BTN press`, the switch pin is not reaching Pico `GP13`, physical pin `17`.

If the OLED shows unreadable or flickering text and Serial finds `0x3C` (`display probe`), suspect a loose STEMMA connector, weak ground, weak 3.3V connection, or a bad OLED module. If Serial does not find `0x3C`, the wiring is wrong or the OLED is not powered.

## Controls

For the full OLED menu manual, see `user-manual.md`.

| Action | Result |
| --- | --- |
| Double-click encoder | Cycle edit focus: `EXP` -> `LFO1` -> `LFO2` |
| Turn encoder with `EXP` focused in `PED` | Change interval size by one semitone, up to major 6th |
| Move pedal with `EXP` focused in `LO`/`FM` | Sweep expression-output LFO speed within that mode's range |
| Turn encoder with `EXP` focused in `LO`/`FM` | Change expression-output LFO depth/attenuation in `5%` steps |
| Turn encoder with `LFO1` or `LFO2` focused | Change that dedicated LFO's speed (or phase offset when `LFO2` is linked) |
| Short press encoder in `PED` | Reset interval to unison |
| Short press encoder in `LO`/`FM`, `LFO1`, or `LFO2` | Sync the focused LFO; two presses at tempo tap its rate |
| Hold encoder about `2s` | Open menu |
| In menu, turn encoder | Scroll the visible menu items |
| In menu, short press encoder | Open the shown setting, run `CAL`, toggle `DIR/POL`, or exit on `DONE` |
| Editing a menu value | Turn to the value you want, then short press to save |
| In menu, hold encoder about `2s` | Exit menu |
| Serial `sleep` / `wake` | Manual sleep test only; automatic idle sleep is disabled for bench testing |

Settings autosave about `3s` after the last change.

PED-mode curve choices:

```text
curve linear     current/default feel
curve easeout    smoother near toe; try this first if the bend jumps late
curve square     slower early, faster near toe
curve smooth     softer heel/toe, faster middle
```

LFO modes:

```text
focus exp        edit the expression output
focus lfo1       edit dedicated LFO1 output
focus lfo2       edit dedicated LFO2 output
mode lo          slow LFO, 0.05-20Hz
mode fm          faster modulation, 8-160Hz
mode ped         return to expression-pedal interval mode
wave sine        smooth sine LFO
wave tri         triangle LFO
wave sawup       rising sawtooth
wave sawdown     falling sawtooth
wave square      50% square wave
wave pulse       pulse wave; width set with pw 5..95, default 25%
wave sh          sample+hold: a new random level each cycle
wave drift       smooth glide between random levels
rate 2.5         set focused LFO speed; EXP LFO uses this as toe/max speed
depth 75         set focused LFO depth/attenuation; 0 pins the output at center
pw 60            pulse-wave width for the focused LFO
offset -25       shift focused LFO1/LFO2 center voltage
link 1:2         lock LFO2 rate to half of LFO1's; link off to free-run
link phase 90    linked LFO2 quarter-cycle offset (0|90|180|270)
clock lfo1       clock square on VOUTD at LFO1's rate; clock off disables
sync             reset focused LFO phase; sync all resets every LFO
tap              send twice at tempo to set the focused LFO rate
polarity down    invert focused LFO1/LFO2
```

All DAC outputs are unipolar `0-3.3V`. `UP` polarity is normal; `DOWN`
polarity inverts the LFO. Depth is bipolar around the midpoint: at `75%`, a sine
peak is `75%` as far above and below the midpoint as it is at `100%`, and at
`0%` the output sits flat at the midpoint (add `offset` for a fixed CV). The
top of `FM` mode is intentionally rough because the MCP4728 is an I2C DAC, not
a true audio DAC.

Linked LFO2 (`link` other than `off`) derives its phase from LFO1, so ratio
pairs like `1:2` or quadrature (`1:1` + `link phase 90`) stay locked forever;
while linked, LFO2's own `rate` is ignored and the encoder steps the phase
offset instead.

## Pedal Calibration

With the encoder:

1. Hold encoder about `2s` to open the menu.
2. Turn to `CAL`.
3. Short press encoder; the OLED says `HEEL`.
4. Put the expression pedal fully heel-down.
5. Short press encoder.
6. Put the expression pedal fully toe-down.
7. Short press encoder again; calibration is saved.

With Serial Monitor:

```text
cal start
```

Put the expression pedal fully heel-down, then send:

```text
cal heel
```

Put the expression pedal fully toe-down, then send:

```text
cal toe
```

To throw away calibration and return to the full ADC range:

```text
cal reset
```

If pedal direction is backward, send:

```text
invert on
save
```
