# User Manual

This controller has one expression output and two dedicated LFO outputs.

```text
EXP   main 1/4 inch output for Therevox expression/CV control
LFO1  first 3.5mm patch output
LFO2  second 3.5mm patch output
```

For patch-panel ideas using these outputs with the Therevox ET-5, see
`sample-patches.md`.

The encoder edits one output at a time. Double-click the encoder to cycle the
edit target:

```text
EXP -> LFO1 -> LFO2 -> EXP
```

## Outputs

`EXP` has three modes:

| Mode | What It Does |
| --- | --- |
| `PED` | Expression pedal bends from no-bend to the selected musical interval |
| `LO` | Expression output becomes a slow LFO; the pedal controls LFO speed |
| `FM` | Expression output becomes a faster modulation source; the pedal controls speed |

`LFO1` and `LFO2` are always LFO outputs. They each have independent mode,
waveform, speed, depth, and polarity settings.

## Normal Controls

| Action | Result |
| --- | --- |
| Turn encoder with `EXP` focused in `PED` | Change the interval by one semitone |
| Turn encoder with `EXP` focused in `LO` or `FM` | Change expression-output LFO depth |
| Turn encoder with `LFO1` or `LFO2` focused | Change that LFO output speed |
| Move pedal with `EXP` focused in `LO` or `FM` | Sweep expression-output LFO speed |
| Short press with `EXP` focused in `PED` | Reset interval to unison |
| Short press with any LFO focused | Sync that LFO phase |
| Double-click | Cycle edit target: `EXP`, `LFO1`, `LFO2` |
| Hold about 2 seconds | Open or close the OLED menu |

## Main Screen

In normal expression mode:

```text
+3       TOE 1430MV
          RSP 924C
          CRV LIN
UP SAVED
```

| Text | Meaning |
| --- | --- |
| `+3` | Selected interval |
| `TOE 1430MV` | Target voltage when the pedal is toe-down |
| `RSP 924C` | Current response fit in cents across full DAC range |
| `CRV LIN` | Pedal curve |
| `UP SAVED` | Bend direction and saved/dirty state |

In expression LFO mode:

```text
LO        SPD PED
          DEP 100%
          SIN UP
LFO SAVED
```

`SPD PED` means the expression pedal controls the LFO speed. The encoder changes
depth while `EXP` is focused in `LO` or `FM`.

For a dedicated LFO output:

```text
LFO1      SPD 2.50HZ
          DEP 75%
          TRI UP
LFO1 SAVED
```

The encoder changes the shown speed while that LFO is focused.

## Menu

Hold the encoder for about 2 seconds to open the menu. Turn the encoder to
scroll, short press to edit or choose, and hold again to exit.

| Item | Applies To | Meaning |
| --- | --- | --- |
| `OUT` | All modes | Choose the edit target: `EXP`, `LFO1`, or `LFO2` |
| `MODE` | Focused output | Choose `PED`, `LO`, or `FM` for `EXP`; choose `LO` or `FM` for `LFO1`/`LFO2` |
| `WAVE` | LFO modes only | Choose the LFO waveform |
| `DEPTH` | LFO modes only | Set LFO depth from `0%` to `100%` |
| `PW` | Pulse wave only | Set pulse width from `5%` to `95%` |
| `OFS` | `LFO1`/`LFO2` | Shift the LFO center voltage `-50%` to `+50%` |
| `LINK` | `LFO2` only | Lock LFO2's rate to LFO1: `OFF`, `1:1`, `1:2`, `1:4`, `3:2`, `2:1`, `4:1` |
| `PHS` | `LFO2` when linked | Linked phase offset: `0`, `90`, `180`, or `270` degrees |
| `CLK` | All modes | Clock square on the DAC's spare channel: `OFF`, `LFO1`, `LFO2` |
| `CURVE` | `EXP` only | Set expression pedal response curve |
| `CAL` | `EXP` only | Start heel/toe pedal calibration |
| `DIR` | `EXP` in `PED` | Toggle bend direction |
| `POL` | LFO modes | Toggle LFO polarity |
| `DONE` | All modes | Close the menu |

Items only appear when they apply: `WAVE`/`DEPTH` hide while `EXP` is in normal
`PED` mode, `PW` shows only when the focused wave is pulse, `OFS` shows only for
`LFO1`/`LFO2`, `LINK`/`PHS` show only for `LFO2` (and `PHS` only while linked),
and `CURVE`/`CAL` hide while editing `LFO1` or `LFO2`.

## LFO Modes

| Setting | Choices |
| --- | --- |
| Mode | `LO`, `FM` |
| Wave | `SIN`, `TRI`, `SAWUP`, `SAWDN`, `SQR`, `PULS`, `SH`, `DRF` |
| Depth | `0%` through `100%`, in `5%` steps |
| Pulse width | `5%` through `95%` (pulse wave only) |
| Offset | `-50%` through `+50%` (`LFO1`/`LFO2` only) |
| Polarity | `UP`, `DN` |

`LO` covers slow LFO rates from about `0.05Hz` to `20Hz`.

`FM` covers faster rates from about `8Hz` to `160Hz`. The high end is useful for
rough modulation effects, but the MCP4728 is an I2C DAC, not a high-fidelity
audio DAC.

Waveform meanings:

| Wave | Shape |
| --- | --- |
| `SIN` | Smooth sine wave |
| `TRI` | Linear rise and fall |
| `SAWUP` | Rising ramp, then instant reset low |
| `SAWDN` | Falling ramp, then instant reset high |
| `SQR` | 50% square wave |
| `PULS` | Pulse wave, width set by `PW` (default 25%) |
| `SH` | Sample+hold: a new random level each cycle |
| `DRF` | Drift: smooth glide between random levels |

Depth is centered around the middle voltage. At `100%`, the LFO uses the full
`0-3.3V` range. At `75%`, the peaks are 75% as far above and below the midpoint.
At `0%`, the output sits flat at the midpoint — combine depth `0` with `OFS` to
use a dedicated LFO jack as a fixed CV source.

`OFS` shifts the whole waveform up or down after depth is applied, clamped to
the `0-3.3V` rails.

## Linking LFO2 To LFO1

`LINK` locks LFO2's rate to a ratio of LFO1's, computed from LFO1's phase so the
pair can never drift apart. `1:2` runs LFO2 at half LFO1's speed; `3:2` gives a
polyrhythm; `1:1` with `PHS 90` gives a quadrature pair for stereo-style
modulation.

While linked, LFO2's own speed setting is ignored, and turning the encoder with
`LFO2` focused steps the phase offset instead of the rate.

## Clock Output

`CLK` drives the MCP4728's spare fourth channel (`D`) with a full-swing
`0/3.3V` square at LFO1's or LFO2's rate, ignoring that LFO's wave, depth, and
offset. Wire `D` through a 1k resistor to a spare 3.5mm jack to clock
external gear in time with your modulation.

## Sync And Tap Tempo

Short-pressing the encoder while an LFO is focused syncs the selected LFO phase.
That means the waveform restarts at the beginning of its cycle.

Examples:

| Wave | What Sync Does |
| --- | --- |
| `SAWUP` | Jump to low and start rising |
| `SAWDN` | Jump to high and start falling |
| `SQR` | Restart at the first half of the square wave |
| `SIN` | Restart at the midpoint and rise |

Sync does not change depth, waveform, polarity, or output focus.

Two short presses in a row also **tap tempo**: if the gap between presses is
between about half a second and five seconds, the focused LFO's rate is set to
the tapped interval (the OLED shows `TAP` instead of `SYNC`). Faster tempos than
that collide with the double-click gesture — use serial `tap` or `rate <hz>`
for those.

Serial `sync all` restarts every LFO phase together — useful to re-align a
patch after the free-running LFOs have wandered apart.

## Pedal Calibration

Calibration teaches the Pico the heel and toe raw values from your expression
pedal.

1. Focus `EXP`.
2. Hold the encoder for about 2 seconds to open the menu.
3. Turn to `CAL`.
4. Short press. The OLED says `HEEL`.
5. Put the pedal fully heel-down.
6. Short press. The OLED says `TOE`.
7. Put the pedal fully toe-down.
8. Short press again. The OLED says `SAVED`.

If the pedal moves backward after calibration, use Serial Monitor:

```text
invert on
save
```

## Direction And Polarity

For `EXP` in `PED` mode:

```text
UP    heel/no-bend is 0V; toe bends upward
DOWN  heel/no-bend is 3.3V; toe bends downward
```

For LFO modes:

```text
UP    normal waveform
DN    inverted waveform
```

## Autosave

Settings autosave about 3 seconds after the last change. The firmware also saves
immediately after menu edits and calibration.

Some Pico board cores do not provide EEPROM persistence. If Serial Monitor says
settings were kept in RAM only, keep a copy of your important `response`, `map`,
and tuning commands.
