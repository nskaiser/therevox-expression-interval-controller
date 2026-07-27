# Pico Physical Layout

This layout is meant to match the generated enclosure in `hardware/enclosure/stl/`.

Use perfboard for the enclosure build. A solderless breadboard is fine for a bench test, but it will not fit cleanly in the printed box and the side jacks/lid controls will pull wires loose.

## Enclosure Orientation

Hold the box so the rear USB opening faces away from you.

```text
Rear / USB side
┌──────────────────────────────────────────────┐
│                 USB opening                  │
│                                              │
│              rotary encoder hole             │
│                                              │
│  expression input TRS      Therevox output   │
│  jack on left side         TRS jack on right │
│                                              │
│                OLED window                   │
└──────────────────────────────────────────────┘
Front / player side
```

The generated enclosure dimensions are:

| Feature | Position |
| --- | --- |
| Body outside | `126mm x 82mm x 34mm` |
| OLED window | lid centerline, front half, `30mm x 11mm` |
| Encoder hole | lid centerline, rear half, `7.4mm` diameter |
| Expression input jack | left side wall, centered front/back |
| Therevox output jack | right side wall, centered front/back |
| USB/service opening | rear wall, centered left/right |

## Perfboard Size And Placement

Use a `0.1 inch / 2.54mm` perfboard cut to about:

```text
30 holes wide x 24 holes deep
about 76mm x 61mm
```

Place it on the enclosure floor:

- Centered left/right.
- Rear edge about `4mm` in front of the rear wall.
- Pico USB end facing the rear USB opening.
- Use `6mm` to `10mm` standoffs, or printed posts plus nylon screws.

Do not mount the OLED or encoder to the perfboard. They mount to the lid/panel and connect with flexible hookup wires.

## Perfboard Coordinate System

View the perfboard from above with the USB opening at the top.

```text
Rear / USB side

columns:  1  2  3 ... 30
rows:
  1
  2
  3
 ...
 24

Front / player side
```

The plan below assumes:

- Row `1` is nearest the rear USB opening.
- Row `24` is nearest the front/player side.
- Column `1` is nearest the expression input jack.
- Column `30` is nearest the Therevox output jack.

## Pico Placement

Put the Pico H on the perfboard first.

```text
Pico USB connector points toward row 1 / rear USB opening.

Pico physical pin 1  -> row 2,  column 11
Pico physical pin 20 -> row 21, column 11

Pico physical pin 40 -> row 2,  column 18
Pico physical pin 21 -> row 21, column 18
```

That puts the Pico's two header rows `7` columns apart, which matches the Pico's `0.7 inch` row spacing on 0.1 inch perfboard.

Useful Pico pins in this layout:

| Signal | Pico label | Physical pin | Perfboard hole |
| --- | --- | ---: | --- |
| `SDA` | `GP4` | `6` | row `7`, column `11` |
| `SCL` | `GP5` | `7` | row `8`, column `11` |
| Encoder switch | `GP13` | `17` | row `18`, column `11` |
| Encoder A / CLK | `GP14` | `19` | row `20`, column `11` |
| Encoder B / DT | `GP15` | `20` | row `21`, column `11` |
| Pedal ADC | `GP26/ADC0` | `31` | row `11`, column `18` |
| Local ADC ground | `GND` | `33` | row `9`, column `18` |
| 3.3V rail feed | `3V3(OUT)` | `36` | row `6`, column `18` |
| Main ground rail feed | `GND` | `38` | row `4`, column `18` |

## Power Rails On Perfboard

Perfboard holes are not connected unless you solder them.

On the underside, make two bare-wire buses:

| Bus | Perfboard location |
| --- | --- |
| `3V3` | row `23`, columns `2` through `29` |
| `GND` | row `24`, columns `2` through `29` |

Then wire:

| From | To |
| --- | --- |
| Pico `3V3(OUT)`, physical pin `36`, row `6` column `18` | `3V3` bus, row `23` |
| Pico `GND`, physical pin `38`, row `4` column `18` | `GND` bus, row `24` |
| Pico `GND`, physical pin `33`, row `9` column `18` | `GND` bus, row `24` |

## I2C Fanout For OLED And DAC

Make a small fanout area on the left rear of the perfboard. This gives you easy solder points for both STEMMA cables.

| Fanout signal | Perfboard tie point | Connects to |
| --- | --- | --- |
| `3V3` | row `4`, columns `2-4` bridged | `3V3` bus |
| `GND` | row `5`, columns `2-4` bridged | `GND` bus |
| `SDA` | row `6`, columns `2-4` bridged | Pico `GP4`, physical pin `6`, row `7` column `11` |
| `SCL` | row `7`, columns `2-4` bridged | Pico `GP5`, physical pin `7`, row `8` column `11` |

Solder both STEMMA cable pigtails to this fanout:

| STEMMA wire | Fanout signal |
| --- | --- |
| Red | `3V3` |
| Black | `GND` |
| Blue | `SDA` |
| Yellow | `SCL` |

One cable goes to the OLED on the lid. One cable goes to the MCP4725 DAC.

## Expression Input Jack

Panel-mount the expression pedal jack in the left side hole.

Use these perfboard tie points:

| Jack lug / part | Perfboard tie point |
| --- | --- |
| Input TRS Sleeve | `GND` bus, row `24`, column `4` |
| Input TRS Ring | `3V3` bus, row `23`, column `4` |
| Input TRS Tip wire | row `11`, column `5` |
| `1k` input resistor | row `11`, column `5` to row `11`, column `16` |
| GP26 input node | bridge row `11`, columns `16-18` |
| `100nF` capacitor leg 1 | GP26 input node, row `11`, column `16` |
| `100nF` capacitor leg 2 | local ground island, row `9`, column `16` |
| Local ground island | bridge row `9`, columns `16-18`; row `9`, column `18` is Pico `GND` pin `33` |

This gives the pedal input the same circuit as the schematic:

```text
Input TRS Tip ---- 1k ---- Pico GP26/ADC0
                           |
                         100nF
                           |
Input TRS Sleeve --------- GND

Input TRS Ring ----------- Pico 3V3(OUT)
```

## DAC And Output Jack

Panel-mount the Therevox output jack in the right side hole.

Mount the MCP4725 breakout near the right side of the enclosure, close to the output jack. It can sit on the perfboard right side or on the enclosure floor with foam tape/standoffs. The STEMMA cable handles `3V3`, `GND`, `SDA`, and `SCL`.

The STEMMA cable does not carry the DAC output, so add this separate wire:

| DAC / output part | Perfboard tie point |
| --- | --- |
| MCP4725 `VOUT` wire | row `12`, column `24` |
| `1k` output resistor | row `12`, column `24` to row `12`, column `29` |
| Output plug physical Tip wire | row `12`, column `29` |
| Output plug physical Ring | leave unconnected for active external CV |
| Output TRS Sleeve | `GND` bus, row `24`, column `29` |

Output circuit:

```text
MCP4725 VOUT ---- 1k ---- Output plug physical Tip
Output TRS Sleeve ------- GND

Output plug physical Ring - not connected
```

## Rotary Encoder

Panel-mount the encoder in the lid hole. Leave enough wire slack to open the lid.

The PEC11 body has:

```text
3 small pins on one side      = encoder rotation pins: A, C, B
2 small pins on opposite side = pushbutton switch
1 large lug on each side      = mechanical mounting tabs
```

On the 3-pin side, the middle pin is `C` / common. The two outside pins are `A` and `B`.

| Encoder lug | Pico destination |
| --- | --- |
| `C` / common | `GND` bus |
| `A` / CLK | Pico `GP14`, physical pin `19`, row `20` column `11` |
| `B` / DT | Pico `GP15`, physical pin `20`, row `21` column `11` |
| Switch lug | Pico `GP13`, physical pin `17`, row `18` column `11` |
| Other switch lug | `GND` bus |
| Large side lugs | no electrical connection; solder only for mechanical strength on perfboard |

If clockwise turns the interval down instead of up, swap encoder `A` and `B`.

## OLED

Mount the OLED behind the lid window.

- OLED PCB long direction runs left/right.
- Display glass/window centered in the printed lid opening.
- STEMMA connector/wire exits toward the rear or side, whichever gives the cleanest slack.
- Leave enough cable slack to open the lid without pulling the OLED or Pico.

The OLED connects only through the STEMMA cable fanout described above.

## Breadboard Approximation

For a bench-only breadboard test:

1. Put the Pico across the center trench with USB at the top/rear.
2. Use the breadboard red rail as `3V3` from Pico `3V3(OUT)`.
3. Use the breadboard blue/black rail as `GND` from a Pico `GND`.
4. Put the expression input parts on the left side of the breadboard.
5. Put the MCP4725/output parts on the right side.
6. Keep OLED and encoder on loose jumper wires in the same left/right/front/rear orientation as the enclosure.

Do not try to mount the breadboard in the enclosure. Once the circuit works, transfer it to the perfboard layout above.

## Mechanical Checklist

Before final soldering:

- Dry-fit the Pico/perfboard in the enclosure with the USB cable plugged in.
- Confirm the side jack lugs do not touch the perfboard or Pico.
- Confirm the encoder body clears the Pico and wires when the lid is closed.
- Confirm the OLED PCB clears the encoder nut/washer.
- Add strain relief: small zip tie, hot glue, or heat shrink on wires that go to jacks and lid controls.
- Verify with a multimeter before connecting the Therevox:
  - `3V3` bus to `GND`: about `3.3V`.
  - Input TRS Ring to Sleeve: about `3.3V`.
  - Output plug physical Tip to Sleeve at `interval 9`, heel: about `0.000V`.
  - Output plug physical Tip to Sleeve at `interval 9`, toe: about `3.214V`.
  - Output plug physical Tip to Sleeve at `interval -9`, heel: about `3.300V`.
  - Output plug physical Tip to Sleeve at `interval -9`, toe: about `0.086V`.
