# Pico Physical Layout

This layout is meant to match the generated enclosure in `hardware/enclosure/stl/`.

> **Untested:** The enclosure has not been printed or built yet, so this physical layout is a plan, not a verified build. The bench wiring in `docs/pico-prototype.md` is what has actually been tested.

Use perfboard for the enclosure build. A solderless breadboard is fine for a bench test, but it will not fit cleanly in the printed box and the side jacks/lid controls will pull wires loose.

## Enclosure Orientation

Hold the box so the rear USB opening faces away from you.

```text
Rear / USB side
+----------------------------------------------+
|                 USB opening                  |
|                                              |
|              rotary encoder hole             |
|                                              |
|  expression input TRS      Therevox output   |
|  jack on left side         TRS jack on right |
|                                              |
|                OLED window                   |
|          LFO1 3.5mm      LFO2 3.5mm          |
+----------------------------------------------+
Front / player side
```

The generated enclosure dimensions are:

| Feature | Position |
| --- | --- |
| Body outside | `126mm x 112mm x 34mm` |
| OLED window | lid centerline, front half, `30mm x 11mm` |
| Encoder hole | lid centerline, rear half, `7.4mm` diameter |
| Expression input jack | left side wall, centered front/back |
| Therevox output jack | right side wall, centered front/back |
| LFO1 3.5mm jack | front wall, left of center, `6.5mm` nominal hole |
| LFO2 3.5mm jack | front wall, right of center, `6.5mm` nominal hole |
| USB/service opening | rear wall, centered left/right |

## Perfboard Size And Placement

Use the full `70mm x 90mm` perfboard with printed columns `A-Z` and rows
`1-31`. Do not cut it down.

```text
Board width left/right: 70mm, columns A-Z
Board depth rear/front: 90mm, rows 1-31
```

Place it on the enclosure floor like this:

- Centered left/right.
- Centered rear/front.
- Printed row `1` nearest the rear USB opening.
- Printed row `31` nearest the front/player side.
- Printed column `A` nearest the expression input jack.
- Printed column `Z` nearest the Therevox output jack.
- Pico USB end facing the rear USB opening.
- Mount the four board corner holes over the four short printed perfboard posts.

The generated enclosure assumes the perfboard corner mounting-hole centers are
`3.0mm` in from each board edge. That gives post spacing of:

```text
left/right post spacing: 64mm
rear/front post spacing: 84mm
```

The Amazon listing confirms the board is `70mm x 90mm`, but it does not publish
the exact corner-hole center offset. Before printing the full box, measure your
actual board. If the hole centers are not `3.0mm` from the edges, change
`PERFBOARD_MOUNT_HOLE_INSET_MM` in
`hardware/enclosure/generate_enclosure_stl.py` and regenerate the STL files.

Do not mount the OLED or encoder to the perfboard. They mount to the lid/panel and connect with flexible hookup wires.

## Perfboard Coordinate System

View the perfboard from above with the USB opening at the top.

```text
Rear / USB side

columns:  A  B  C ... Z
rows:     1  2  3 ... 31

Front / player side
```

The plan below assumes:

- Row `1` is nearest the rear USB opening.
- Row `31` is nearest the front/player side.
- Column `A` is nearest the expression input jack.
- Column `Z` is nearest the Therevox output jack.

## Pico Placement

Put the Pico H on the perfboard first.

```text
Pico USB connector points toward row 1 / rear USB opening.

Pico physical pin 1  -> row 4,  column J
Pico physical pin 20 -> row 23, column J

Pico physical pin 40 -> row 4,  column Q
Pico physical pin 21 -> row 23, column Q
```

That puts the Pico's two header rows `7` columns apart, which matches the Pico's `0.7 inch` row spacing on 0.1 inch perfboard.

Useful Pico pins in this layout:

| Signal | Pico label | Physical pin | Perfboard hole |
| --- | --- | ---: | --- |
| `SDA` | `GP4` | `6` | row `9`, column `J` |
| `SCL` | `GP5` | `7` | row `10`, column `J` |
| Encoder switch | `GP13` | `17` | row `20`, column `J` |
| Encoder A / CLK | `GP14` | `19` | row `22`, column `J` |
| Encoder B / DT | `GP15` | `20` | row `23`, column `J` |
| Pedal ADC | `GP26/ADC0` | `31` | row `13`, column `Q` |
| Local ADC ground | `GND` | `33` | row `11`, column `Q` |
| 3.3V rail feed | `3V3(OUT)` | `36` | row `8`, column `Q` |
| Main ground rail feed | `GND` | `38` | row `6`, column `Q` |

## Power Rails On Perfboard

Perfboard holes are not connected unless you solder them.

On the underside, make two bare-wire buses:

| Bus | Perfboard location |
| --- | --- |
| `3V3` | row `29`, columns `B` through `Y` |
| `GND` | row `30`, columns `B` through `Y` |

Then wire:

| From | To |
| --- | --- |
| Pico `3V3(OUT)`, physical pin `36`, row `8` column `Q` | `3V3` bus, row `29` |
| Pico `GND`, physical pin `38`, row `6` column `Q` | `GND` bus, row `30` |
| Pico `GND`, physical pin `33`, row `11` column `Q` | `GND` bus, row `30` |

## I2C Fanout For OLED And DAC

Make a small fanout area on the left rear of the perfboard. This gives you easy solder points for both STEMMA cables.

| Fanout signal | Perfboard tie point | Connects to |
| --- | --- | --- |
| `3V3` | row `4`, columns `B-D` bridged | `3V3` bus |
| `GND` | row `5`, columns `B-D` bridged | `GND` bus |
| `SDA` | row `6`, columns `B-D` bridged | Pico `GP4`, physical pin `6`, row `9` column `J` |
| `SCL` | row `7`, columns `B-D` bridged | Pico `GP5`, physical pin `7`, row `10` column `J` |

Solder the OLED STEMMA cable pigtail and the MCP4728 I2C/power wires to this fanout:

| Signal / OLED STEMMA wire | Fanout signal |
| --- | --- |
| Red | `3V3` |
| Black | `GND` |
| Blue | `SDA` |
| Yellow | `SCL` |

The OLED cable goes to the OLED on the lid. The MCP4728 DAC uses the same four I2C/power signals from this fanout through its `VCC`, `GND`, `SDA`, and `SCL` header pins.

## Expression Input Jack

Panel-mount the expression pedal jack in the left side hole.

Use these perfboard tie points:

| Jack lug / part | Perfboard tie point |
| --- | --- |
| Input TRS Sleeve | `GND` bus, row `30`, column `C` |
| Input TRS Ring | `3V3` bus, row `29`, column `C` |
| Input TRS Tip wire | row `13`, column `B` |
| `1k` input resistor | row `13`, column `B` to row `13`, column `P` |
| GP26 input node | bridge row `13`, columns `P-Q` |
| `100nF` capacitor leg 1 | GP26 input node, row `13`, column `P` |
| `100nF` capacitor leg 2 | local ground island, row `11`, column `P` |
| Local ground island | bridge row `11`, columns `P-Q`; row `11`, column `Q` is Pico `GND` pin `33` |

This gives the pedal input the same circuit as the schematic:

```text
Input TRS Tip ---- 1k ---- Pico GP26/ADC0
                           |
                         100nF
                           |
Input TRS Sleeve --------- GND

Input TRS Ring ----------- Pico 3V3(OUT)
```

## DAC And Output Jacks

Panel-mount the Therevox output jack in the right side hole.

Panel-mount the `LFO1` and `LFO2` 3.5mm jacks in the front wall. If those jacks are TRS, use only `Tip` and `Sleeve`; leave `Ring` unconnected.

Mount the MCP4728 breakout near the right/front side of the enclosure, close to the output jacks. It can sit on the perfboard right side or on the enclosure floor with foam tape/standoffs. Its I2C/power pins connect to `3V3`, `GND`, `SDA`, and `SCL`. Also tie the breakout's `LDAC` pin to the `GND` bus — a floating `LDAC` can freeze all four analog outputs even though I2C writes succeed.

The I2C wiring does not carry the DAC outputs, so add these separate wires:

| DAC / output part | Perfboard tie point |
| --- | --- |
| MCP4728 `VOUTA` wire | row `14`, column `S` |
| `1k` expression output resistor | row `14`, column `S` to row `14`, column `Y` |
| Therevox output plug physical Tip wire | row `14`, column `Y` |
| Output plug physical Ring | leave unconnected for active external CV |
| Output TRS Sleeve | `GND` bus, row `30`, column `Z` |
| MCP4728 `VOUTB` wire | row `16`, column `S` |
| `1k` LFO1 resistor | row `16`, column `S` to row `16`, column `Y` |
| LFO1 3.5mm Tip wire | row `16`, column `Y` |
| LFO1 3.5mm Sleeve | `GND` bus, row `30`, column `X` |
| MCP4728 `VOUTC` wire | row `18`, column `S` |
| `1k` LFO2 resistor | row `18`, column `S` to row `18`, column `Y` |
| LFO2 3.5mm Tip wire | row `18`, column `Y` |
| LFO2 3.5mm Sleeve | `GND` bus, row `30`, column `Y` |
| MCP4728 `LDAC` wire | `GND` bus, row `30` |
| MCP4728 `VOUTD` | optional: `1k` to a spare 3.5mm clock jack (`clock lfo1|lfo2`); otherwise leave unconnected |

Output circuit:

```text
MCP4728 VOUTA ---- 1k ---- Therevox output plug physical Tip
Output TRS Sleeve -------- GND

Output plug physical Ring - not connected

MCP4728 VOUTB ---- 1k ---- LFO1 3.5mm Tip
LFO1 Sleeve --------------- GND

MCP4728 VOUTC ---- 1k ---- LFO2 3.5mm Tip
LFO2 Sleeve --------------- GND
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
| `A` / CLK | Pico `GP14`, physical pin `19`, row `22` column `J` |
| `B` / DT | Pico `GP15`, physical pin `20`, row `23` column `J` |
| Switch lug | Pico `GP13`, physical pin `17`, row `20` column `J` |
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
5. Put the MCP4728/output parts on the right/front side.
6. Keep OLED and encoder on loose jumper wires in the same left/right/front/rear orientation as the enclosure.

Do not try to mount the breadboard in the enclosure. Once the circuit works, transfer it to the perfboard layout above.

## Mechanical Checklist

Before final soldering:

- Before printing the full enclosure, measure the perfboard corner mounting-hole
  centers. If they are not `64mm x 84mm` apart, update
  `PERFBOARD_MOUNT_HOLE_INSET_MM` in the enclosure generator and regenerate.
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
  - LFO1 Tip to Sleeve: moving voltage between about `0V` and `3.3V`.
  - LFO2 Tip to Sleeve: moving voltage between about `0V` and `3.3V`.
