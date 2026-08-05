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
| OLED window | lid centerline, front half, `30mm x 11.5mm` |
| Encoder hole | lid centerline, rear half, `7.4mm` diameter |
| Expression input jack | left side wall, centered front/back, square `23/32 inch` / `18.26mm` opening |
| Therevox output jack | right side wall, centered front/back, square `23/32 inch` / `18.26mm` opening |
| LFO1 3.5mm jack | front wall, left of center, square `5.72mm` opening, `20%` larger than the measured `6/32 inch` jack body |
| LFO2 3.5mm jack | front wall, right of center, square `5.72mm` opening, `20%` larger than the measured `6/32 inch` jack body |
| USB/service opening | rear wall, centered left/right |

## Perfboard Size And Placement

Use the full `90mm x 70mm` perfboard, mounted landscape. Do not cut it down.

```text
Board width left/right: 90mm
Board depth rear/front: 70mm
```

Place it on the enclosure floor like this:

- Centered left/right.
- Centered rear/front.
- Long `90mm` edge runs left/right.
- Short `70mm` edge runs rear/front.
- The long horizontal lettering sequence, `A-Z` then restarting at `A`, runs
  left/right.
- The short vertical `A-Z` lettering sequence runs rear/front.
- Pico USB end facing the rear USB opening.
- Mount the four board corner holes over the four short printed perfboard posts.

The generated baseplate uses your measured `3mm` perfboard corner holes. The
hole edge starts `1.3mm` from the left/right `70mm` side edges and `0.6mm` from
the top/bottom `90mm` side edges, so the hole centers are `2.8mm` and `2.1mm`
from those edges. That gives post spacing of:

```text
left/right post spacing: 84.4mm
rear/front post spacing: 65.8mm
```

The Amazon listing confirms the board is `90mm x 70mm`, but it does not publish
the exact corner-hole center offset. Before printing the full box, measure your
actual board. If the hole centers differ, change
`PERFBOARD_MOUNT_HOLE_INSET_X` and `PERFBOARD_MOUNT_HOLE_INSET_Y` in
`hardware/enclosure/generate_enclosure_stl.py` and regenerate the STL files.

Do not mount the OLED or encoder to the perfboard. They mount to the lid/panel and connect with flexible hookup wires.

## Perfboard Coordinate System

View the perfboard from above with the USB opening at the top. The exact
printed labels vary by board side; the mechanical STL only depends on the
`90mm x 70mm` outline and corner mounting holes.

```text
Rear / USB side

left/right: 90mm long edge
rear/front: 70mm short edge

Front / player side
```

The plan below assumes:

- The long `90mm` edge runs left/right.
- The short `70mm` edge runs rear/front.
- Pico USB faces the rear USB opening.
- The board is centered over the four printed perfboard posts.

## Step-By-Step Perfboard Transfer

Use this order when moving the tested breadboard circuit onto perfboard. Keep
the board loose until the electrical tests pass, then mount it in the printed
body.

1. Dry-fit the empty perfboard in the printed body.
   - Long `90mm` edge runs left/right.
   - Short `70mm` edge runs rear/front.
   - Pico USB will face the rear USB opening.
   - The four corner holes should sit over the four printed perfboard posts.

2. Put the Pico H on the perfboard, but do not solder every pin yet.
   - Pico USB connector points toward row `1` / rear USB opening.
   - Pico physical pin `1` sits at row `4`, column `J`.
   - Pico physical pin `20` sits at row `23`, column `J`.
   - Pico physical pin `40` sits at row `4`, column `Q`.
   - Pico physical pin `21` sits at row `23`, column `Q`.
   - Dry-fit the perfboard in the enclosure with a USB cable plugged in before
     soldering the Pico permanently.

3. Make the two underside power buses.
   - `3V3` bus: row `29`, columns `B-Y`.
   - `GND` bus: row `30`, columns `B-Y`.
   - Connect Pico `3V3(OUT)`, physical pin `36`, row `8` column `Q`, to row `29`.
   - Connect Pico `GND`, physical pin `38`, row `6` column `Q`, to row `30`.
   - Connect Pico `GND`, physical pin `33`, row `11` column `Q`, to row `30`.
   - Before adding modules, power from USB and verify row `29` to row `30` is
     about `3.3V`.

4. Build the I2C fanout at the left rear.
   - Bridge row `4`, columns `B-D` for `3V3`.
   - Bridge row `5`, columns `B-D` for `GND`.
   - Bridge row `6`, columns `B-D` for `SDA`.
   - Bridge row `7`, columns `B-D` for `SCL`.
   - Wire row `6` fanout to Pico `GP4`, physical pin `6`, row `9` column `J`.
   - Wire row `7` fanout to Pico `GP5`, physical pin `7`, row `10` column `J`.
   - Wire row `4` fanout to the row `29` `3V3` bus.
   - Wire row `5` fanout to the row `30` `GND` bus.

5. Move the expression input circuit.
   - Input TRS Sleeve goes to `GND` bus row `30`, column `C`.
   - Input TRS Ring goes to `3V3` bus row `29`, column `C`.
   - Input TRS Tip goes to row `13`, column `B`.
   - Put the `1k` input resistor from row `13`, column `B` to row `13`, column `P`.
   - Bridge row `13`, columns `P-Q`; row `13`, column `Q` is Pico `GP26/ADC0`.
   - Put the `100nF` capacitor from row `13`, column `P` to row `11`, column `P`.
   - Bridge row `11`, columns `P-Q`; row `11`, column `Q` is Pico `GND` pin `33`.

6. Wire the MCP4728 quad DAC.
   - DAC `V` goes to the `3V3` fanout.
   - DAC `S` goes to the `GND` fanout; this board's ground pin is printed `S`.
   - DAC `DA` goes to the `SDA` fanout.
   - DAC `CL` goes to the `SCL` fanout.
   - DAC `L` goes to the `GND` bus.
   - DAC `R` is left unconnected.
   - DAC `A` goes to row `14`, column `S`.
   - DAC `B` goes to row `16`, column `S`.
   - DAC `C` goes to row `18`, column `S`.
   - DAC `D` is optional clock output; leave it unconnected unless you add a
     third 3.5mm jack.

7. Wire the output jacks.
   - Put a `1k` resistor from row `14`, column `S` to row `14`, column `Y`.
   - Therevox output physical Tip goes to row `14`, column `Y`.
   - Therevox output Sleeve goes to `GND` bus row `30`, column `Z`.
   - Therevox output Ring is left unconnected.
   - Put a `1k` resistor from row `16`, column `S` to row `16`, column `Y`.
   - LFO1 3.5mm Tip goes to row `16`, column `Y`.
   - LFO1 Sleeve goes to `GND` bus row `30`, column `X`.
   - Put a `1k` resistor from row `18`, column `S` to row `18`, column `Y`.
   - LFO2 3.5mm Tip goes to row `18`, column `Y`.
   - LFO2 Sleeve goes to `GND` bus row `30`, column `Y`.

8. Wire the lid controls with flexible wire slack.
   - OLED red goes to `3V3` fanout.
   - OLED black goes to `GND` fanout.
   - OLED blue goes to `SDA` fanout.
   - OLED yellow goes to `SCL` fanout.
   - Encoder `C` / common goes to `GND`.
   - Encoder `A` / CLK goes to Pico `GP14`, physical pin `19`, row `22` column `J`.
   - Encoder `B` / DT goes to Pico `GP15`, physical pin `20`, row `23` column `J`.
   - One encoder switch lug goes to Pico `GP13`, physical pin `17`, row `20` column `J`.
   - The other encoder switch lug goes to `GND`.

9. Test before final mounting.
   - With USB power only, verify `3V3` to `GND` is about `3.3V`.
   - Verify input Ring to Sleeve is about `3.3V`.
   - Verify the OLED boots and the encoder changes values.
   - Verify serial reports MCP4728 pass at `0x60` and OLED pass at `0x3C`.
   - Run fixed CV tests before plugging into the Therevox: `cv 0`, `cv 1000`,
     and `cv 3200`, measuring output physical Tip to Sleeve.
   - Only after those checks pass, mount the perfboard to the printed posts and
     add strain relief to the jack and lid wires.

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

The OLED cable goes to the OLED on the lid. The MCP4728 DAC uses the same four I2C/power signals from this fanout through its `V`, `S`, `DA`, and `CL` header pins. The DAC ground pin is printed as `S`; there may not be a separate `GND` label on the breakout.

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

Mount the MCP4728 breakout near the right/front side of the enclosure, close to the output jacks. It can sit on the perfboard right side or on the enclosure floor with foam tape/standoffs. Its I2C/power pins connect as `V` -> `3V3`, `S` -> `GND`, `DA` -> `SDA`, and `CL` -> `SCL`; again, `S` is the DAC board's ground pin. Also tie the breakout's `L` / `LDAC` pin to the `GND` bus; a floating `LDAC` can freeze all four analog outputs even though I2C writes succeed. Leave `R` / `RDY` unconnected.

The I2C wiring does not carry the DAC outputs, so add these separate wires:

| DAC / output part | Perfboard tie point |
| --- | --- |
| MCP4728 `A` wire | row `14`, column `S` |
| `1k` expression output resistor | row `14`, column `S` to row `14`, column `Y` |
| Therevox output plug physical Tip wire | row `14`, column `Y` |
| Output plug physical Ring | leave unconnected for active external CV |
| Output TRS Sleeve | `GND` bus, row `30`, column `Z` |
| MCP4728 `B` wire | row `16`, column `S` |
| `1k` LFO1 resistor | row `16`, column `S` to row `16`, column `Y` |
| LFO1 3.5mm Tip wire | row `16`, column `Y` |
| LFO1 3.5mm Sleeve | `GND` bus, row `30`, column `X` |
| MCP4728 `C` wire | row `18`, column `S` |
| `1k` LFO2 resistor | row `18`, column `S` to row `18`, column `Y` |
| LFO2 3.5mm Tip wire | row `18`, column `Y` |
| LFO2 3.5mm Sleeve | `GND` bus, row `30`, column `Y` |
| MCP4728 `L` wire | `GND` bus, row `30` |
| MCP4728 `R` | leave unconnected |
| MCP4728 `D` | optional: `1k` to a spare 3.5mm clock jack (`clock lfo1|lfo2`); otherwise leave unconnected |

Output circuit:

```text
MCP4728 A ---- 1k ---- Therevox output plug physical Tip
Output TRS Sleeve -------- GND

Output plug physical Ring - not connected

MCP4728 B ---- 1k ---- LFO1 3.5mm Tip
LFO1 Sleeve --------------- GND

MCP4728 C ---- 1k ---- LFO2 3.5mm Tip
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

- Before printing the full enclosure or baseplate, measure the perfboard corner
  mounting-hole centers. If they are not `84.4mm x 65.8mm` apart, update
  `PERFBOARD_MOUNT_HOLE_INSET_X` / `PERFBOARD_MOUNT_HOLE_INSET_Y` in the
  enclosure generator and regenerate.
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
