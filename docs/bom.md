# BOM

This is the active Pico H prototype BOM.

| Item | Qty | Notes |
| --- | ---: | --- |
| Raspberry Pi Pico H | 1 | Adafruit PID 5525, headers already soldered |
| MCP4728 quad I2C DAC breakout | 1 | DIYmall GY-MCP4728 or equivalent, default address `0x60` |
| Adafruit 0.91 inch 128x32 I2C SSD1306 OLED | 1 | PID 4440, default address `0x3C` |
| STEMMA QT / Qwiic JST-SH to male headers cable | 1 | For OLED |
| Rotary Encoder + Extras | 1 | Adafruit PID 377 |
| 1/4 inch TRS jack/breakout | 2 | One expression pedal input, one Therevox expression/CV output |
| 3.5mm TS or TRS jack | 2 | Dedicated `LFO1` and `LFO2` patch outputs; leave Ring unconnected on TRS jacks |
| 3.5mm TS or TRS jack | 1 (optional) | Clock output from MCP4728 `D` |
| `1k` resistor | 4 minimum, 5 with optional clock | Input Tip series resistor plus DAC `A`, `B`, and `C`; add one more for optional DAC `D` clock output |
| `100nF` ceramic capacitor | 1 | Pico `GP26/ADC0` input filter to `GND` |
| Solderable perfboard | 1 | Use for the enclosure build |
| Hookup wire | as needed | `22-24 AWG`; use separate wires for MCP4728 `A`, `B`, and `C` |
| USB micro cable | 1 | For programming and power |
| USB power bank | 1 | Recommended portable power source |
| M2.5 screws/standoffs or printed posts | as needed | For mounting perfboard/enclosure |

The I2C wiring does not carry the DAC outputs. Wire these separately:

```text
MCP4728 V  -> Pico 3V3(OUT), physical pin 36
MCP4728 S  -> Pico GND, physical pin 38  (this DAC pin is printed `S`, not `GND`)
MCP4728 DA -> Pico GP4/SDA, physical pin 6
MCP4728 CL -> Pico GP5/SCL, physical pin 7
MCP4728 L  -> Pico GND (required; outputs can freeze if LDAC floats)
MCP4728 R  -> leave unconnected
MCP4728 A  -> 1k -> Therevox expression output plug physical Tip
MCP4728 B  -> 1k -> LFO1 3.5mm jack Tip
MCP4728 C  -> 1k -> LFO2 3.5mm jack Tip
MCP4728 D  -> leave unconnected, or 1k -> optional clock 3.5mm jack Tip (clock lfo1|lfo2 command)
All output jack sleeves -> Pico GND
Therevox expression output plug physical Ring -> leave unconnected
```

Optional finishing supplies:

| Item | Notes |
| --- | --- |
| Heat shrink or electrical tape | Insulate jack/control solder joints |
| Small zip ties or hot glue | Strain relief for lid and jack wires |
| Knob for encoder | Included with Adafruit encoder kit, but any matching knob is fine |
