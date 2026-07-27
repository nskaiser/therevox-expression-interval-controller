# BOM

This is the active Pico H prototype BOM.

| Item | Qty | Notes |
| --- | ---: | --- |
| Raspberry Pi Pico H | 1 | Adafruit PID 5525, headers already soldered |
| Adafruit MCP4725 I2C DAC breakout | 1 | PID 935, default address `0x62`; STEMMA handles I2C/power |
| Adafruit 0.91 inch 128x32 I2C SSD1306 OLED | 1 | PID 4440, default address `0x3C` |
| STEMMA QT / Qwiic JST-SH to male headers cable | 2 | PID 4209, one for DAC and one for OLED |
| Rotary Encoder + Extras | 1 | Adafruit PID 377 |
| TRS jack/breakout | 2 | One expression pedal input, one Therevox CV output |
| `1k` resistor | 2 | Input Tip series resistor and DAC output series resistor |
| `100nF` ceramic capacitor | 1 | Pico `GP26/ADC0` input filter to `GND` |
| Solderable perfboard | 1 | Use for the enclosure build |
| Hookup wire | as needed | `22-24 AWG`; use one separate wire for MCP4725 `VOUT` |
| USB micro cable | 1 | For programming and power |
| USB power bank | 1 | Recommended portable power source |
| M2.5 screws/standoffs or printed posts | as needed | For mounting perfboard/enclosure |

The STEMMA cable does not carry the DAC output. Wire this separately for the active-CV test:

```text
MCP4725 VOUT -> 1k -> output plug physical Tip
Output TRS Sleeve -> Pico GND
Output plug physical Ring -> leave unconnected
```

Optional finishing supplies:

| Item | Notes |
| --- | --- |
| Heat shrink or electrical tape | Insulate jack/control solder joints |
| Small zip ties or hot glue | Strain relief for lid and jack wires |
| Knob for encoder | Included with Adafruit encoder kit, but any matching knob is fine |
