# 3D Printed Enclosure

> **Untested:** This enclosure design has not been printed or built yet. The STLs are generated from nominal component dimensions and have never been test-fitted against real hardware. Expect to verify every cutout and tolerance yourself. Treat this as a starting point, not a proven design.

First-pass printable enclosure for the Pico/DAC/OLED/rotary-encoder prototype.

Generated files:

```text
stl/expression_controller_baseplate.stl
stl/expression_controller_body.stl
stl/expression_controller_lid.stl
```

The generator is:

```text
generate_enclosure_stl.py
```

## Dimensions

- Outside body footprint: `126 mm x 112 mm`
- Body height: `34 mm`
- Lid thickness: `3 mm`
- Wall thickness: `2.4 mm`
- Fits a Prusa Mini build plate.
- Perfboard footprint: `90 mm x 70 mm`, mounted landscape.
- Open baseplate footprint: `98 mm x 78 mm`, giving `4 mm` margin around the
  perfboard and no side walls or lid.
- Perfboard post height: `7 mm`, so the top of a nominal `1.6 mm` board sits
  about `11 mm` above the enclosure floor.

## Cutouts

- Top lid OLED window: `30 mm x 11.5 mm`
- Top lid rotary encoder hole: `7.4 mm`
- Left/right side TRS jack openings: square `23/32 inch` / `18.26 mm`,
  starting at the perfboard top surface.
- Front 3.5mm LFO jack openings: square `5.72 mm` / `20%` larger than `6/32 inch`, with the
  bottom edge `5/32 inch` / `3.97 mm` above the perfboard top surface.
- Rear USB/service opening: `18 mm x 10 mm`
- Lid screws: `M2.5` clearance holes
- Internal corner posts: `M2.5` pilot holes
- Internal perfboard posts: four short posts for a centered `90 mm x 70 mm` board

The TRS side openings are sized for your square-body 1/4 inch jacks. If your
jacks need a different opening, adjust `TRS_OPENING_W` / `TRS_OPENING_H` in
`generate_enclosure_stl.py` and regenerate.

The 3.5mm front openings are sized from your `5/32 inch` to `11/32 inch`
above-perfboard measurement. If those jacks need a different opening, adjust
`LFO_OPENING_W`, `LFO_OPENING_H`, or `LFO_BOTTOM_Z` in
`generate_enclosure_stl.py` and regenerate.

The perfboard posts use your measured `3 mm` board holes. The hole edge starts
`1.3 mm` from the left/right `70 mm` side edges and `0.6 mm` from the top/bottom
`90 mm` side edges, so the center insets are `2.8 mm` and `2.1 mm`. That gives
`84.4 mm x 65.8 mm` post spacing. The printed pilot holes are `3.2 mm` diameter
for tolerance. If your board measures
differently, adjust `PERFBOARD_MOUNT_HOLE_INSET_X` /
`PERFBOARD_MOUNT_HOLE_INSET_Y` in `generate_enclosure_stl.py` and regenerate.

## Print

Recommended first print:

```text
Material: PLA or PETG
Layer height: 0.20 mm
Perimeters: 3
Infill: 20%
Supports: off, or build-plate-only if your slicer wants support for the rear USB opening
```

Print orientation:

- `expression_controller_baseplate.stl`: flat bottom face on the build plate.
- `expression_controller_body.stl`: bottom face on the build plate, open side upward.
- `expression_controller_lid.stl`: outside/top face upward.

The baseplate is now the preferred raw-build part: it protects the soldered
underside and provides the four perfboard risers without enclosing the controls.
The full body/lid files remain available as older enclosure experiments.

## Intended Layout

```text
Top lid, with the rear USB opening facing away:
  rotary encoder in the rear half
  OLED in the front half

Left side:
  expression pedal TRS input

Right side:
  Therevox CV/output TRS

Front:
  LFO1 and LFO2 3.5mm patch outputs

Rear:
  USB access/service opening
```

For exact perfboard placement, Pico orientation, jack wiring, OLED/encoder wiring, and row/column solder points, see:

```text
../../docs/pico-physical-layout.md
```

## Regenerate

From the repository root:

```bash
python3 hardware/enclosure/generate_enclosure_stl.py
```

The script has no CAD dependencies. It writes ASCII STL files directly.
