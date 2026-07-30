# 3D Printed Enclosure

> **Untested:** This enclosure design has not been printed or built yet. The STLs are generated from nominal component dimensions and have never been test-fitted against real hardware. Expect to verify every cutout and tolerance yourself. Treat this as a starting point, not a proven design.

First-pass printable enclosure for the Pico/DAC/OLED/rotary-encoder prototype.

Generated files:

```text
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

## Cutouts

- Top lid OLED window: `30 mm x 11 mm`
- Top lid rotary encoder hole: `7.4 mm`
- Left/right side TRS jack holes: `10.5 mm`
- Front 3.5mm LFO jack holes: `6.5 mm`
- Rear USB/service opening: `18 mm x 10 mm`
- Lid screws: `M2.5` clearance holes
- Internal corner posts: `M2.5` pilot holes
- Internal perfboard posts: four short posts for a centered `70 mm x 90 mm` board

The TRS holes are sized for common 1/4 inch panel jacks, but jack bushings vary. If your jacks need a different diameter, adjust `TRS_HOLE_D` in `generate_enclosure_stl.py` and regenerate.

The 3.5mm holes are sized for small threaded panel jacks, but those vary too. If
your jacks need a different diameter, adjust `LFO_HOLE_D` in
`generate_enclosure_stl.py` and regenerate.

The perfboard posts assume the board's corner mounting-hole centers are `3.0 mm`
in from each edge, giving `64 mm x 84 mm` post spacing. If your board measures
differently, adjust `PERFBOARD_MOUNT_HOLE_INSET_MM` in
`generate_enclosure_stl.py` and regenerate.

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

- `expression_controller_body.stl`: bottom face on the build plate, open side upward.
- `expression_controller_lid.stl`: outside/top face upward.

Before printing the full body, print the lid first and test-fit the OLED and encoder. Those two holes are the most likely to need small tolerance tweaks.

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
