# Sample Patches

Patch ideas for the Therevox ET-5 patch panel using this controller's outputs:

```text
EXP   1/4 inch expression/CV output (pitch-bend intervals, or a third LFO)
LFO1  3.5mm LFO output, DAC channel B
LFO2  3.5mm LFO output, DAC channel C
CLK   optional 3.5mm clock output, DAC channel D (clock lfo1|lfo2)
```

The ET-5 panel's top two rows are `0-5V` CV inputs; the bottom rows are
performance-control outputs. The passive 4-way mults split or combine CVs, and
the middle inverter flips a control's response.

The LFO jacks put out `0-3.3V`, which drives any ET-5 input to about two thirds
of full range. Use `depth` and `offset` to place the modulation window: depth
scales the swing around the midpoint, offset shifts the center up or down.

Serial settings shown below can also be dialed in from the OLED menu.

## With EXP Playing Pitch-Bend Intervals

The pedal owns pitch; LFO1/LFO2 act as a second player.

### Auto-Crossfade Tremolo

```text
LFO1 -> vol 1
LFO2 -> vol 2

focus lfo1
wave sine        (square for hard alternation)
rate 1.5         (or tap twice at tempo)
focus lfo2
link 1:1
link phase 180
```

The two oscillators breathe in opposition, morphing between their timbres while
you bend on top. The link is phase-derived, so the pair never drifts.

### Breathing Filter

```text
LFO2 -> cutoff

focus lfo2
wave drift
rate 0.1
depth 70
```

Drift never repeats a cycle, so the tone slowly evolves under sustained notes.
Swap `wave sh` at a faster rate for stepped random filter jumps.

### Rhythmic Sparkle, Locked Together

```text
LFO1 -> vol 1
LFO2 -> cutoff

focus lfo1
wave pulse
pw 25
rate 2
focus lfo2
wave sh
link 1:2
```

LFO1 chops the volume; every second chop lands on a new random filter color.

### Slow Spatial Swells

```text
LFO2 -> reverb   (or fx mix)

focus lfo2
wave sine
rate 0.05
```

The space around the instrument inhales and exhales over ten to twenty seconds.

### Vibrato On Osc 2 Only

```text
LFO1 -> pitch 2

focus lfo1
wave sine
rate 5.5
depth 10
offset -45
```

Low depth plus a negative offset keeps the swing small and near `0V`, so osc 2
shimmers against a pure osc 1 instead of sounding like a siren.

## With EXP As A Third LFO

Set `mode lo` or `mode fm` on `EXP` and the pedal sweeps that LFO's speed —
a rate pedal.

### Foot-Controlled Rotary Speaker

```text
EXP  -> mult -> vol 1
        mult -> cutoff
LFO1 -> vol 2

focus exp
mode lo
wave sine
focus lfo1
wave sine
link phase 90    (after link 1:1 with LFO2 if used)
```

Heel is a slow throb, toe a fast shimmer, and the pedal ramps between them like
a Leslie spinning up.

### Playable FM Growl

```text
EXP -> fm 1

focus exp
mode fm
wave sine
```

The pedal sweeps `8-160Hz` of rough FM, from slow warble at heel to clangorous
sidebands at toe.

### Quadrature Orbit

```text
LFO1 -> vol 1
LFO2 -> vol 2
EXP  -> fx mix

focus lfo1
wave sine
rate 0.5
focus lfo2
link 1:1
link phase 90
```

At 90 degrees the sound seems to circle between the oscillators rather than
see-saw. The pedal puts motion depth underfoot.

### Interlocking Polyrhythm

```text
LFO1 -> vol 1
LFO2 -> vol 2
CLK  -> external gear clock input

focus lfo1
wave pulse
pw 25
rate 2       (or tap twice at tempo)
focus lfo2
wave pulse
link 3:2
clock lfo1
```

A 3-against-2 pattern between the oscillators that never drifts, with the rest
of the rig locked to the clock jack.

### Self-Evolving Drone

```text
hold 1 -> vol 1        (ET-5 hands-free drone)
LFO1   -> pitch 2
LFO2   -> reson

focus lfo1
wave drift
rate 0.05
depth 15
offset -40
focus lfo2
wave sh
rate 0.2
```

The drone plays itself; you sculpt it from the encoder.

## Utility Tricks

- **Extra knobs**: `depth 0` plus `offset` turns any LFO jack into a fixed CV —
  a set-and-forget level for `reson` or `fx mix`.
- **Complementary motion**: run one LFO through the panel inverter to a second
  destination — for example bright-but-dry / dark-but-wet from a single source.
- **Sync as a gesture**: `sync all` (or a short press per output) restarts every
  waveform together — useful at section changes.
