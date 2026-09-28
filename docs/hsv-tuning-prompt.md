# HSV tuning prompt

Paste this whole file (instructions + your photos) to a vision-capable AI whenever you need to
(re)derive HSV thresholds for Pollen or Nectar — a new venue's lighting, a new ball batch, or
adding a ball type this file doesn't cover yet. The output slots directly into the `BALL_TYPES`
tuple of `limelight/ball_detection_snapscript.py` (and `ball_contour_snapscript.py`, which shares it).

Treat the numbers it gives you as new **defaults** to paste in, not a final answer — confirm them
live in the Limelight web editor with `DISPLAY_MODE = "MASK"`, which paints each type's mask in its
own colour so you can see directly what each threshold is and isn't catching. Photos analyzed
offline are a starting point; the Limelight's sensor, compression, and lighting are the ground
truth.

---

## Instructions (copy from here down)

You are deriving HSV colour thresholds for an FTC vision pipeline that detects wiffle-ball-style
game pieces: **Pollen** (yellow, 2.8in) and **Nectar** (red or blue, 3.6in). I will give you one or
more photos of ball(s) of a single type per batch. Analyze the actual pixels and report thresholds
in the exact format below — do not just state typical/textbook HSV ranges for "yellow" or "red"
from memory; the numbers must come from the pixels in the photos I give you.

### 1. Critical convention: this is OpenCV's HSV, not standard HSV

OpenCV's 8-bit HSV is scaled differently from the HSV you'll see in most colour pickers:

- **H (hue): 0–179**, not 0–360 or 0–255. Convert with `openCV_H = standard_H_degrees / 2`.
- **S (saturation): 0–255**
- **V (value/brightness): 0–255**

Every number you report must already be in this 0–179 / 0–255 / 0–255 space. If you catch yourself
about to write a hue above 179, you've used the wrong scale — halve it.

Rough OpenCV-hue neighbourhoods, for sanity-checking your own output only (derive the real bounds
from the photos, don't just use these): yellow ≈ 15–35, red ≈ wraps around 0/179 (roughly 0–10 and
170–179), blue ≈ 95–130.

### 2. What you're building: two bands per ball type, not one

Each ball type needs a **colour band** (the ball's actual saturated surface colour) and a **glare
band** (the washed-out specular highlight, usually near the top of the ball under any overhead
light). They get OR-ed together into one mask, so:

- **Colour band**: `hLow/hHigh`, `sLow/sHigh`, `vLow/vHigh` — sampled from the ball's normally-lit
  surface. Saturation and value should both be reasonably high here; this is the ball's "true" colour.
- **Glare band**: same hue range as the colour band, but `sHigh` becomes `glareSHigh` (a **low**
  ceiling — glare desaturates) and `vLow` becomes `glareVLow` (a **high** floor — glare is bright).
  The glare band always implicitly runs `sLow=0` and `vHigh=255`.

Sample the glare band from the bright/blown-out patch on the ball's surface specifically, separately
from the main colour band — don't average them together into one sample set, or you'll get numbers
that describe neither region well.

**Note on red's hue**: OpenCV hue wraps at the 0/179 seam, and red's true hue sits right next to it.
A ball type takes any number of hue ranges, each with the same saturation, value and glare bounds.
If your sampled red pixels cluster near just one end (e.g., all near 179, or all near 0), report one
hue range. If they genuinely span **both** ends, report two, one ending at 179 and one starting at 0,
rather than one range that only covers whichever end has more samples.

### 3. How to sample pixels from the photos

For each photo:

1. Identify the ball's silhouette. Sample pixels from well inside it — stay several pixels back from
   the edge, where anti-aliasing and background bleed-through will contaminate the sample and drag
   your saturation/value floors down artificially.
2. Exclude: the shadow the ball casts on the ground (not part of the ball), any part of a different
   ball or object in frame, motion-blurred pixels, and the very edge/rim where the surface curves
   out of focus.
3. Within what's left, separate into two pixel populations by eye: the "normally lit, saturated"
   majority of the surface (→ colour band) and any small bright/white/washed-out highlight patch
   (→ glare band). A photo with no visible glare just won't contribute glare-band samples — that's
   fine, use whatever other photos have it, or fall back to a modest default (`glareSHigh` a bit
   above the colour band's own low-saturation photos, `glareVLow` around 190–210).
4. Convert sampled pixels' RGB to OpenCV HSV (H/2 for hue, as above) and note each pixel's (H,S,V).

Do this across **every photo you're given for that ball type** before computing bounds — one photo
under one lighting condition will give you a threshold too narrow for real, variable competition
lighting.

### 4. Turning samples into bounds

Don't use raw min/max of your samples — a single stray outlier pixel (a sub-pixel edge blend, a
sensor noise spike) will blow a bound out needlessly. Instead:

1. For each channel (H, S, V) in each population (colour, glare), take roughly the **5th–95th
   percentile** of your sampled values as the core range.
2. Pad **outward**: hue ±2–4, saturation/value floors down by ~10–15, saturation/value ceilings up
   to 255 unless something meaningful caps them. This mask only has to seed a region-of-interest for
   the circle search — the actual shape validation happens downstream, so it's fine, even
   correct, for these bounds to stay loose rather than tight. Erring wide costs a little extra
   compute; erring narrow costs missed balls.
3. Clamp everything into legal ranges: H ∈ [0,179], S/V ∈ [0,255].

### 5. Output format — match this exactly

Report one line per ball type (only the type(s) I actually gave you photos for), in this literal
Python form: the four arguments that follow the label in that type's `BallType(...)` entry of
`BALL_TYPES`, so they can be pasted straight over the old ones.

```python
# Pollen:      hue ranges, (sLow, sHigh), (vLow, vHigh), (glareSHigh, glareVLow)
((<>, <>),), (<>, <>), (<>, <>), (<>, <>)
# Red Nectar:  one hue range, or two if the samples straddle the 0/179 wrap
((<>, <>), (<>, <>)), (<>, <>), (<>, <>), (<>, <>)
# Blue Nectar
((<>, <>),), (<>, <>), (<>, <>), (<>, <>)
```

After each line, also report, as plain numbers (not Python), so I can sanity-check your work:

- How many photos and roughly how many sampled pixels went into that type's colour band and glare
  band separately.
- The raw 5th/95th-percentile numbers **before** you padded them, so I can see how much padding you
  added.
- Anything that looked off: suspiciously low saturation/value floors (possible background
  contamination), a photo where lighting looked unlike the others (possible outlier worth excluding
  or noting), or too few glare-band pixels to trust that half of the range.

### 6. Photo requirements — tell me if these aren't met

For the result to transfer to the real robot instead of just describing your photos:

- Photos should come from (or closely resemble) the actual Limelight 3A feed, not a phone camera — a
  different sensor's colour science shifts hue/saturation in ways no amount of careful sampling
  fixes. Photos taken at a different exposure, gain or white balance than the ball pipeline's Input
  tab won't match what the pipeline actually sees.
- Include photos at multiple distances/angles and, ideally, multiple lighting conditions — a
  threshold derived from one ideal, close-up, evenly-lit photo will be too narrow for a ball
  half-shadowed across the field under gym lighting.
- Include at least one photo per type where a bright glare highlight is visible, or the glare band
  will just be a guess.

If any of this is missing from what I gave you, say so explicitly rather than silently producing a
narrower threshold than the pipeline actually needs.
