# cycloidal-30-d80 — 30:1, NEMA 17, Ø80 × 51.5 mm

Small-joint companion to `cycloidal-50-d100`. Same architecture, same bearing
family, scaled down to the Ø80 × ~50 mm envelope.

```
ratio            30:1   (31 ring pins, 30 lobes)
envelope         Ø80.0 × 51.5 mm  (motor face to arm face)
motor            NEMA 17, Ø5 shaft
pin circle R     31.0 mm
eccentricity     0.79 mm   (curtate ratio 1.27)
backlash         0.08 mm profile offset
rated torque     ~8.7 Nm continuous, lobe-contact limited
motor-limited    ~10.1 Nm  (0.45 Nm holding × 30 × 0.75 eff.)
output speed     up to ~20 rpm at a sane NEMA 17 step rate
```

The two limits land within 15% of each other, which is where you want them:
the gearbox and the motor give out at about the same place, so nothing is
carrying dead weight.

## What changed from the Ø100 unit

Three things, all forced by the smaller diameter:

The ring pins went **up** in size (Ø3 → Ø4) even though the box got smaller.
At 31 pins instead of 51 the circumferential pitch is actually wider here, so
there is room for a fatter pin, and fatter pins are what buy back the contact
stress lost to the smaller radius. This is the single biggest reason the unit
still makes ~9 Nm at 64% of the diameter.

The output rollers became **plain dowels**. There is no radial room for 625ZZ
rollers inside a Ø59.4 disc, so the eight Ø6 dowels run directly in the printed
disc holes. This sounds worse than it is: the pin only oscillates ±0.79 mm
inside its hole per input revolution, it never rotates continuously, so it is a
low-amplitude sliding contact rather than a bearing duty. Print the discs in
PA-CF and grease the holes on assembly.

The eccentric shaft is now **supported at both ends** — a 6801 behind it in the
motor plate and a 608 in the output boss ahead of it. On the Ø100 unit the
shaft hangs off the motor bearings. Since your original prototype failed by
radial disengagement (ring spreading, eccentric deflecting), holding both ends
of the shaft and capturing all 31 pins in both end plates is the specific fix
for that mode, and it costs nothing but the 2 mm of motor plate.

## Bill of materials

Printed (7 parts, ~200 cm³ total):

| part | qty | material |
|---|---|---|
| `motor_plate` | 1 | PA-CF or PC |
| `sleeve` | 1 | PA-CF or PC |
| `disc_a`, `disc_b` | 1 each | **PA-CF** (different parts — B is phased 6°) |
| `carrier` | 1 | PA-CF |
| `ring_plate` | 1 | PA-CF or PC |
| `face_plate` | 1 | PA-CF or PC |
| `shaft` | 1 | **PA-CF**, or turn from Ø15 steel |
| `coupon` | 1 | **print this first** — same material and profile as the real parts |

Bought:

| item | qty | note |
|---|---|---|
| 6801-2RS (12×21×5) | 5 | 2 per disc + 1 rear support — [10-pack, ~$10](https://www.amazon.com/uxcell-6801-2RS-Bearing-12x21x5mm-Bearings/dp/B07TJDVQ3D) |
| 6808-2RS (40×52×7) | 1 | output bearing — [5-pack](https://www.amazon.com/6808-2RS-61808RS-Bearing-40x52x7-Bearings/dp/B0DPZC1TFP) or [single](https://bearingsdirect.com/6808-2rs-ball-bearing-40x52x7-sealed-6808ddu/) |
| 608 (8×22×7) | 1 | nose support — **you already have a pile of these** |
| Ø4 × 30 dowel pin | 31 | ring pins — [DIN 6325 hardened](https://www.aspenfasteners.com/m4-x-30mm-din-6325-metric-solid-dowel-pins-steel/); hardened & ground matters here |
| Ø6 × 32 dowel pin | 8 | output pins — [100-pack, hardened](https://www.amazon.com/Dowel-Through-Hardened-Finish-Quantity/dp/B07FMKL5NK) |
| M3 × 45 SHCS | 6 | housing tie screws |
| M3 heat-set insert | 6 | motor plate |
| M4 heat-set insert | 4 | carrier boss |
| M4 × 10 SHCS | 4 | face plate |
| M3 × 4 grub screw | 1 | shaft to motor shaft |
| Ø30 × 0.5 shim washer | 1 | between discs |

Ring pins are the one place not to cheap out — 31 soft pins will brinell and
the box will develop exactly the backlash you are trying to design out. The
Ø6 output pins can be ordinary hardened dowels.

## Printing

Print `coupon.stl` first. It's a 92 × 26 × 8 bar carrying every press fit
and clearance in the gearbox — both 6801 seats, the 608 seat, three ring-pin
pockets at true pitch, the Ø6 dowel press fit, the output-pin orbit hole and
both insert holes. Twenty minutes of printing tells you whether your X/Y
compensation is right before you commit 200 cm³ of PA-CF. A 6801 should push
in with thumb pressure and stay; a Ø4 dowel should slide in the through holes
and need a light tap in the blind one.

0.4 nozzle, 0.15 mm layers, 5 walls, 40% gyroid. All parts print flat on the
bed with no supports; the discs, the carrier and the sleeve all have their
critical features as vertical holes.

The discs are the only parts where accuracy matters. Print them one at a time,
turn off anything that trades dimensional accuracy for speed, and dial in your
X/Y compensation first — a 0.1 mm hole error here is larger than the 0.08 mm
of designed backlash. If the assembled box is tight, raise `DISC_CLEARANCE` in
`params.py` to 0.10 and reprint the discs; if it feels notchy with visible
lash, drop it to 0.06.

## Assembly order

1. Heat-set the M3 inserts into the motor plate and the M4s into the carrier boss.
2. Press the rear 6801 into the motor plate, the 6808 into the ring plate, and
   the 608 into the carrier boss.
3. Press the eight Ø6 dowels into the carrier. They should need a light tap —
   the holes are 0.05 mm under.
4. Slide the shaft into the rear 6801, seat it on the motor shaft, **tighten
   the grub screw now** — collar 1 is buried once the sleeve goes on.
5. Two 6801s onto cam A, drop disc A on. Shim washer. Two onto cam B, disc B on,
   rotated so its output holes line up with the carrier's dowel pattern.
6. Drop the sleeve over the stack and feed the 31 ring pins down through it into
   the motor plate pockets. Grease them as they go.
7. Carrier down over the shaft nose, dowels through both discs.
8. Ring plate on, catching the pin tops. Six M3 × 45 through into the motor
   plate inserts, crossed and even.
9. Face plate onto the boss with four M4 × 10.

Turn the motor shaft by hand before powering anything. It should feel smooth
and slightly heavy, with a faint 31-per-revolution ripple. Grittiness or a hard
spot means a disc is phased wrong — pull the carrier and rotate disc B.

## Files

```
params.py      every dimension; edit here
check.py       clearance, web and capacity checks    -> all pass
mesh_check.py  pin-by-pin gap over a full revolution -> uniform 0.080 mm
cad.py         builds out/*.step and out/stl/*.stl
render.py      builds out/preview.png
bom.py         builds out/BOM-d80.csv and out/BOM-d80.xlsx
```

Run `python check.py && python mesh_check.py && python cad.py` after any change
to `params.py`. The checks are cheap and they catch the failures that only show
up after you have printed 200 cm³ of PA-CF.

## The output interface

Ø70 face plate, 6 × M3 clearance on a Ø50 bolt circle, same generic pattern as
the big joint so arm hardware interchanges. The 6808 takes the radial load and
the 608 on the shaft nose gives the moment a second, widely spaced reaction —
but a single 6808 is still the weak point for a long cantilevered arm. If this
joint carries more than about 1 kg at 200 mm, put the arm's own bearing outboard
of the face plate rather than asking the 6808 to do it.
