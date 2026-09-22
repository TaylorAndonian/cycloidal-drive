# Cycloidal 30:1, Ø80 small joint, v2 (full revision)

NEMA 17 in, 30:1, housing Ø80 x 50 mm, output face at 50.5 mm from the motor face.
Every rotating interface runs on a rolling bearing, including the output pins, and the output carrier is held at both ends.

## What changed from v1 (cycloidal-30-d80)

| Problem in v1 | v2 fix |
|---|---|
| No way to get the cams onto the shaft | The input shaft has a **10 mm hex**. Both cams are the **same part** with a hex bore. They slide on from the front, cam B turned 180°. One M3 screw at the shaft tip clamps the whole stack (hub shoulder, cam A, cam B, spacer, 608 inner race). |
| Input shaft carried only by the motor | The shaft tip runs in a **608** inside the output flange. The cycloid loads no longer bend the motor shaft. |
| Plain Ø6 dowels as output pins (sliding, cantilevered) | **8 x Ø5 ground shoulder bolts with MR105ZZ rollers.** Each bolt is clamped between the output flange and a **rear ring**, so the pins are held at both ends and don't bend. |
| Output side load on one bearing plus a nose bearing | **6810 (50x65x7) at the front + 6704 at the rear, 36 mm apart.** The rear ring rides on a boss on the back plate, so the whole output carrier resists tipping. |
| Ring pins in a thin ring could spread under load (the v1/prototype failure) | Ø4 pins sit in **blind grooves in a solid one-piece housing** with a 5 mm+ wall behind them. Blind grooves also stop the pins axially at the front. |
| Looks | Plain one-piece Ø80 housing, flush front retainer, flush output cap, all screws hidden or recessed. |

## Key numbers

- 31 ring pins (Ø4 x 30) on R 31.5, 30-lobe discs, e = 0.75 mm, 2 discs x 10 mm at 180°
- Disc profile offset +0.02 mm (`PROFILE_DELTA`). Output holes are exactly roller OD + 2e = 11.5 mm, so the design itself has zero backlash and what's left comes from print tolerance.
- The two discs are the **same part**. Disc 2 goes in turned **90°** from disc 1 (the dimple on each disc face shows this). That puts its lobes half a tooth out of phase while the output holes still line up.
- Estimated capacity (PA-CF discs): about **12 Nm continuous**. Hertz stress at the lobes is ~50–75 MPa if about a third of the pins share the load. A NEMA 17 through 30:1 gives about 10–11 Nm, so **the motor is now the limit, not the gearbox**. PC discs: about 9–10 Nm continuous.
- Eccentric bearings see about 300 N per disc (2x 6803 each, far below their rating). Output pins see about 70 N each.
- Checked in CAD: no interference between any parts at input angles of 0°, 41°, 97° and 180°. The discs mesh with the pins with zero gap at all tested angles.

## Print (Bambu H2D / P1P, 0.4 nozzle)

| Part | Qty | Material | Notes |
|---|---|---|---|
| disc | 2 | PA-CF (dried) or PC | Print flat, 100% infill or 6+ walls, 0.12–0.16 layers |
| cam | 2 | PA-CF / PC | Print flat (collar down), 100% infill |
| input_shaft | 1 | PA-CF / PC | Print standing (motor end down), 100% infill |
| housing | 1 | PA-CF / PC | Front face down, 5+ walls |
| back_plate, rear_ring, output_flange, output_cap | 1 each | PA-CF / PC (PETG OK for cap) | 5+ walls, 40%+ infill |
| front_retainer, shaft_spacer | 1 each | any | |
| roller_sleeve_3p9 / roller_sleeve_6p9 | 16 / 8 | PETG / PA | |
| fit_coupon | 1 | same as real parts | **Print this first** |

Bearing seats are modeled at nominal size. Print `fit_coupon.step` first and check: the 6803 seats (25.9/26.0/26.1), 6704 and 6810 rings, the Ø5 shoulder fit, the Ø4 pin fit, the insert holes, and the hex + D-bore hub against your motor. If you need to change something, edit `PRESS` / `SLIP` / `PROFILE_DELTA` at the top of `cyc30_v2.py` and re-run it (`pip install build123d`, then `python cyc30_v2.py`).

## Assembly order

1. Install the heat-set inserts: housing (6 rear, 6 front), output flange (8x M4 rear, 8x M3 front), input shaft tip (1x M3).
2. Press the **6810** into the housing from the front, against the step. Fit the front retainer with 6x M3x8 flat head.
3. Press the **608** into the rear of the output flange. Put the flange into the 6810 from inside the housing. Fit the output cap with 4x M3x8 flat head (the cap clamps only the inner race).
4. Press 2x **6803** into each disc and the **6704** into the rear ring.
5. Rotor stack: shaft → cam A (collar toward the hub) → disc 1 → cam B (turned 180°) → disc 2 (dimple 90° from disc 1) → shaft_spacer (large side toward cam B). Push the shaft tip into the 608. From the front, through the cap hole, fit M3x6 + the 4.3x12 washer and tighten.
6. Push 8 shoulder bolts through the rear ring from the back. On each one, stack: sleeve 3.9, MR105, sleeve 6.9, MR105, sleeve 3.9. Slide the ring and bolts in through the disc holes (rollers clear the holes) and tighten each bolt into the flange.
7. Slide the 31 ring pins into the grooves from the back, turning the input shaft to help them in.
8. Bolt the motor to the back plate with 4x M3x6 from the inside face. Line up the D-flat, then fit back plate and motor: the boss goes into the 6704 and the spigot into the bore. Fit 6x M3x12 tie screws. Use longer ones if they also clamp your robot bracket.

## Interfaces

- **Motor:** NEMA 17, Ø22 pilot, 31 mm pattern. The shaft must reach at least 20 mm past the face. The D-bore is round for the first 7.5 mm, then has the flat.
- **Arm:** 4x M3 on PCD 40 (inserts in the flange, through the cap) plus 2x Ø4 dowels on PCD 28 at 0°/180°. This **replaces the v1 Ø70 / PCD 50 plate**, so the arm adapter needs updating.
- **Mounting:** through the 6 tie-screw positions on the back face (PCD 73.2).

## Files

`step/ASSEMBLY_cycloidal_30_d80_v2.step`: full assembly, with stand-ins for the bought parts and a NEMA 17 reference.
One STEP per printed part. `cyc30_v2.py` is the parametric source. `section_XZ.png` is a true cross-section.
