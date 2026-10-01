# Cycloidal 30:1, Ø80 small joint, v3

NEMA 17 in, 30:1, Ø80 × 50.5 mm. Same hardware as v2 — **nothing new to buy.** All printed parts are new.

## Why v2 was stiff, and what v3 does about it

| Cause | v2 | v3 |
|---|---|---|
| Disc profile had almost no clearance, so the pins wedged in the lobes | 0.02 mm | **0.08 mm** (`MESH_CLEAR`) |
| Output holes were sized for zero backlash, so 8 pins fought each other | roller + 2e exactly | **+0.12 mm** (`OUT_CLEAR`) |
| Bearing seats at nominal printed undersize, pressed in hard and squeezed the disc bore oval, tightening the mesh | 26.00 | **26.10** (`BORE_PRESS`) |
| Ring pins pinched between the back plate and the end of the groove | 0 play | **0.6 mm end play** |
| Shaft held by motor bearings *and* a tight 608 *and* the cams — any misalignment bent it | tight 608 seat | 608 seat **+0.30 mm**, floats. It limits deflection instead of fighting the motor |
| Rear 6704 seat tight, so back-plate misalignment pinched the output | nominal | seat **+0.30 mm**, floats |
| Printed ODs under a bearing came out undersize | nominal | **+0.125 mm** on every one, your number |
| Eccentric bearings slid around inside the discs | plain through bore | **relief / seat / lip pocket** — the bearing drops in from the back, bottoms on the lip, and is trapped |
| Shaft broke: thin Ø10 hex with a Ø5 bore through it, plus a thin Ø8 journal | printed hex shaft | **one-piece shaft with cam A built in**: Ø14 body, Ø13 drive, Ø10 mid. No hex. Cam B slides on a single flat, so it can only go on the right way |

Everything the printer affects now lives in one block at the top of `cyc30_v3.py` under `FITS`.

## Fit table

| Parameter | Value | Where |
|---|---|---|
| `OD_SHRINK` | +0.125 | printed OD a bearing sits on: cam journals 17.125, flange journal 50.125, nose 8.125 |
| `BORE_PRESS` | +0.10 | bearing OD into a printed bore: disc 26.10, housing 65.10 |
| `BORE_SLIP` | +0.30 | floating seats: 6704 in the rear ring 27.30, 608 in the flange 22.30 |
| `SHAFT_SLIP` | −0.15 | the 6704 slides onto the back-plate boss: 19.975 |
| `MESH_CLEAR` | 0.08 | disc profile |
| `OUT_CLEAR` | +0.12 | disc output holes, 11.62 |
| `PIN_CLEAR` / `PIN_ENDPLAY` | 0.15 / 0.6 | ring-pin grooves |
| `AX_GAP` | 0.9 | gaps between parts that rotate relative to each other |

## Hardware — all of it is already in your v2 build

| Qty | Part | Note |
|---|---|---|
| 1 | 6810-2RS | output |
| 1 | 6704-2RS | rear support, now a floating fit |
| 2 | 6803-2RS | one per disc, centred (you have 4, so 2 spare) |
| 1 | 608 | front steady bearing, floating |
| 16 | MR105ZZ | output-pin rollers |
| 31 | Ø4 × 30 dowel | ring pins |
| 8 | Ø5 × 25 / M4 shoulder bolt | output pins |
| — | M3/M4 inserts, M3 screws | as v2; **no M3 set screw needed any more** |

## Checks I ran on the model

- Every printed part is a single connected solid. (This caught a real break in the shaft — it was in two pieces at one point.)
- No unexpected interference at six input angles. The only overlaps are the four intended shrink-allowance fits.
- Minimum gap between a pin's surface and the disc flank over a full turn: **80 µm**, matching the target.
- The disc-1 bearing clears everything in front of cam A, and cam B's slide-on path to its seat is clear.
- Shoulder-bolt span works out at exactly 25.00 mm.

## Print

Same materials as before: PA-CF (dried) or PC for the shaft, cam, discs and housing. 5+ walls, 100% infill on the shaft, cams and discs.

**Print `fit_coupon.step` first.** It now tests every fit in the table: three 6803 seats (26.0 / 26.1 / 26.2), the shoulder-bolt hole, the pin groove, both insert holes, the floating 6704 and 608 seats, plus three pegs — a 6803 must slide onto the 17.125 peg by hand, a 608 onto the 8.125 peg, and the 6704 onto the 19.975 peg. There's also a motor D-bore test and a cam-B D-drive test.

If a fit is off, change the one number in `FITS` and re-run `python cyc30_v3.py`.

**Shaft:** print it standing up, motor end down, 100% infill. Its sections are now much thicker and the load on it is much lower, but a printed shaft is still weakest across the layers. If it ever breaks again, the fallback is a turned Ø14 aluminium shaft to the same drawing.

## Assembly

1. Inserts: housing (6 back, 6 front), flange (8× M4 back, 8× M3 front), shaft tip (1× M3).
2. Press the 6810 into the housing from the front; fit the retainer ring with 6× M3×8 countersunk.
3. Drop the 608 into the flange's pocket — it should be loose, that's intentional. Push the flange through the 6810 from inside; fit the output cap with 4× M3×8 countersunk.
4. Push one 6803 into each disc **from the rear face** until it bottoms on the lip.
5. Rotor, on the bench: slide disc 1 down the shaft onto cam A. Slide cam B on (one orientation fits), then disc 2 onto cam B's journal, then the spacer.
6. Rotor into the housing, nose into the 608. Through the cap's centre hole fit M3×6 + the 4.3×12 washer and tighten — that clamps cam B and the spacer against cam A.
7. **Spin test:** turn the shaft by hand now. It should spin freely with only bearing drag. If it's stiff at this point the problem is the rotor, not the mesh.
8. Shoulder bolts through the rear ring; on each stack sleeve 3.9, MR105, sleeve 6.5, MR105, sleeve 3.6. Feed through the disc holes and thread into the flange. Star pattern, snug only.
9. **Spin test again** before the pins go in.
10. Slide the 31 ring pins in from the back, turning the shaft as you go.
11. **Spin test again.** This is the real one: one input turn = 1/30 output turn, and it should turn with light finger pressure on the shaft.
12. Motor onto the back plate (4× M3×6 from the inside), line up the D-flat, then back plate onto the housing with 6× M3×12.

## If it's still stiff

Work down this list — each step is a one-part reprint:

1. **Stiff before the pins go in (step 9):** the rotor or output carrier is binding. Check the 608 is loose in its pocket and the 6704 is loose in the rear ring.
2. **Stiff only after the pins (step 11):** it's the mesh. Raise `MESH_CLEAR` to 0.11 and reprint only the two discs.
3. **Tight spots once per output turn:** one pin groove or one disc lobe is off. Rotate slowly and find it.
4. **Free but with noticeable backlash:** go the other way — drop `MESH_CLEAR` to 0.05 and `OUT_CLEAR` to 0.08.

## Still unproven

Torque is still an estimate: roughly 10–12 Nm continuous in PA-CF, with the motor topping out near 10–11 Nm through 30:1. Extra mesh clearance means slightly fewer pins share the load at light torque, so expect a little more backlash than v2 aimed at — on purpose, because v2's zero backlash is what seized it. Once it runs freely you can tighten it up with step 4 above.
