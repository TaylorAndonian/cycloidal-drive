# cycloidal-50-d100 — 50:1 cycloidal gearbox, Ø100 max, for an arm on an X axis

Ø100 version of `cycloidal-50`, re-sized for an arm that sticks out along X and takes a downward Z
load. Same equations, same scripts, different parameter block — and four parts of the architecture
had to change, not just scale.

| Key numbers | Ø138 version | **this Ø100 version** |
|---|---|---|
| Housing OD × length | Ø138 × 71 | **Ø100 × 88** (from motor face to arm face) |
| Pin circle / pins | Ø112, 51 × Ø4 | **Ø80, 51 × Ø3 × 60** |
| Eccentricity / lobe height | 0.82 / 1.64 mm | **0.59 / 1.18 mm** |
| Discs | 2 × 12 mm | **2 × 14 mm** |
| Rated torque (continuous) | 36 Nm | **18 Nm** |
| Lobe yield onset (PA-CF) | 41 Nm | **30 Nm** |
| Hertz at rated | 61 MPa | **72 MPa** |
| Eccentric bearing | 6902 | **6802 (15×24×5)** |
| Output rollers | 12 × 608 (yours) | **24 × 625ZZ (5×16×5)** — the 608s no longer fit |
| Carrier rear brg | 6809 (yours) | **6804 (20×32×7)** — the 6809 no longer fits |
| Output brg | 6816 | **6810 (50×65×7)** |
| Arm interface | Ø48 PCD on a Ø97 body | **Ø72 face plate, 6 × M3 on Ø58 PCD, Ø10 pilot** |

Everything below is computed by the scripts in `scripts/` and lives in `analysis/results.json`,
`analysis/geometry_check.json` and `cad/layout.json`. **39/39 geometry checks pass**, exported STEP
profile follows the equations to 2 µm, 50 lobes counted from the solid, pin circle measured Ø80.000.

Re-run everything: `.\setup.ps1` (Windows) or `./setup.sh`.

---

## 1. What Ø100 costs you, stated plainly

Shrinking the pin circle from Ø112 to Ø80 shrinks the mesh's torque radius (`N·e`) from 41 mm to
**29.5 mm**, and forces `e` down from 0.82 to 0.59 mm. Torque per pin goes up, contact radius goes
down, so **contact stress at a given torque rises ~40 %**. That is why the rating drops from 36 Nm to
18 Nm. Nothing about the anti-skip architecture got worse — skip is still not the limit — the printed
lobe surface just runs out of margin sooner.

| torque | Hertz p_max | vs PA-CF yield (≈90 MPa) |
|---|---|---|
| 12 Nm | 59 MPa | 66 % |
| **18 Nm (rated)** | **72 MPa** | **80 %** |
| 24 Nm | 83 MPa | 92 % |
| 30 Nm | 93 MPa | over |

Skip margin at 18 Nm: elastic radial separation **0.065 mm** against **0.32 mm** of contact depth —
only 20 % consumed, and the cascade model finds no skip below 300 Nm. So run it into a hard stop and
it will stall or strip the lobe surface, not jump teeth. That is the behaviour you wanted.

A NEMA 17 at 0.45 Nm × 50 × ~0.85 efficiency delivers ≈19 Nm, so **the drive is matched to the motor
and the motor is the fuse.** You cannot easily overload it except by back-driving the arm into
something.

---

## 2. What changed architecturally (and why your 608s and 6809 can't come along)

**Output rollers: 608 → 625ZZ.** Six Ø22 rollers need a ≥Ø66 pin circle; at Ø100 the flange bearing
bore caps the pin circle at Ø48. Ø16 rollers (625ZZ) fit with a 3.4 mm ligament to the disc bore.
A 5 mm-wide 625 in a 14 mm disc would concentrate the hole contact on a third of the width, so
**two 625s side by side per pin per disc** (24 total, ~$0.30 each) cover 10 of the 14 mm.

**Carrier rear bearing: 6809 → 6804.** The 6809's Ø58 OD would swallow the whole output pin circle.
A 6804 (Ø32 OD) clears it and clears the motor screw counterbores by 2.7 mm. Your 6809s stay in the
drawer for the Ø138 build.

**Eccentric bearing: 6802, not 6801.** The first pass used a 6801 (bore 12). Going up to a 6802
(bore 15) doubles the printed cam's section modulus and **halves the shaft bending stress from 53 to
27 MPa at 2× torque** — the printed shaft was one of the top-three risks in the Ø138 design, and this
fixes it for free. Cam wall around the D-bore goes 2.9 → 4.4 mm.

**Rear input bearing: in the carrier, not the cup.** The first cut of this design put the 6801 in the cup
floor exactly where the Ø138 version had it. That works when the carrier bearing is a 6809 (bore Ø45, room
to spare inside it) and fails silently when it becomes a 6804 (bore Ø20) — the 6801's Ø21 outer race is
*larger than the hole it was drawn inside*, so it had no seat at all and also overlapped the carrier boss
by 806 mm³. The fix moves it into a pocket in the carrier plate's front face (z 18.5–23.5, on a 3 mm floor),
which also straddles the shaft properly: both journals are now carried by the carrier, and the carrier is
grounded on the 6804 and 6810. Shaft support span 35.8 mm; cam bending stress at 2× torque actually
*improves*, 53 → 45 MPa.

`check_geometry.py` now probes a 1 mm shell around every bearing race and asks how much of it is solid
(threshold 85 %). Run against the faulty geometry that check reads **0 %**; against the fix, 100 %. It is
the test that should have existed from the start — the old interference tests only ever compared printed
part against printed part, so a bearing floating in a void looked fine.

**Ring pins: Ø3, not Ø4.** 51 pins on Ø80 gives a 4.93 mm pitch; Ø4 would leave a 0.93 mm ligament in
the plates. Ø3 leaves 1.93 mm. The cost is real: pin bending stiffness falls from 2 984 to **698
N/mm**, and pin bending is now 96 % of the total per-pin compliance (Hertz 82 000, seats 32 000 /
52 800 N/mm). It is still not enough to cause skip — see §4.

---

## 3. The arm load, which is now the sizing case for the output end

Assumed **2 kg at 300 mm reach, ×2 dynamic factor** (edit `m_kg, reach_mm, dyn` in `analysis.py`):

| quantity | value |
|---|---|
| static moment | 5.9 Nm |
| dynamic moment | 11.8 Nm |
| output bearing span (6804 ↔ 6810) | 59.6 mm |
| reaction at the 6810 (front) | 217 N |
| reaction at the 6804 (rear) | 198 N |
| share per output pin from the couple | 66 N |
| 6810 static capacity | ~8.3 kN (verify) |

**The moment is not a bearing problem — it is 2.6 % of the 6810's static rating.** It is a
*stiffness* and *architecture* problem, and three things in this design are there because of it:

1. **Two grounded bearings, 60 mm apart.** A single output bearing would let the arm tilt on the
   printed flange. The 6804/6810 pair takes the couple as two opposed radial loads.
2. **A real Ø72 face plate with a Ø58 bolt circle**, not a small pattern on the flange body. Bolt
   circle diameter is what sets how much of the arm's moment becomes bolt tension rather than
   flange-face prying: at Ø58, 11.8 Nm is ~203 N per bolt pair. On the Ø48 pattern of the previous
   version it would be 245 N.
3. **The Ø10 centre pilot.** Put a matching spigot on the arm root — the bolts should not be locating
   the arm in shear.

**Mount the gearbox by the rear cup**, not the front plate: 4 × M3 heat-set inserts on a Ø90 PCD in
the back face (the same face the motor bolts to). The load path is arm → flange → 6810 → front plate
→ 8 × M3 tie bolts → shell → cup → your machine. Those 8 tie bolts see 97 N of shear each from
torque; the arm moment adds prying, which is why they are M3×30 through the full plate into 9 mm-deep
inserts rather than short screws.

**Orientation note:** with the arm horizontal, gravity puts a steady radial load on the ring-pin
mesh in one direction, so one side of the ring wears preferentially. If the arm has a home position,
vary it, or plan to rotate the front plate 180° at some point (the pin holes are symmetric).

---

## 4. The floating mid-ring: proposed, modelled, rejected

Because Ø3 pins are so much more compliant, I modelled an extra part — a 3 mm printed ring floating
in the gap between the two discs, with 51 slip-fit holes, tying all the pins at mid-span. It is in the
code (`MID_RING = True`) and it works, but:

| | k_pin | separation @18 Nm | margin @18 Nm |
|---|---|---|---|
| no mid ring | 669 N/mm | 0.065 mm | 0.307 mm |
| with mid ring | 749 N/mm | 0.062 mm | 0.318 mm |

**+12 % pin stiffness, +3.6 % margin, for 3 mm of length and an extra part.** The ring is itself only
186 N/mm (it is a thin printed annulus, and its own ovalisation eats most of the benefit). Since skip
margin is already 5× what it needs to be, it does not earn its place. **Rejected — set `MID_RING =
True` in `cad.py` if you disagree**; the geometry and the assertions are all there.

The same logic says: don't spend effort stiffening the pins. Spend it on the disc material.

---

## 5. Everything else that carries load, at 2× rated (36 Nm)

| item | value | capacity / limit |
|---|---|---|
| eccentric bearing 6802, per disc | 1 003 N | C0 ≈ 1.27 kN (verify) — **this is the second limit; at rated 18 Nm it is 506 N** |
| 625 roller | 441 N | C0 ≈ 0.6 kN (verify); 222 N at rated |
| ring pin bending (Ø3) | 265 MPa | hardened dowel ≫ 1 000 MPa |
| output pin bending (Ø5) | 268 MPa | hardened dowel; 134 MPa at rated |
| printed shaft, cam section | 22 MPa | printed PC ~60 MPa in Z |
| printed shaft, collar | 12 MPa | " |
| carrier reaction from the internal couple | 218 N | on top of the 198 N arm reaction |
| front plate ovalisation | 0.022 mm | (cup floor 0.003 mm) |
| output wind-up at rated | 0.25° | plus ~0.3° backlash from the 0.10 profile clearance |
| mesh efficiency | 91 % / 83 % / 76 % | µ = 0.10 / 0.20 / 0.30 |

Two limits sit close together at 2×: the lobe surface (30 Nm) and the 6802 (≈36 Nm). At the rated
18 Nm both are at ~40–80 % — that is the margin you're buying with the smaller envelope.

---

## 5b. Shaft compensation — when the printed shaft comes out undersize

The eccentric shaft is the only printed part where a dimensional error changes the **kinematics**,
because the eccentricity `e` is a printed *position*, not just a size. So it gets its own
compensation, wired into the parameters block:

```python
SHAFT_XY_SCALE = 1.000,  # multiplies every shaft XY dimension INCLUDING e and the D-bore
SHAFT_DIA_COMP = 0.00,   # mm added to each external diameter, subtracted from the D-bore
```

Both act in XY only. Z is left alone: on FDM the layer height is mechanical and rarely carries the
same error. Set **one** of them and leave the other neutral.

### Which one?

Two different faults look identical if you only measure one diameter:

| fault | cause | signature | fix |
|---|---|---|---|
| **shrinkage** | material contracting as it cools | every diameter under by the same **percentage** | `SHAFT_XY_SCALE` |
| **bead width / flow** | extrusion narrower than modelled | every diameter under by the same **amount**, and **holes come out oversize** | `SHAFT_DIA_COMP` |

They need different corrections, because shrinkage moves `e` and a bead-width error does not.
Getting this backwards leaves the eccentricity wrong, which shows up later as a failed indexing
check or a drive that meshes shallowly and feels notchy.

### The two-minute test

Print **`cad/coupon_shaft.stl`** — 17 × 17 × 27 mm, ~15 minutes — in the shaft's own material,
orientation (vertical, Ø12 end on the bed) and profile. It carries all four shaft diameters and the
full D-bore. Then:

1. **Try the D-bore on your motor shaft.** This is the fastest discriminator, because holes err in
   the *opposite* direction to outside surfaces:
   - **sloppy / rattles** → bead-width error → `SHAFT_DIA_COMP`
   - **will not go on** → shrinkage → `SHAFT_XY_SCALE`
2. **Measure the Ø8.03 front journal band.** For a 0.53 mm error on the cam it reads **7.75** if
   shrinkage, **7.50** if bead width — 0.25 mm apart, unambiguous with calipers.

### Setting the value

Let `measured` be the cam band and `15.03` the modelled value:

- shrinkage → `SHAFT_XY_SCALE = 15.03 / measured`
- bead width → `SHAFT_DIA_COMP = 15.03 - measured`

For a cam measuring 14.50 that is `1.0366` or `0.53`. Re-run `python scripts\cad.py`, reprint the
coupon, confirm, then print the shaft. Verified round-trip at 1.0366: the model becomes Ø15.579 with
e = 0.612, which at 3.53 % shrink prints as **Ø15.030 with e = 0.590** — both back on target.

### How much is too much

PA-CF shrinks about 0.3–0.6 %, PC about 0.6–0.9 %. **A 3.5 % error is five to ten times that**, so it
is a machine or profile fault, not the material: check the slicer's shrinkage-compensation field for
that filament, the flow/extrusion multiplier, and that the model was not scaled on import. Compensation
will get one part to size, but a machine that far out will not hold the indexing spec
(1.18 mm TIR, cams 180° ± 3°, amplitudes matched within 0.05 mm) repeatably.

**If the shaft still fails the indexing check after compensation, stop printing it.** This part was
already risk #2 in §9 — it carries 1 kN per cam on printed journals. `cad/eccentric_shaft.step` is
the drawing; turned from steel or aluminium it is a 20-minute lathe job and removes the risk entirely.

### Check the discs before you reprint anything

A 3.5 % error that is machine-wide, not shaft-specific, matters far more than the shaft:

- Discs should measure **79.16 mm ±0.15** across the lobe tips. At 3.5 % under they would be **76.4**.
- The pin circle is set by **steel pins in printed plates**. The pins do not shrink, so if the plates
  shrink the ring geometry no longer matches the disc profile and the drive will not mesh properly at
  any compensation.

If the discs and plates are on size and only the shaft is off, this is a shaft-profile problem and the
compensation above is the whole fix.

---

## 6. Tolerances

Identical strategy to the Ø138 version; the numbers that changed:

| fit | CAD nominal | intent |
|---|---|---|
| Ø3 ring pin, cup floor | Ø2.95 × 8 deep | firm press |
| Ø3 ring pin, front plate | Ø2.98 × 15 deep (10.2 engaged) | light press |
| Ø5 output pin, carrier / flange | Ø4.95 through / × 12.5 deep | press |
| bearing OD in printed pocket | OD − 0.05 | press |
| 6802 OD in disc bore | Ø23.95, 1.0 mm ledge, centred in the 14 mm disc | press |
| printed journal in bearing bore (rear Ø12, cams Ø15, front Ø8, bosses) | ID + 0.03 | press |
| roller hole in disc | Ø17.28 = 16 + 2e + 0.10 | 0.05 radial slop |
| disc ↔ thrust faces | 0.4 mm per face; disc ↔ disc 1.0 mm | clearance |
| lobe profile clearance | **0.10 mm** — same start point | see below |
| M3 clearance / insert | Ø3.4 / Ø4.0 × 9.0 deep, Ø4.6 countersink | |

Thinnest features anywhere in the model: disc bearing ledge 1.00 mm, face plate under the arm
inserts 1.00 mm, printed washer wall 1.10 mm, shell wall beside an insert 1.50 mm, ring pins to shell
1.50 mm. All ≥ 2 perimeters.

**Profile clearance:** binds → +0.03 at a time; skips or feels loose → −0.03. At Ø100 the sweep is
tighter than before because the lobes are shallower: 0.05 → 12 pins share the load, 0.10 → 13 pins,
0.20 → 8 pins. Don't go below 0.05; that is inside the H2D's XY repeatability.

Print `coupon_pins_inserts` and `coupon_bearings` first. **The Ø3 row on the pin coupon is new and
matters more than it did at Ø4** — a loose Ø3 hole loses a larger fraction of the pin's seat.

---

## 7. Print settings

0.4 mm nozzle, no supports anywhere.

| part | material | orientation | walls | infill | layer |
|---|---|---|---|---|---|
| rear_cup | PA-CF (PC ok) | back face on bed, open side up | 5 | 40 % gyroid | 0.2 |
| front_plate | PA-CF (PC ok) | front face on bed, pin holes up | 5 | 40 % gyroid | 0.2 |
| disc ×2 | **PA-CF, no substitute** | flat, Ø22 side on bed | 6 | 100 % | 0.16 |
| eccentric_shaft | **PC** (best Z) | vertical, Ø12 end on bed; 45° cones modelled | — | 100 % | 0.12 |
| carrier_rear_plate | PA-CF | boss up | 5 | 60 % | 0.2 |
| output_flange | PA-CF or PC | **face plate on the bed**, pin holes up | 5 | 60 % | 0.2 |
| washers ×18 | anything rigid | flat | 3 | 100 % | 0.2 |

Only the **discs** truly need PA-CF (PC yields at 18 Nm — exactly your rated torque, so PC discs give
zero margin). Both plates want it for stiffness. The shaft wants PC for Z-strength. The flange is
carrying the arm moment, so use PA-CF there too if you have it.

---

## 8. Assembly (differences from the Ø138 version in **bold**)

1. Heat-set 8 inserts in the shell top, 6 in the flange face plate, 4 in the cup back face.
2. Motor into the cup pilot; 4 × M3×8 from inside through the counterbores.
3. Press the 6804 into the cup floor (the 6801 goes into the carrier plate, step 5). Press the 51 Ø3 ring pins into the floor
   (8 mm deep). **Ø3 pins bend far more easily than Ø4 — use a drill-press chuck as a square pusher
   and check each one against a square. A pin leaning 1° is 0.7 mm out at the front plate.**
4. Shaft: press the cam-A 6802 on **from the rear** (over the Ø12 journal), the cam-B 6802 on **from
   the front** (over the Ø8 journal). Both butt on the Ø17 collar.
5. Press the **6801 into the carrier plate's front face** (Ø21 pocket, 3 mm floor). Then **slide the carrier onto the shaft from the REAR end** — its Ø14 bore clears the Ø12
   rear journal but not the Ø15 cams, so it can only go on from that side. Do it now.
6. Indexing check: dial-indicate each 6802 outer race. Each must read **2e = 1.18 mm** total runout,
   peaks 180° apart within ~3°, amplitudes matching within 0.05 mm.
7. Press the 6 × Ø5 pins into the carrier plate, then per pin: washer 2.4 → 625 → 625 → washer 5.0 →
   625 → 625 → washer 2.4.
8. Shaft + carrier sub-assembly into the cup: D-bore over the motor shaft, carrier boss into the 6804.
9. Disc A onto cam A's bearing, rollers into its six holes. Grease the lobes. Disc B rotated 3.6°
   (180/50) out of phase — the roller holes force this, it only drops on in one phase.
10. Flange onto the six pin ends (11.5 mm engagement) with the 608 over the front journal; press the
    6810 onto the flange boss.
11. Front plate: start all 51 pins and the 6810 together, draw down with the 8 × M3×30 in a star
    pattern. **No counterbores at Ø100** — the heads sit on the front face, clear of the Ø72 arm plate.
12. Turn by hand; then bolt the arm on through the Ø10 pilot.

---

## 9. Three things most likely to go wrong (updated for this version)

1. **Lobe surface creep, sooner than before.** 72 MPa at rated vs 61 MPa on the Ø138. Symptoms:
   backlash growth, then reversal clunk. Keep it greased, keep peak torque below ~24 Nm, and expect
   the first few hours to bed in. If you need more, the fix is machined POM or nylon discs, not more
   printed thickness (14 mm is already past the knee).
2. **Ø3 ring-pin seats.** The single most load-bearing tolerance in the design. A loose front-plate
   hole turns a two-end-supported pin into a cantilever and undoes the whole anti-skip argument.
   Use the coupon; reprint the plate if a hole is loose. Do not glue.
3. **The arm's moment on a printed flange over time.** Bearings are fine; the printed flange body
   between the pins and the boss creeps under a constant 12 Nm. Symptom: the arm droops a fraction of
   a degree and doesn't come back. If it matters, add a fourth check to your routine: measure arm tip
   height at a known pose monthly.

Also worth knowing: the ring pins sit 1.5 mm from the shell wall, so there is no room to add a metal
band later — the Ø138 analysis showed a band was worthless anyway (<7 % of the plate's stiffness).

---

## 10. Still uncertain — please verify

- **Ø3 × 60 mm DIN 6325 dowels are the one iffy purchase.** They exist (UK suppliers list them; some
  eBay multi-size listings include 3 × 60) but they are not as stocked as 4 mm. Fallback: Ø3 h6
  silver steel / drill rod, cut to 60 mm and deburred square — same hardness class, ~$1/metre, but
  51 cuts of work.
- Bearing capacities are catalogue values from memory (6802 C0 ≈ 1.27 kN, 625 ≈ 0.6 kN, 6804 ≈ 1.5 kN,
  6810 ≈ 8.3 kN). The 6802 is close enough to its limit at 2× that you should check the real number
  for whatever you buy.
- The arm load is my assumption (2 kg @ 300 mm, ×2). If your real case is heavier or further out,
  change the three numbers in `analysis.py` and re-run — the bearing reactions scale linearly.
- Printed material properties, insert geometry and the NEMA 17 31 mm square bolt pattern carry the
  same caveats as the Ø138 README §10.
- Efficiency is a mesh-sliding model only; add 2–4 % for bearings and seals.
