# cycloidal-50 — 50:1 single-stage cycloidal gearbox for a NEMA 17

Fully printable except bearings, pins, screws and heat-set inserts. Every dimension comes from the
parameters block at the top of `scripts/cad.py`; every number in this README comes from
`analysis/results.json` and `cad/layout.json`, which the scripts regenerate.

| Key numbers | |
|---|---|
| Ratio | 50 lobes / 51 pins → **50:1**, output opposite to input |
| Pin circle | Ø112 mm (R = 56), 51 × Ø4 × 60 hardened dowels, held in **both** plates |
| Eccentricity | e = 0.82 mm (K = e·Zp/R = 0.75), lobe height 1.64 mm, profile clearance 0.10 mm |
| Discs | 2 × 12 mm thick, 180° apart, on one printed eccentric shaft |
| Envelope | Ø138 × 71 mm from motor face to flange face |
| Nominal / design torque | 18 Nm / 36 Nm (2×) |
| Real limit | **lobe contact stress**, not radial disengagement: 85 MPa Hertz at 36 Nm on PA-CF (yield ≈ 90 MPa → 41 Nm) |
| Radial-disengagement (skip) torque | not reached below 250 Nm in the elastic model (margin ≥ 0.44 mm at every torque up to 250 Nm); a cantilevered-pin printed-ring reference design skips at 26 Nm in the same model |
| Bearings | 6809 (yours) rear carrier, **6816** (buy) output, 2 × 6902 eccentric, 1 × 6801 rear input, 13 × 608 (12 rollers + front journal) |
| Fasteners | 4 × M3×8, 8 × M3×20, 14 × M3 heat-set inserts (Ø4.0 × 7.0 hole) |
| Mesh efficiency (model) | 91 % greased (µ = 0.1), 84 % (µ = 0.2), 77 % dry → 0.45 Nm × 50 × ~0.85 ≈ 19 Nm out |

---

## 1. What is in this folder

```
scripts/cad.py             parametric build123d model (PARAMETERS block at the top) -> cad/*.step, *.stl
scripts/cycloid_profile.py epitrochoid + equidistant offset (the disc profile equations), undercut check
scripts/analysis.py        mesh geometry, pressure angle, load sharing, Hertz stress, skip model, bearing loads
scripts/check_geometry.py  39 programmatic checks on the EXPORTED geometry (lobe count, pin circle, walls, interference)
scripts/render_section.py  true CAD cross-section + axial stack diagram
cad/                       STEP + STL of every part, assembly.step/.stl, layout.json (every axial position)
analysis/                  results.json, geometry_check.json, PNG plots
renders/                   assembly_section.png, axial_layout.png
BOM.csv                    bill of materials
setup.ps1 / setup.sh       recreate the venv and rerun everything (Windows / Linux-macOS)
```

Re-running everything (Windows PowerShell, from this folder):

```powershell
.\setup.ps1          # creates .venv, installs build123d numpy scipy matplotlib trimesh, runs all scripts
```

or step by step: `python scripts\cad.py`, `python scripts\analysis.py`, `python scripts\check_geometry.py`,
`python scripts\render_section.py`.

---

## 2. Two places your brief did not survive the numbers

**Ring pins**: you were right. 51 rollers of Ø22 need ≥ 24 mm pitch → a Ø390 mm pin circle. Small hardened
dowels it is. I chose **Ø4 over Ø3**: bending stiffness ∝ d⁴, so a Ø4 pin over the 34.8 mm free span is
3.2× stiffer, and pin bending is the single largest compliance term left in this design
(`k_bend = 2 984 N/mm` vs `k_hertz = 70 000 N/mm`, seats 32 000 / 69 000 N/mm). Needle sleeves on Ø4
pins would push the pin OD to 7–8 mm, and there is no room for that at 51 pins on Ø112.

**608 as the eccentric bearing — no.** The 608 bore is Ø8. The eccentric journal that fills it is Ø8 offset by
e from a Ø5 motor shaft, so the printed wall is `4 − 2.5 − e ≈ 0.7 mm`. A crankpin arrangement (8 mm dowel
at offset e) fails as well: two discs need two opposed crankpins and both fall inside r < 5 mm. The eccentric
needs a bearing whose bore clears Ø5 + e + a real wall: **6902 (15×28×7)**, same 7 mm width as a 608, ~$2, cam
wall 4.4 mm. The 608s are used as the 12 output-pin rollers and the front input-shaft journal.

**6809 cannot be the output bearing with 608 rollers.** Its Ø45 bore only fits an output-pin circle
< Ø37, which cannot hold six Ø22 rollers. It is used as the **rear carrier bearing** instead (inner race on a
Ø45 boss of the carrier rear plate, outer race in the motor plate). The front needs a bore larger than the
pin circle + pins (Ø82): **6816 (80×100×10)**, the one part you have to buy that is not a $2 bearing. Two
grounded bearings 51 mm apart give the carrier real moment stiffness (160 N reaction at 36 Nm from the
internal couple; external loads on top of that).

---

## 3. Design decisions and the calculations behind them

### 3.1 Why 50:1 with 51 pins means shallow lobes, and what that implies
The ring pitch radius is `Zp·e`. With the pin circle fixed at R = 56 and the curtate ratio
`K = e·Zp/R` limited to < 1 (undercut), **e < 1.1 mm**. Lobe height is 2e. That is the whole game: a
50:1 single stage on a Ø112 pin circle has ~1.6 mm of lobe, so a printed drive skips as soon as its
radial compliance eats a millimetre. The design therefore goes after every compliance term.

K sweep at 36 Nm (`analysis/results.json → K_sweep_at_design_T`):

| K | e (mm) | lobe (mm) | ρ_min convex (mm) | Hertz p_max (MPa) | eccentric brg (N) | min contact depth (mm) |
|---|---|---|---|---|---|---|
| 0.60 | 0.66 | 1.32 | 4.39 | 81.5 | 788 | 0.36 |
| 0.70 | 0.77 | 1.54 | 3.92 | 82.1 | 740 | 0.45 |
| **0.75** | **0.82** | **1.64** | **3.63** | **85.1** | **724** | **0.49** |
| 0.80 | 0.88 | 1.76 | 3.29 | 90.3 | 711 | 0.52 |
| 0.85 | 0.93 | 1.87 | 2.89 | 103 | 696 | 0.55 |
| 0.90 | 0.99 | 1.98 | 2.39 | 171 | 692 | 0.57 |

Higher K = deeper lobes and lower bearing load but sharper lobe tips (ρ_min falls towards the pin radius
2.1 → contact stress explodes). K = 0.75 is the knee. The min convex radius of curvature (3.63 mm) is 1.7×
the offset distance, so there is no undercut (`cycloid_profile.undercut_check`).

### 3.2 Profile
`cycloid_profile.py`: base curve `x = R cos s − e cos(Zp s)`, `y = R sin s − e sin(Zp s)` (the epitrochoid a
pin centre traces in the disc frame — this is what cycloidal-drive literature calls the hypocycloid disc
curve), offset inward along the normal by `d = Rr + c` with **c = 0.10 mm**. The kinematics used to derive it
(`C = e·u(θ)`, `φ = −θ/N`) are the same ones the analysis uses, and `analysis.py` verifies that every pin sits
at exactly `Rr + c` from the profile at every input angle (max error 28 nm). `check_geometry.py` confirms the
exported STEP follows the equations within 5 µm and has 50 lobes.

### 3.3 Load path (theta = 0, cam A points +x)
disc lobes → 51 pins (simply supported, both ends press-fit) → cup floor (15 mm solid disc) and front plate
(19.8 mm annulus, r 50–69) → 8 × M3 into shell inserts. The disc's reaction goes disc bore → 6902 → cam →
shaft → 6801 (rear, in the cup floor) and 608 (front, in the flange). Output: disc holes → 608 rollers → Ø8
dowels (both ends press-fit: carrier rear plate 8 mm + flange 11.2 mm) → flange → 6816 / 6809 → housing.

### 3.4 Ovalisation, not hoop tension — and why bands do not help here
A ring loaded outward on two opposite sides (that is what two discs at 180° do to the pin ends) bends;
thin-ring theory gives `w_n = q_n R⁴ / (E I (n²−1)²)` per Fourier harmonic, which the script evaluates with
the actual pin-load pattern (`ring_deflection`). Results at 36 Nm: **front plate 0.048 mm, cup floor 0.005
mm**. The in-plane bending stiffness `E·h·t³/12` of the front plate (t = 19 mm radial, h = 19.8 mm) is
4.5e7 N·mm². Adding a metal band around it:

| band | added EI | gain |
|---|---|---|
| 6061 tube ring, 3 mm wall | 3.1e6 | +7 % |
| 6061 ring, 6 mm wall (lathe job) | 2.5e7 | +54 % |
| 1 mm steel band (hose clamp) | 3.3e5 | +0.7 % |
| 2 mm steel band | 2.6e6 | +6 % |
| steel wire wrap | hoop only | ~0 % in bending |

So: **no band**. Thick diaphragm plates in a stiff material are the fix; a wrap or hose clamp is cosmetic.
If you want more, the lever is the front plate material (PA-CF E ≈ 4 GPa vs PETG 1.9 GPa) and its radial width.
The rear "ring" is the motor plate itself, which is why the cup is one piece.

### 3.5 Two discs vs three
Modelled both (`skip.3disc_PACF`, `ripple_3disc`). Findings:
- Torque ripple: in a quasi-static frictionless model both are zero (`F_e = T/(N·e)` exactly). Real ripple
  comes from friction and has the same period for 2 or 3 discs. Net radial force on the shaft: 9.8 N (2
  discs) vs 9.9 N (3 discs) — both already balanced.
- Eccentric indexing accuracy is **not** the constraint: a phase error δθ moves the disc centre by e·δθ, so 2°
  costs 0.03 mm — the same order as printer XY error. With a one-piece printed shaft it is the printer's
  accuracy anyway.
- Three discs cut per-disc torque by ⅓ → contact stress ×0.82, but add 13 mm of stack, ~40 % more pin
  bending (span 48 mm) and, crucially, the one-piece same-size-cam shaft is impossible: the middle cam's
  bearing cannot pass a neighbouring cam at 120° (centre distance e·√3 = 1.4 mm with equal radii). You would
  need stepped cams (three different disc bores) or a split, keyed shaft.
- The same contact-stress gain is had by 2 discs at 12 mm instead of 10 mm (`disc_thickness_sweep`:
  110 → 93 → 85 → 78 MPa for 7 / 10 / 12 / 14 mm). **Decision: two discs, 12 mm.**

### 3.6 Pressure angle across the mesh cycle (`analysis/pressure_angle_pins.png`, `pressure_angle_cycle.png`)
Definition used: angle between the contact normal (which passes through the pitch point) and the tangential
direction about the disc centre at the contact; radial (disengaging) component = F·sin α. The mesh pattern
repeats every pin pitch (2π/51 of input), so the sweep covers one pitch. At the max-lever pin α = **42.2–42.4°**
over the whole cycle (nearly constant — cycloids are like that); lever-weighted mean over the loaded set
**54°**. Consequence: the radial force exceeds the tangential force, `F_r/F_t = tan 54° = 1.37`, and the
eccentric bearing sees `F_t / cos α ≈ 746 N` per disc by hand, **727 N** in the model.

Hand check of the tangential force: `F_t = T_disc / (N·e) = 18 000 / (50 × 0.82) = 439 N`. The effective
torque radius of a cycloidal mesh is the disc pitch radius N·e = 41 mm, not the pin circle.

### 3.7 Load sharing and contact stress (`load_sharing_design.png`)
Compliant model: each pin is a spring (Hertz + bending + two seats in series, 2 531 N/mm), each roller 12 400
N/mm, the eccentric bearing 38 000 N/mm; unknowns are disc wind-up, disc centre displacement and the drive
force; solved as a linear complementarity problem, then Newton-refined against the exact geometric gaps of
the displaced profile. At 36 Nm: **10 pins loaded, F_max = 69 N**, rollers 325 N max, wind-up 0.18°, backlash
take-up 0.26 mm tangential (≈ 0.35° at the output — expect that as free play with c = 0.10).

Hertz line contact, steel Ø4 on the local lobe curvature, 12 mm long: **p_max = 85 MPa at 36 Nm, 63 MPa at
18 Nm**. PA-CF compressive yield ≈ 90 MPa, PC ≈ 70 MPa. p_max ∝ √T, so:

| limit | torque |
|---|---|
| PA-CF lobe yield onset (90 MPa) | ~41 Nm |
| PC lobe yield onset (70 MPa) | ~25 Nm — **PC discs are not adequate at 2×** |
| flank flow / functional skip (~180 MPa) | ~160 Nm (steel pins still at 430 MPa bending, fine) |
| radial disengagement, elastic model | > 250 Nm (min contact depth 0.51 mm, separation 0.06 mm at 36 Nm) |

**This is where the real limit is: the printed lobe surface, at about 40 Nm continuous for PA-CF.** Above
that the flanks creep, clearance grows, backlash grows, and eventually you get the skip you are used to — by
a different mechanism than your current drive. Reference case run through the same model — Ø4 pins
cantilevered 20 mm from one plate, thin printed PETG ring, slip-fit pins, loose eccentric bearing — loses its
first pin at **11 Nm and skips at 26 Nm** with 1.2 mm of radial separation at 18 Nm. That is your current
failure, reproduced.

### 3.8 Other loads at 36 Nm (`results.json → loads`)
| item | value | capacity |
|---|---|---|
| 6902 eccentric bearing | 727 N radial per disc | C0 ≈ 2.2 kN, C ≈ 4.4 kN (verify) |
| 608 roller | 325 N | C0 ≈ 1.37 kN |
| 6809 / 6816 carrier reaction from the internal couple | 160 N | C0 6–12 kN |
| shaft support reaction (6801 / 608) | 234 N | fine |
| shaft bending stress, collar / cam | 18 / 38 MPa (conservative bound) | printed PC ~60 MPa Z; PA-CF 40–60 MPa Z |
| ring pin bending | 96 MPa | hardened dowel ≫ 1 000 MPa |
| output pin bending | 42 MPa | fine |
| output pin seat pressure (carrier) | 5 MPa | fine |
| motor shaft torque | 0.72 Nm; D-flat pressure 7.5 MPa | fine |
| housing bolt shear | 70 N per bolt | fine |

---

## 4. Tolerance strategy

Printed holes come out small and printed pegs large; the numbers below are **nominal CAD dimensions**
already biased for a press where a press is wanted. Print the two coupons first and shift the `FIT_*`
parameters by whatever your printer needs.

| fit | CAD nominal | intent | parameter |
|---|---|---|---|
| Ø4 ring pin in cup floor | Ø3.95 × 8 deep | firm press (−0.05) | `FIT_PIN_PRESS_REAR` |
| Ø4 ring pin in front plate | Ø3.98 × 18 deep | light press (−0.02): plate is drawn on by the 8 bolts | `FIT_PIN_PRESS_FRONT` |
| Ø8 output pin in carrier / flange | Ø7.95 through / × 12 deep | press (−0.05) | `FIT_OUTPIN_PRESS` |
| bearing OD in printed pocket (608, 6801, 6809, 6816) | OD − 0.05 | press | `FIT_BRG_OD_IN_PLASTIC` |
| 6902 OD in disc bore | 27.95, 1.0 mm ledge, bearing centred in the 12 mm disc | press | `DISC_BRG_CLR` |
| printed journal in bearing bore (cams Ø15, front journal Ø8, carrier boss Ø45, flange boss Ø80) | ID + 0.03 | press | `FIT_BRG_ID_ON_PLASTIC` |
| M3 clearance | Ø3.4 | clearance | `FIT_SCREW_CLR` |
| M3 heat-set insert (4.0 OD × 5.7) | Ø4.0 × 7.0 deep, Ø4.6 entry countersink | insert | `INSERT_HOLE_D/DEPTH/CSK_D` |
| motor pilot | Ø22.3 × 2.3 deep | 0.3 clearance | `PILOT_CLR` |
| motor D-bore | Ø5.1, flat 0.45 deep, 13 mm long | slip | (in cad.py) |
| disc ↔ carrier thrust faces | 0.4 mm each face | axial clearance | `DISC_AX_GAP` |
| disc A ↔ disc B | 1.0 mm | axial clearance | `DISC_DISC_GAP` |
| disc B ↔ front plate | 0.9 mm; disc A ↔ cup floor 1.4 mm | clearance | derived |
| roller hole in disc | Ø23.74 = 22 + 2e + 0.10 | 0.05 radial slop | `HOLE_CLR` |
| flange body ↔ front plate bore | 1.5 mm radial | clearance | `FLANGE_BODY_OD` |
| disc tips ↔ shell | 3.2 mm radial | clearance | derived |
| 6816 outer race ↔ lip | 0.3 mm axial play (carrier axial float; shim with a printed 0.2 mm ring if you want zero) | `LIP_GAP` |

Why the pin seats must be press fits: a slip-fit pin adds its full clearance to radial separation on the
loaded side. The model shows the design tolerates 0.05 mm of it, but it is free margin you should not spend.

### Fit-test coupons (`cad/coupon_*.stl`)
- `coupon_pins_inserts` (print in the cup/front-plate material and profile): Ø4 holes 3.90 / 3.95 / 4.00 /
  4.05 / 4.10 (row 1), Ø8 holes 7.90 … 8.10 (row 2), M3 insert holes 3.9 / 4.0 / 4.1 / 4.2 × 7 deep and an
  M3 clearance hole (row 3). Pick the Ø4 hole that takes a dowel with a firm thumb/arbor press and no
  cracking → set `FIT_PIN_PRESS_REAR` to (that hole − 4.00); use one step larger for the front plate.
  Same for Ø8. Melt an insert into each of the four holes: the one that goes in square with a little plastic
  rising around the rim and pulls out only with the thread stripped is your `INSERT_HOLE_D`.
- `coupon_bearings` (print in the disc/flange material): 608 pockets 21.85 / 21.95 / 22.05, a 6902 pocket
  27.95, a 6801 pocket 20.95, pegs 7.95 / 8.00 / 8.05 / 8.10 (608 bore) and 14.95 / 15.00 / 15.05 (6902
  bore). Pick press fits that seat with an arbor press or a vice and soft jaws, not a hammer.

### Profile clearance `PROFILE_CLR` (start at 0.10)
- Binds / stiff / warm after a minute at no load → increase by 0.03 (0.13, 0.16). Binding is almost always
  the disc tips touching the pins on the **unloaded** side, i.e. real e or R is off; before opening the
  clearance, measure the printed pin circle with the pins in (`R_PIN` is what the plate holds, not what the
  slicer thinks) and the disc across lobe tips (should be 109.44).
- Skips / lots of backlash → decrease. `clearance_sweep_at_design_T`: 0.10 → 10 pins share, 69 N max; 0.05
  → 12 pins, 58 N max, backlash 0.20 mm; 0.03 → 14 pins, 51 N. Below ~0.05 you are inside the H2D's XY
  accuracy and load sharing stops improving in practice.

---

## 5. Print settings

0.4 mm nozzle everywhere. Nothing needs supports. "PA-CF" = Bambu PAHT-CF / PA6-CF class (hardened nozzle,
dry filament, enclosure; fine on the H2D, use the P1P only with a hardened nozzle and a dried spool).

| part | material | why | orientation | walls | top/bottom | infill | layer |
|---|---|---|---|---|---|---|---|
| rear_cup | **PA-CF** (PC acceptable, PETG/PLA no — motor heat + it is the rear diaphragm) | stiffness E; 60–80 °C motor face | open side up, back face on bed; pilot recess is a 45° cone, bearing pockets/pin holes open upward | 5 | 6 / 6 | 40 % gyroid | 0.2 |
| front_plate | **PA-CF** (PC −40 % stiffness, still OK) | it is the front diaphragm | front face on bed, pin holes + bearing bore open upward; bolt counterbores bridge 1.3 mm — fine | 5 | 6 / 6 | 40 % gyroid | 0.2 |
| disc ×2 | **PA-CF, no substitute** (PC yields at 25 Nm) | 85 MPa Hertz on the lobes; creep | flat, bearing-ledge side (Ø26 hole) on the bed, Ø28 pocket opening upward | 6 (2.4 mm — the whole lobe is wall) | 6 / 6 | 100 % | 0.16 |
| eccentric_shaft | **PC** (best Z-strength) or PA-CF | 38 MPa bending across layers at 36 Nm | vertical, rear end (Ø12) on the bed; 45° cones under the steps are modelled | 100 % walls | – | 100 % | 0.12 |
| carrier_rear_plate | PA-CF (PC OK) | pin seats, 5 MPa | boss up, plate on bed | 5 | 6 / 6 | 60 % | 0.2 |
| output_flange | PA-CF or PC | pin seats + your load | front face up, rear face (pin holes, 608 pocket) on the bed | 5 | 6 / 6 | 60 % | 0.2 |
| washers ×18 | anything rigid (PETG fine) | spacers | flat | 3 | – | 100 % | 0.2 |
| coupons | same as the parts they calibrate | – | as modelled | as the part | | | |

Which parts genuinely need the expensive material: **the two discs** (contact stress) and, for stiffness,
**both plates** (the whole point of the design). The shaft wants PC for isotropy. Everything else is
indifferent; PETG works for the carrier and flange at nominal torque.

Mass (solid volumes × 1.2 g/cm³ at the infills above, roughly): cup ~260 g, front plate ~110 g, disc 2 × 85 g,
flange ~110 g, carrier ~50 g. Print time is dominated by the cup (~7 h on the H2D at 0.2).

---

## 6. Assembly order (nothing is glued; every fit is a press, do them with an arbor press or a vice)

Fasteners, every one of them: **4 × M3×8** (motor), **8 × M3×20** (front plate), **14 × M3 inserts**.
Your load: M3 × (your part thickness + 5).

1. **Inserts**: 8 in the shell top of the cup, 6 in the flange face. Soldering iron at 220–240 °C for PA/PC.
2. **Motor to cup**: drop the motor pilot into the recess on the back of the cup, run the 4 × M3×8 in from the
   inside through the counterbores (a ball-end 2.5 mm key through the 6809 pocket notches). Heads end up
   below the 6809 pocket floor. Check the screw does not bottom in the motor before torquing; use M3×6 if it does.
3. **Cup bearings**: press the 6801 into its pocket (outer race, Ø21), then the 6809 into the Ø58 pocket (outer
   race; the 0.3 mm you feel it sit proud of nothing is the LIP_GAP float). Press the 51 ring pins into the
   floor holes, chamfered end first, until they bottom (8 mm in). A drill press with the chuck open as a
   pusher keeps them square; check squareness with a square against the floor — a pin leaning 1° is 0.6 mm
   off at the front plate and will fight the front holes.
4. **Eccentric shaft**: press the 6902 for **cam A on from the rear** (over the Ø12 journal), the 6902 for
   **cam B on from the front** (over the Ø8 journal); both butt against the collar. Press the 608 front journal
   bearing into the flange pocket now.
5. **Indexing check (before anything else goes in)** — see §7.
6. **Carrier**: press the six Ø8 pins into the carrier rear plate (8 mm through), then onto each pin:
   washer_end 2.9 → 608 → washer_mid 6.0 → 608 → washer_end 2.9. Grease the pins lightly.
7. **Shaft into cup**: slide the shaft's D-bore over the motor shaft and its Ø12 journal into the 6801 (the
   shaft end stays 0.5 mm off the pocket floor). Slide the carrier's Ø45 boss into the 6809 (press) — its
   plate now sits 0.5 mm above the floor.
8. **Disc A** (pocket side facing forward) over cam A's bearing (press the disc onto the outer race), rollers
   into the six holes as you lower it. Grease the lobes. **Disc B** rotated so that its lobes are 180/50 =
   3.6° out of phase with disc A (any lobe of B sits opposite a root of A across the centre); it is the
   *holes* that force this — with the rollers already on the pins, disc B only drops on in the right phase.
   Press disc B onto cam B's bearing, pocket side facing rearward.
9. **Flange**: press the flange onto the six pin ends (11.2 mm) with the front 608 over the shaft journal; press
   the 6816 onto the flange boss (inner race).
10. **Front plate**: start all 51 pins into the front-plate holes and the 6816 into the bore at once. Use the
    8 × M3×20 to draw the plate down evenly (quarter turns, star pattern). It is a light press on 51 pins so
    expect real force; if it will not move, one pin is bent or leaning — find it, do not force it.
11. Turn the motor shaft by hand: two full turns should feel uniform. Free-running current on the driver at
    low speed should not rise more than ~15 % over the motor alone.

### 7. Checking the eccentric indexing before final assembly
The cams are one print, so the 180° is baked in; what you are checking is that the print is round and the
two cam axes are where the equations put them.
- With both 6902s pressed on, set the shaft between V-blocks (or spin it in the 6801 in the cup) with a dial
  indicator on each bearing's outer race in turn. Runout of each should read **2e = 1.64 mm** total, and the
  two must peak **180° apart within ~3°** (that is 0.04 mm of disc centre error, `e·δθ`, below printer error).
  Also check that the two sinusoids have the same amplitude within 0.05 mm: unequal e means one disc carries
  more than half the torque (a difference in e shifts torque toward the disc with the larger e; keep it inside the profile clearance).
- Faster: put the shaft with bearings in the cup with no discs, lay disc A on its bearing, rotate the shaft:
  the disc should walk around the pins with no tight spot; repeat with disc B alone on cam B. A tight spot
  that moves with the shaft is cam runout; one that stays with the housing is a leaning pin.

---

## 8. Motor shaft length
`MOTOR_SHAFT_L = 22`. The shaft passes through the 15 mm cup floor and enters the eccentric shaft's D-bore from
z = 10.5; the bore is `MOTOR_SHAFT_L + 0.5 − 10.5 = 12 mm` deep (`d_bore_depth` in layout.json). The D-bore
can go as deep as z ≈ 38 (through the collar, 4 mm of cam wall around it) so shafts up to ~37 mm work by
changing that one parameter. Shorter than ~19 mm leaves < 8 mm of engagement and the assertion in
`check_layout` will tell you — either accept a Ø5 coupling sleeve or thin the floor (`FLOOR_T`) to 13.

---

## 9. The three things most likely to go wrong

1. **The lobe flanks creep and backlash grows.** 85 MPa Hertz at 36 Nm is at PA-CF's yield; at nominal
   (63 MPa) it is fine but plastics creep at 40–60 °C. Symptoms: rising backlash, then a "clunk" under
   reversing load, eventually skipping under peak load. Fixes in order: keep the discs greased (µ from 0.3 →
   0.1 halves the heat), cap peak torque at ~30 Nm, reprint discs at 14 mm (`DISC_T`: −8 % stress) or with
   `PROFILE_CLR = 0.05` (−6 %), or — the real fix — nylon/POM machined discs.
2. **The printed eccentric shaft.** It sees 727 N per cam and up to ~38 MPa bending across layer lines, and
   the 6902 inner races are pressed on a plastic journal that will relax with heat. Symptoms: growing
   runout, one disc taking the load (uneven wear on the two discs), cracked cam at the collar. Print it in
   PC at 100 %, 0.12 mm layers, vertical; check runout as in §7 after the first 50 hours. Upgrade path: a
   turned steel shaft to the same drawing (the printed one is the STEP).
3. **Pin seats.** If the front plate's 51 holes come out even 0.05 mm large, the plate no longer holds the
   pin ends and the pin becomes a cantilever on the loaded side — that single change takes the model from
   "no skip below 250 Nm" halfway to your current drive. Use the coupon; if a hole is loose, do not
   "fix" it with glue in the pin hole — reprint with `FIT_PIN_PRESS_FRONT` one step tighter.

Honourable mentions: the motor screw heads are inside the cup (motor cannot be swapped without pulling the
carrier — I chose this over a separate adapter plate to keep the rear diaphragm one solid piece); the 6816
has 0.3 mm of designed axial float; grease migrates out of open 608s — use ZZ.

---

## 10. Things I am not certain about (please verify)
- Bearing ratings are from memory of SKF/NSK catalogue values (6902: C 4.36 / C0 2.24 kN; 608: 3.45 / 1.37;
  6809: 6.6 / 6.1; 6816: ~12/12.7 kN). Marked "verify" in the BOM.
- Printed-material properties: E = 4.0 GPa / σ_c ≈ 90 MPa for PA-CF, 2.3 GPa / 70 MPa for PC are XY
  datasheet-class numbers; Z-direction strength of the shaft is the least certain number in this package.
- The Hertz calculation treats the lobe as a half-space; a 12 mm wide PA-CF lobe is close enough, but
  local plastic flow at the first-print surface roughness will make the first hours of running "bed in".
- Insert geometry assumes the common 4.0 mm OD × 5.7 mm brass M3 insert. Other inserts (CNC Kitchen short =
  4.0 × 4.0, or 4.6 OD types) change `INSERT_HOLE_D/DEPTH`.
- NEMA 17 bolt pattern is taken as 31.0 mm **square spacing** (holes on Ø43.8, 45°). If yours is really on a
  Ø31 circle, set `MOTOR_HOLE_SP` accordingly and re-run; the counterbores are checked against the pockets.
- Efficiency comes from a sliding-friction model at the pin contacts only; add 2–4 % for bearings/seals.
