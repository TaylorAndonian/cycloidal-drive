# Belt-drive conversion — Ø80 30:1 cycloidal joint

Motor moves alongside the gearbox, driving it through a 1:1 2GT belt. **Overall ratio stays 30:1**, so nothing in your control setup changes.

## What stays, what changes

**Unchanged — keep the parts you have:** housing, both discs, cam B, rear ring, output flange, output cap, front retainer, all three roller sleeves, shaft spacer, and every bearing, pin and shoulder bolt inside the gearbox.

**Reprint these three, plus two small ones:**

| Part | What it does |
|---|---|
| `back_plate_belt` | Replaces `back_plate`. Gearbox-side features are identical (register ring, 6704 boss, 6 tie holes). The NEMA pattern and pilot are gone. It grows a **rear tower holding two 608 bearings** — this is what stops belt tension from cocking the input axle. |
| `input_shaft_belt` | Replaces `input_shaft`. The whole front half (cam A, D-drive, nose) is unchanged. The motor bore is gone, so the shaft is now **solid** — stronger than before. The back end becomes a Ø8.125 journal running through both 608s, ending in a pulley seat with a flat for the set screw. |
| `motor_bracket` | New. Hangs the motor beside the gearbox off three of your tie screws. Slotted motor holes give ±5 mm of travel, and there's a slot for the idler. |
| `tower_bearing_spacer` | New. A 4 mm ring that sets the gap between the two 608s. |
| `idler_standoff` | New. A 9 mm tube that puts the idler bearing on the belt centre line. |

## Layout

The motor's shaft points **backwards** and its body lies alongside the gearbox, overlapping it in height. Both pulleys sit in a plane 23 mm behind the back plate.

- Centre distance **70.0 mm**, 20T both ends, **180 mm** 2GT closed loop. The maths: for equal pulleys, belt length = 2 × centres + π × 12.732, so 2 × 70 + 40 = 180 exactly.
- Motor slots give 65–75 mm of centre distance, which covers 170–190 mm belts if you'd rather use a different one.
- The motor body's nearest face sits 8.9 mm clear of the Ø80 housing.
- **Height: 82.5 mm total** (50.5 gearbox + 32 of tower, pulley and belt plane) against 98.5 stacked. So about **16 mm saved** — the bigger win is that the motor's weight is now beside the joint instead of hanging off the back, and a longer motor no longer adds any height.

## Two ways to tension, both included

1. **Motor slots** (primary). Slide the motor away from the gearbox and tighten the 4 M3 screws. The slots run along the line of centres.
2. **Idler** (secondary). The F695 runs on the back of the belt's lower run, on a slot that travels from fully clear (y −19.5) to about 2 mm into the belt (y −5.5). Use it to fine-tune without moving the motor, or leave it off entirely if the slots are enough.

Set tension so the belt doesn't skip teeth when you stall the output by hand, and no tighter. Over-tensioning just loads the 608s and the motor bearings.

## What to buy

| Qty | Part | Notes |
|---|---|---|
| 1 | 2GT closed-loop belt, **180 mm** (90 teeth), 6 mm wide | Steel core preferred. Fibreglass works but has less margin. |
| 1 | 2GT **20T pulley, 5 mm bore**, for 6 mm belt | Motor. You likely have these from Voron spares. |
| 1 | 2GT **20T pulley, 8 mm bore**, for 6 mm belt | Gearbox shaft. Hub must be **Ø19 or less** — a bigger hub fouls the tower. |
| 1 | **F695-2RS** flanged idler (5×13×4) | Or a 625ZZ (5×16×5) if you have spares from the Ø100 build — the slot takes either. |
| 1 | M5 × 30 bolt, nut, 2 washers | Idler axle. |
| 2 | **608** | From your stock — the two tower bearings. |

Screws: 3 × M3×25 (the tie screws that also hold the bracket), 3 × M3×12 (the other tie screws), 4 × M3×12 for the motor.

**Belt tension at full motor torque is about 71 N**, which sits inside a 6 mm steel-core 2GT belt's working range but isn't generous. If you want more margin, a 9 mm belt and 9 mm pulleys drop in at the same centre distance.

## Numbers I checked

- Every printed part is a single connected solid; no unexpected interference in the assembly.
- Belt tension 71 N → 168 N on the 608 nearest the pulley, 97 N on the other. Both are far below the 608's 1960 N static rating.
- Bending in the Ø8 journal: 799 N·mm, about 16 MPa. PA-CF across layers is roughly 40 MPa, so about 2.5× margin. Print the shaft standing up, motor end down, 100% infill.
- Belt length checks out exactly at 180.0 mm for 70 mm centres.

## Assembly

1. Press the two 608s into the tower **from the back**: first bearing, then the 4 mm spacer, then the second. The first one stops against the Ø15.6 hole.
2. Build the gearbox exactly as before, using `back_plate_belt` and `input_shaft_belt`. The shaft's rear journal passes through both 608s as the back plate goes on.
3. Fit the bracket behind the back plate: the three standoffs land on the tie-screw bosses, held by the 3 long M3×25 screws that carry on into the housing. The other three tie screws stay M3×12.
4. Bolt the motor to the bracket's **rear** face, screws from behind, centred in the slots for now.
5. Slide both pulleys on so their grooves line up 23 mm behind the back plate's rear face. Set screws onto the flat on the gearbox shaft.
6. Fit the belt, slide the motor outward until tensioned, tighten. Add the idler if you want finer control: M5 bolt → bracket slot → standoff → bearing → washer → nut.
7. Turn the motor by hand and check the output turns smoothly, then check the belt doesn't rub the bracket anywhere.
