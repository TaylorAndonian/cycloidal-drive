"""
Generates every part of the 30:1 Ø80 cycloidal reducer as STEP + STL,
plus an assembled STEP.

    python cad.py            -> ./out/*.step, ./out/stl/*.stl
"""
import math
import os
from build123d import *
from params import *

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
STL = os.path.join(OUT, "stl")
os.makedirs(STL, exist_ok=True)

PROFILE_PTS = 720


# --------------------------------------------------------------------- helpers
def cyl(d, h, z=0.0, x=0.0, y=0.0):
    """Cylinder with its base at z."""
    return Pos(x, y, z) * Cylinder(
        d / 2, h, align=(Align.CENTER, Align.CENTER, Align.MIN))


def disc_profile(phase_deg=0.0, ecc=ECC, R=R_PIN_CIRCLE,
                 Rr=R_PIN + DISC_CLEARANCE, N=N_PINS, n=PROFILE_PTS):
    """Epitrochoid-with-roller cycloidal disc profile (closed point loop).

    Generated about the disc's own centre.  `phase_deg` rotates the lobes
    relative to the output-pin pattern; disc B needs half a lobe pitch
    (180/N_LOBES deg) because its cam sits 180 deg round from disc A's.
    """
    ph = math.radians(phase_deg)
    pts = []
    for i in range(n):
        t = 2 * math.pi * i / n
        psi = math.atan2(math.sin((1 - N) * t),
                         (R / (ecc * N)) - math.cos((1 - N) * t))
        x = R * math.cos(t) - Rr * math.cos(t + psi) - ecc * math.cos(N * t)
        y = -R * math.sin(t) + Rr * math.sin(t + psi) + ecc * math.sin(N * t)
        pts.append((x * math.cos(ph) - y * math.sin(ph),
                    x * math.sin(ph) + y * math.cos(ph), 0.0))
    return pts


def ring_pin_pockets(depth, z=0.0, clr=PIN_POCKET_CLR):
    return (PolarLocations(R_PIN_CIRCLE, N_PINS)
            * cyl(RING_PIN_D + clr, depth, z=z))


# ----------------------------------------------------------------------- parts
def make_disc(phase_deg=0.0):
    face = make_face(Polyline(*disc_profile(phase_deg), close=True))
    p = extrude(face, DISC_T)
    # bearing pocket: 2 x 6801 side by side, retained by a shoulder
    p -= cyl(DISC_SHOULDER_BORE, DISC_T)
    p -= cyl(DISC_BORE, DISC_T - DISC_SHOULDER_T, z=DISC_SHOULDER_T)
    # output pin holes -- oversize by 2e so the pin can orbit
    p -= PolarLocations(OUT_PCD_R, N_OUT_PINS) * cyl(OUT_HOLE_D, DISC_T)
    return p


def make_sleeve():
    p = cyl(HOUSING_OD, SLEEVE_T)
    p -= cyl(2 * R_PIN_CIRCLE, SLEEVE_T)
    p -= ring_pin_pockets(SLEEVE_T)
    p -= PolarLocations(TIE_PCD / 2, N_TIE) * cyl(TIE_D, SLEEVE_T)
    p = chamfer(p.edges().filter_by(GeomType.CIRCLE)
                .group_by(SortBy.RADIUS)[-1], 1.0)
    return p


def make_motor_plate():
    t = MOTOR_PLATE_T
    p = cyl(HOUSING_OD, t)
    # motor side (z = 0)
    p -= cyl(NEMA17_PILOT_D, 2.0)
    p -= (PolarLocations(NEMA17_BOLT_PCD * math.sqrt(2) / 2, 4, start_angle=45)
          * cyl(NEMA17_BOLT_D, t))
    # rear support bearing seat, 2 mm in from the motor face
    p -= cyl(REAR_BRG[2], t - 2.0, z=2.0)
    # gearbox side (z = t)
    p -= ring_pin_pockets(1.5, z=t - 1.5)
    p -= (PolarLocations(TIE_PCD / 2, N_TIE)
          * cyl(TIE_INSERT_D, 5.0, z=t - 5.0))
    return p


def make_carrier():
    p = cyl(CARRIER_OD, CARRIER_T)
    p += cyl(BOSS_D, BOSS_LEN, z=CARRIER_T)
    total = CARRIER_T + BOSS_LEN
    # nose bearing seat in the front of the boss
    p -= cyl(NOSE_BRG[2], NOSE_BRG[3], z=total - NOSE_BRG[3])
    p -= cyl(18.0, total)
    # output dowel press fits
    p -= PolarLocations(OUT_PCD_R, N_OUT_PINS) * cyl(OUT_PIN_D - 0.05, CARRIER_T)
    # face plate inserts
    p -= (PolarLocations(BOSS_BOLT_PCD / 2, N_BOSS_BOLTS)
          * cyl(BOSS_INSERT_D, 8.0, z=total - 8.0))
    return p


def make_ring_plate():
    t = RING_PLATE_T
    p = cyl(HOUSING_OD, t)
    p -= ring_pin_pockets(1.5)                      # rear face
    p -= PolarLocations(TIE_PCD / 2, N_TIE) * cyl(TIE_D, t)
    p -= cyl(OUT_BRG[2], OUT_BRG[3])                # 6808 seat, rear
    p -= cyl(BOSS_D + 4.0, t)                       # front relief for the boss
    p = chamfer(p.edges().filter_by(GeomType.CIRCLE)
                .group_by(SortBy.RADIUS)[-1], 1.0)
    return p


def make_face_plate():
    t = FACE_PLATE_T
    p = cyl(FACE_PLATE_OD, t)
    p -= cyl(10.0, 1.5)                             # shaft nose relief
    p -= PolarLocations(BOSS_BOLT_PCD / 2, N_BOSS_BOLTS) * cyl(4.4, t)
    p -= PolarLocations(FACE_BOLT_PCD / 2, N_FACE_BOLTS) * cyl(FACE_BOLT_D, t)
    return p


def make_shaft():
    z0 = 2.0                                        # rear journal start
    p = cyl(CAM_JOURNAL_D, Z_SLEEVE - z0, z=z0)                       # rear jrnl
    p += cyl(COLLAR_D, 2.0, z=Z_SLEEVE)                               # collar 1
    zA = Z_DISC_A + DISC_SHOULDER_T
    p += cyl(CAM_JOURNAL_D, 10.0, z=zA, x=+ECC)                       # cam A
    zB = Z_DISC_B + DISC_SHOULDER_T
    p += cyl(COLLAR_D, zB - (zA + 10.0), z=zA + 10.0)                 # collar 2
    p += cyl(CAM_JOURNAL_D, 10.0, z=zB, x=-ECC)                       # cam B
    p += cyl(COLLAR_D, 1.0, z=zB + 10.0)                              # collar 3
    nose_z = zB + 11.0
    p += cyl(NOSE_JOURNAL_D, Z_RINGPL + NOSE_BRG[3] - 0.5 - nose_z, z=nose_z)
    # motor shaft bore with a D-flat
    p -= cyl(SHAFT_BORE_D, 20.0, z=z0)
    p -= Pos(0, SHAFT_BORE_D / 2 + 2.25, z0 + 10.0) * Box(10, 5, 20)
    # grub screw through collar 1
    p -= Pos(0, 0, Z_SLEEVE + 1.0) * Rot(90, 0, 0) * Cylinder(GRUB_D / 2, 20)
    return p



def make_coupon():
    """Fit-test coupon.  PRINT THIS FIRST -- it carries every press fit and
    clearance in the gearbox, so one 20-minute print tells you whether your
    X/Y compensation is right before you commit 200 cm3 of PA-CF."""
    L, W, T = 92.0, 26.0, 8.0
    p = Pos(0, 0, 0) * Box(L, W, T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    x = -L / 2 + 13.0
    # 6801 seat (eccentric + rear support)
    p -= cyl(ECC_BRG[2], 5.0, z=T - 5.0, x=x)
    x += 20.0
    # 608 seat (nose)
    p -= cyl(NOSE_BRG[2], T, x=x)
    x += 19.0
    # three ring-pin pockets at true pitch: one blind, two through
    pitch = 2 * math.pi * R_PIN_CIRCLE / N_PINS
    p -= cyl(RING_PIN_D + PIN_POCKET_CLR, 1.5, z=T - 1.5, x=x - pitch)
    p -= cyl(RING_PIN_D + PIN_POCKET_CLR, T, x=x)
    p -= cyl(RING_PIN_D + PIN_POCKET_CLR, T, x=x + pitch)
    x += 13.0
    # output dowel press fit
    p -= cyl(OUT_PIN_D - 0.05, T, x=x)
    x += 11.0
    # output pin orbit hole
    p -= cyl(OUT_HOLE_D, T, x=x)
    x += 11.0
    # M3 heat-set insert
    p -= cyl(TIE_INSERT_D, 5.0, z=T - 5.0, x=x)
    # M4 heat-set insert
    p -= cyl(BOSS_INSERT_D, 6.0, z=T - 6.0, x=x, y=-8.0)
    return p


# ------------------------------------------------------------------- assembly
def assembly():
    parts = {
        "motor_plate": (make_motor_plate(), Pos(0, 0, 0)),
        "sleeve":      (make_sleeve(),      Pos(0, 0, Z_SLEEVE)),
        "disc_a":      (make_disc(0.0),                Pos(+ECC, 0, Z_DISC_A)),
        "disc_b":      (make_disc(180.0 / N_LOBES),    Pos(-ECC, 0, Z_DISC_B)),
        "carrier":     (make_carrier(),     Pos(0, 0, Z_CARRIER)),
        "ring_plate":  (make_ring_plate(),  Pos(0, 0, Z_RINGPL)),
        "face_plate":  (make_face_plate(),  Pos(0, 0, Z_FACE)),
        "shaft":       (make_shaft(),       Pos(0, 0, 0)),
    }
    extras = {"coupon": make_coupon()}
    def fuse(shapes):
        out = shapes[0]
        for s in shapes[1:]:
            out += s
        return out

    placed = [loc * part for part, loc in parts.values()]
    # ring pins
    pin = cyl(RING_PIN_D, RING_PIN_LEN, z=Z_SLEEVE - 1.5)
    placed.append(fuse(PolarLocations(R_PIN_CIRCLE, N_PINS) * pin))
    # output dowels
    dowel = cyl(OUT_PIN_D, OUT_PIN_LEN, z=Z_CARRIER + CARRIER_T - OUT_PIN_LEN)
    placed.append(fuse(PolarLocations(OUT_PCD_R, N_OUT_PINS) * dowel))
    return parts, Compound(children=placed), extras


def main():
    parts, asm, extras = assembly()
    for name, (part, _) in parts.items():
        export_step(part, os.path.join(OUT, f"{name}.step"))
        export_stl(part, os.path.join(STL, f"{name}.stl"))
        bb = part.bounding_box()
        print(f"  {name:<12} Ø{max(bb.size.X, bb.size.Y):6.2f} x {bb.size.Z:5.2f}"
              f"   {part.volume/1000:7.2f} cm3")
    for name, part in extras.items():
        export_step(part, os.path.join(OUT, f"{name}.step"))
        export_stl(part, os.path.join(STL, f"{name}.stl"))
        bb = part.bounding_box()
        print(f"  {name:<12}  {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")
    export_step(asm, os.path.join(OUT, "assembly.step"))
    bb = asm.bounding_box()
    print(f"\n  ASSEMBLY      Ø{max(bb.size.X, bb.size.Y):6.2f} x {bb.size.Z:5.2f} mm")
    print(f"  ratio {N_LOBES}:1   files in {OUT}")


if __name__ == "__main__":
    main()
