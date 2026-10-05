"""
Belt-drive conversion for the v3 Ø80 30:1 cycloidal joint.

The gearbox internals are untouched: housing, discs, cams(B), rear ring, output
flange, cap, retainer, sleeves and all bearings/pins stay exactly as built.
Only three printed parts change or get added, plus one standoff:

  back_plate_belt   replaces back_plate  -- no NEMA pattern, gains a rear tower
                    carrying 2x 608 so belt tension cannot cock the input axle
  input_shaft_belt  replaces input_shaft -- solid (no motor D-bore); the rear
                    end becomes a Ø8 journal through both 608s + a pulley seat
  motor_bracket     new -- hangs the motor beside the gearbox on 3 of the tie
                    screws, with slotted motor holes (tension) and an idler slot
  idler_standoff    new -- puts the idler bearing in the belt plane

Belt stage: 1:1, 20T 2GT both ends, 180 mm closed loop, centres 70.0 mm.
Motor shaft points backwards (-Z); the motor body lies alongside the gearbox.

Run:  python cyc30_belt.py  ->  STEP files in ./step_belt/
"""
import os, math
import numpy as np
from build123d import *
import cyc30_v3 as V3
from cyc30_v3 import (cyl, ring, polar, dshaft, csk_m3, safe_chamfer,
                      OD_SHRINK, BORE_PRESS, BORE_SLIP, SHAFT_SLIP,
                      CLR_M3, INS_M3, TIE_ANG, TIE_R, REAR_BRG, Z_BP, Z_REG, Z_RR0)

OUT = os.environ.get("CYC_OUT", "step_belt")
os.makedirs(OUT, exist_ok=True)
C_, MIN, MAX = Align.CENTER, Align.MIN, Align.MAX

# =========================================================== belt stage
PITCH       = 2.0            # 2GT / GT2
T_MOTOR     = 20             # motor pulley teeth
T_SHAFT     = 20             # gearbox pulley teeth  (1:1 -> overall stays 30:1)
PD_MOTOR    = PITCH * T_MOTOR / math.pi      # 12.732
PD_SHAFT    = PITCH * T_SHAFT / math.pi
BELT_L      = 180.0          # standard 2GT closed loop, 90 teeth
CENTRES     = (BELT_L - math.pi * (PD_MOTOR + PD_SHAFT) / 2) / 2   # 70.0
BELT_PLANE  = -23.0          # z of the belt centre line; both pulleys slide to this
SLOT_TRAVEL = 5.0            # +/- on the motor slots

# ===================================================== input-shaft support
SUP_BRG = (8.0, 22.0, 7.0)   # 608 -- you have plenty
JOURNAL = SUP_BRG[0] + OD_SHRINK          # 8.125
TOWER_OD = 28.0
Z_T0 = 0.0                                 # tower runs from the back plate rearwards
Z_B1 = -SUP_BRG[2]                         # bearing 1: -7.0 .. 0  (stops on the Ø15.6 hole)
SPACER_T = 4.0                             # printed ring keeps the two bearings apart
                                           # (wider spacing = less shaft tilt under belt load)
Z_B2 = Z_B1 - SPACER_T - SUP_BRG[2]        # bearing 2: -15.2 .. -8.2
Z_TOWER_END = Z_B2 - 0.9                   # 0.9 lip retains bearing 2
Z_SHAFT_END = BELT_PLANE - 9.0             # pulley seat runs past the belt plane

# ===================================================== motor bracket
BR_T   = 6.0                 # bracket plate thickness
Z_BR1  = -6.0                # bracket front face (6 mm standoff behind the back plate)
Z_BR0  = Z_BR1 - BR_T        # -12.0, motor mounting face (motor body extends +Z)
NEMA_P = 31.0
BOSS_ANG = sorted(TIE_ANG, key=lambda a: abs(((a + 180) % 360) - 180))[:3]  # 3 nearest 0 deg
IDLER_BRG = (5.0, 13.0, 4.0)  # F695 (or 625ZZ 5x16x5 -- both fit the slot)
# belt runs are tangents at y = +/- PD/2 = +/- 6.37; the idler pushes the slack run inwards
IDLER_X, IDLER_Y = 42.0, -12.5   # slot travel +/-7 -> idler can clear the belt or press 2 mm in

def slot(d, length, h, z0, x, y, ang=0.0):
    """a slot of width d and total length `length`, centred on x,y, along `ang`"""
    s = Rot(0, 0, ang) * (Box(length - d, d, h, align=(C_, C_, MIN))
                          + Pos(-(length - d) / 2, 0, 0) * Cylinder(d / 2, h, align=(C_, C_, MIN))
                          + Pos((length - d) / 2, 0, 0) * Cylinder(d / 2, h, align=(C_, C_, MIN)))
    return Pos(x, y, z0) * s

# ============================================================== parts
def make_back_plate_belt():
    """v3 back plate, NEMA features removed, rear bearing tower added."""
    p = cyl(V3.OD, Z_BP)
    p = safe_chamfer(p, p.edges().filter_by(GeomType.CIRCLE).group_by(Axis.Z)[0]
                     .sort_by(SortBy.RADIUS)[-1:], 0.8)
    # ---- unchanged gearbox-side features
    p += ring(62.7, 58.5, Z_REG - Z_BP, Z_BP)                               # register
    p += ring(REAR_BRG[0] + OD_SHRINK + SHAFT_SLIP, 15.6, 5.2, Z_BP)        # 6704 boss
    p -= cyl(15.6, 20, -1)                                                  # shaft clearance
    for a in TIE_ANG:
        x, y = polar(TIE_R, a)
        p -= cyl(CLR_M3, 10, -1, x, y)
    # ---- new: rear tower with two 608s, pressed in from the back with a spacer between
    p += cyl(TOWER_OD, Z_T0 - Z_TOWER_END, Z_TOWER_END)
    p -= cyl(SUP_BRG[1] + BORE_PRESS, Z_B1 + SUP_BRG[2] - Z_B2, Z_B2)       # both seats, one bore
    p -= cyl(20.0, 2.0, Z_TOWER_END - 0.01)                                 # rear retaining lip
    p = safe_chamfer(p, p.edges().filter_by(GeomType.CIRCLE)
                     .group_by(Axis.Z)[0].sort_by(SortBy.RADIUS)[-1:], 0.6)
    return p

def make_input_shaft_belt():
    """v3 shaft, but solid at the back (no motor bore) and extended into a pulley seat."""
    s = cyl(14.0, V3.Z_CAMA0 - 1.5, 1.5)
    s += cyl(V3.COLLAR, 0.75, V3.Z_CAMA0, V3.ECC, 0)
    s += cyl(V3.CAM_J, V3.ECC_BRG[2] + 0.1, V3.Z_EB1, V3.ECC, 0)
    s += dshaft(V3.D_DRIVE, V3.Z_CAMB_J - 0.1 - V3.Z_DD0, V3.Z_DD0, V3.D_FLAT)
    s += cyl(V3.MID_D, V3.Z_NOSE0 - V3.Z_MID0, V3.Z_MID0)
    s += cyl(V3.NOSE_D, V3.Z_SHAFT_END - V3.Z_NOSE0, V3.Z_NOSE0)
    s -= cyl(INS_M3, 6.0, V3.Z_SHAFT_END - 6.0)          # front end-clamp insert (unchanged)
    # rear extension: journal through both 608s, then the pulley seat with a flat
    s += cyl(JOURNAL, 1.5 - Z_SHAFT_END, Z_SHAFT_END)
    s -= Pos(JOURNAL / 2 - 0.75, 0, Z_SHAFT_END) * Box(10, 10, (BELT_PLANE + 5.0) - Z_SHAFT_END,
                                                       align=(MIN, C_, MIN))   # set-screw flat
    return s

def make_motor_bracket():
    cx = CENTRES
    b = Pos(20.0, 0, Z_BR0) * Box(cx + 22.0 - 20.0, 46.0, BR_T, align=(MIN, C_, MIN))
    b = fillet(b.edges().filter_by(Axis.Z), 5.0)
    # gearbox end: a D-shaped flange over the back plate carrying 3 standoff bosses
    flange = (cyl(82.0, BR_T, Z_BR0) - cyl(31.0, BR_T + 2, Z_BR0 - 1)) \
             & (Pos(2.0, 0, Z_BR0) * Box(90, 90, BR_T, align=(MIN, C_, MIN)))
    b += flange
    for a in BOSS_ANG:
        x, y = polar(TIE_R, a)
        b += cyl(8.5, -Z_BR1, x=x, y=y, z0=Z_BR1)                  # standoff up to the plate
        b -= cyl(CLR_M3, 30, Z_BR0 - 1, x, y)
    # motor: slotted mounting holes + slotted pilot clearance, slots along the centre line
    b -= slot(23.0, 23.0 + 2 * SLOT_TRAVEL, 30, Z_BR0 - 1, cx, 0)
    for sx in (-1, 1):
        for sy in (-1, 1):
            b -= slot(CLR_M3, CLR_M3 + 2 * SLOT_TRAVEL, 30, Z_BR0 - 1,
                      cx + sx * NEMA_P / 2, sy * NEMA_P / 2)
    # idler: slot perpendicular to the belt run so it can be pushed into the belt
    b -= slot(5.3, 5.3 + 14.0, 30, Z_BR0 - 1, IDLER_X, IDLER_Y, 90)
    return b

def make_bearing_spacer():
    """goes between the two 608s in the tower"""
    return ring(21.9, 19.0, SPACER_T, Z_B2 + SUP_BRG[2])

def make_idler_standoff():
    """sits between the bracket's back face and the idler bearing, putting it on the belt line.
    Modelled at the origin for printing; the assembly places it in the slot."""
    h = Z_BR0 - (BELT_PLANE + IDLER_BRG[2] / 2)
    return ring(9.0, 5.3, abs(h), BELT_PLANE + IDLER_BRG[2] / 2)

# =============================================================== build
def pulley(teeth, z_centre, x=0.0, y=0.0):
    pd = PITCH * teeth / math.pi
    return (Pos(x, y, z_centre - 3.5) * Cylinder(pd / 2, 7, align=(C_, C_, MIN))
            + Pos(x, y, z_centre - 4.5) * Cylinder(pd / 2 + 1.2, 1, align=(C_, C_, MIN))
            + Pos(x, y, z_centre + 3.5) * Cylinder(pd / 2 + 1.2, 1, align=(C_, C_, MIN)))

def build():
    parts = {"back_plate_belt": make_back_plate_belt(),
             "input_shaft_belt": make_input_shaft_belt(),
             "motor_bracket": make_motor_bracket(),
             "idler_standoff": make_idler_standoff(),
             "tower_bearing_spacer": make_bearing_spacer()}
    for n, p in parts.items():
        export_step(p, f"{OUT}/{n}.step")
        print(f"  {n:20s} vol {p.volume/1000:7.2f} cm3  solids={len(p.solids())}  valid={p.is_valid}")
    return parts

def assembly(parts):
    items = []
    def add(s, label, color):
        s.label = label; s.color = Color(color); items.append(s)
    add(parts["back_plate_belt"], "back_plate_belt", "gray")
    add(parts["input_shaft_belt"], "input_shaft", "steelblue")
    add(parts["motor_bracket"], "motor_bracket", (0.3, 0.35, 0.4))
    add(Pos(IDLER_X, IDLER_Y, 0) * parts["idler_standoff"], "idler_standoff", "white")
    add(parts["tower_bearing_spacer"], "tower_bearing_spacer", "white")
    add(ring(SUP_BRG[1], SUP_BRG[0], SUP_BRG[2], Z_B1), "608_rear_1", "gold")
    add(ring(SUP_BRG[1], SUP_BRG[0], SUP_BRG[2], Z_B2), "608_rear_2", "gold")
    add(pulley(T_SHAFT, BELT_PLANE), "pulley_20T_shaft", "darkgray")
    add(pulley(T_MOTOR, BELT_PLANE, CENTRES, 0), "pulley_20T_motor", "darkgray")
    add(Pos(IDLER_X, IDLER_Y, BELT_PLANE - IDLER_BRG[2] / 2)
        * ring(IDLER_BRG[1], IDLER_BRG[0], IDLER_BRG[2]), "idler_F695", "gold")
    # gearbox body (reference only -- unchanged parts)
    add(cyl(V3.OD, V3.Z_FRONT - Z_BP, Z_BP), "housing_reference", (0.25, 0.3, 0.38))
    motor = Pos(CENTRES, 0, Z_BR1) * (Box(42.3, 42.3, 48, align=(C_, C_, MIN)))
    motor += Pos(CENTRES, 0, Z_BR1 - 2) * Cylinder(11, 2, align=(C_, C_, MIN))
    motor += Pos(CENTRES, 0, Z_BR1 - 24) * Cylinder(2.5, 24, align=(C_, C_, MIN))
    add(motor, "NEMA17", (0.15, 0.15, 0.15))
    return Compound(children=items, label="cycloidal_30_d80_belt")

if __name__ == "__main__":
    print(f"belt: {T_MOTOR}T -> {T_SHAFT}T, {BELT_L:.0f} mm loop, centres {CENTRES:.2f} mm")
    print(f"motor body nearest face at r={CENTRES - 21.15:.1f} mm vs housing r={V3.OD/2:.0f} -> "
          f"gap {CENTRES - 21.15 - V3.OD/2:.1f} mm")
    parts = build()
    export_step(assembly(parts), f"{OUT}/ASSEMBLY_belt_drive.step")
    print("assembly written")
