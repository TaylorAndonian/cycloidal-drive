"""
Cycloidal reducer v2 -- small joint, 30:1, NEMA 17, ~Ø80 x 50.5 mm
Parametric build123d model. Run:  python cyc30_v2.py  -> STEP files in ./step/

Axis = Z. z=0 is the motor mounting face (back of gearbox). Output faces +Z.
All dimensions mm.
"""
import os, math
import numpy as np
from build123d import *

OUT = os.environ.get("CYC_OUT", "step")
os.makedirs(OUT, exist_ok=True)
C, MIN, MAX = Align.CENTER, Align.MIN, Align.MAX

# ---------------------------------------------------------------- parameters
RATIO_PINS = 31          # ring pins  -> 30 lobes -> 30:1
R_PIN = 31.5             # ring-pin pitch radius
D_PIN = 4.0              # ring pin dowel Ø4 x 30
ECC = 0.75               # eccentricity
PROFILE_DELTA = 0.02     # equidistant clearance added to disc profile (backlash vs. print tolerance)
N_OUT = 8                # output pins (Ø5 shoulder bolts)
R_OUT = 20.75            # output pin PCD radius (PCD 41.5)
ROLLER_OD = 10.0         # MR105ZZ 5x10x4
OUT_HOLE = ROLLER_OD + 2 * ECC   # 11.5 -- zero-backlash output hole
OD = 80.0

# print-fit allowances (tune with fit_coupon.step)
PRESS = 0.00             # bearing OD seats: modelled at nominal (Bambu holes print ~0.05-0.1 under)
SLIP = 0.10              # sliding / clearance fits
INS_M3, INS_M4 = 4.0, 5.6   # heat-set insert holes (CNC-Kitchen / Ruthex style)
CLR_M3 = 3.4

# ---------------------------------------------------------------- z stack
Z_BP = 6.0                       # back plate 0..6
Z_RR0, Z_RR1 = 6.6, 13.2         # rear ring
CB_DEPTH = 4.3                   # shoulder-bolt head counterbore (head Ø9 x 4.0)
Z_HEAD = Z_RR0 + CB_DEPTH        # 10.9 head underside
L_SHOULDER = 25.0
Z_FL0 = Z_HEAD + L_SHOULDER      # 35.9 output flange rear face
GAP = (Z_FL0 - Z_RR1 - 20.0) / 3 # 0.9
Z_D1 = Z_RR1 + GAP               # disc 1: 14.1..24.1
Z_D2 = Z_D1 + 10 + GAP           # disc 2: 25.0..35.0
T_DISC = 10.0
Z_PIN0, Z_PIN1 = 7.0, 37.0       # ring pins Ø4 x 30
Z_BRG0, Z_BRG1 = 41.0, 48.0      # 6810 output bearing
Z_FRONT = 50.0                   # housing front face
Z_CAP1 = 50.5                    # output face

def cyl(d, h, z0=0.0, x=0.0, y=0.0):
    return Pos(x, y, z0) * Cylinder(d / 2, h, align=(C, C, MIN))

def ring(od, idia, h, z0=0.0):
    return cyl(od, h, z0) - cyl(idia, h + 2, z0 - 1)

def polar(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)

def hexprism(af, h, z0=0.0):
    sk = RegularPolygon(af / math.sqrt(3), 6, major_radius=True)
    return Pos(0, 0, z0) * extrude(sk, h)

def csk_m3(x, y, ztop, depth=10):
    """countersunk M3 clearance going down from ztop"""
    cone = Pos(x, y, ztop) * Cone(3.3 / 2, 6.4 / 2, 1.6, align=(C, C, MAX))
    return cone + cyl(CLR_M3, depth, ztop - depth, x, y)

TIE_ANG = [(k + 0.5) * 360 / RATIO_PINS for k in (0, 5, 10, 15, 20, 26)]  # between pins
TIE_R = 36.6
RET_ANG = [30 + 60 * k for k in range(6)]
RET_R = 35.8

def safe_chamfer(part, edges, d):
    try:
        return chamfer(edges, d)
    except Exception as ex:
        print("  chamfer skipped:", ex)
        return part

# ---------------------------------------------------------------- disc profile
def disc_profile_pts(n_per_lobe=48):
    n = n_per_lobe * (RATIO_PINS - 1)
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    rp = D_PIN / 2 + PROFILE_DELTA
    Np = RATIO_PINS
    psi = np.arctan2(np.sin((1 - Np) * t), (R_PIN / (ECC * Np)) - np.cos((1 - Np) * t))
    x = R_PIN * np.cos(t) - rp * np.cos(t + psi) - ECC * np.cos(Np * t)
    y = -R_PIN * np.sin(t) + rp * np.sin(t + psi) + ECC * np.sin(Np * t)
    return [(float(a), float(b)) for a, b in zip(x, y)]

def make_disc():
    pts = disc_profile_pts()
    edge = Edge.make_spline([Vector(p[0], p[1], 0) for p in pts], periodic=True)
    face = Face(Wire([edge]))
    d = extrude(face, T_DISC, dir=(0, 0, 1))
    d -= cyl(26.0 + PRESS, T_DISC + 2, -1)                      # 2 x 6803
    for k in range(N_OUT):
        x, y = polar(R_OUT, 45 * k)
        d -= cyl(OUT_HOLE, T_DISC + 2, -1, x, y)
    x, y = polar(17.5, 22.5)                                    # orientation dimple (front face)
    d -= cyl(2.0, 0.8, T_DISC - 0.8, x, y)
    return d

# ---------------------------------------------------------------- parts
def make_back_plate():
    p = cyl(OD, Z_BP)
    p = safe_chamfer(p, p.edges().filter_by(GeomType.CIRCLE).group_by(Axis.Z)[0].sort_by(SortBy.RADIUS)[-1:], 0.8)
    p += ring(62.9, 58.5, 1.0, Z_BP)                            # register spigot into housing bore
    p += ring(20.0 - 0.02, 13.0, Z_RR0 + 4.0 - Z_BP, Z_BP)      # boss for 6704 (rear ring bearing)
    p -= cyl(22.3, 2.2, -0.01)                                  # NEMA17 pilot
    p -= cyl(13.0, 20, -1)                                      # shaft hub clearance
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = 15.5 * sx, 15.5 * sy
            p -= cyl(CLR_M3, 10, -1, x, y)
            p -= cyl(6.2, Z_BP - 2.0 + 0.01, 2.0, x, y)         # counterbore, head below face
    for a in TIE_ANG:
        x, y = polar(TIE_R, a)
        p -= cyl(CLR_M3, 10, -1, x, y)
    return p

def make_housing():
    h = cyl(OD, Z_FRONT - Z_BP, Z_BP)
    h = safe_chamfer(h, h.edges().filter_by(GeomType.CIRCLE).sort_by(SortBy.RADIUS)[-2:], 0.8)
    h -= cyl(2 * R_PIN, Z_BRG0 - Z_BP + 0.01, Z_BP - 0.01)     # pin-region bore Ø63
    for k in range(RATIO_PINS):                                  # blind pin grooves
        x, y = polar(R_PIN, 360 * k / RATIO_PINS)
        h -= cyl(D_PIN + 0.04, Z_PIN1 - Z_BP + 0.01, Z_BP - 0.01, x, y)
    h -= cyl(65.0 + PRESS, Z_BRG1 - Z_BRG0 + 0.01, Z_BRG0)     # 6810 seat
    h -= cyl(78.2, Z_FRONT - Z_BRG1 + 0.01, Z_BRG1)            # retainer ring recess
    for a in TIE_ANG:                                            # rear tie inserts
        x, y = polar(TIE_R, a)
        h -= cyl(INS_M3, 6.5, Z_BP - 0.01, x, y)
    for a in RET_ANG:                                            # front retainer inserts
        x, y = polar(RET_R, a)
        h -= cyl(INS_M3, 6.0, Z_BRG1 - 6.0, x, y)
    return h

def make_front_retainer():
    r = ring(78.0, 62.0, Z_FRONT - Z_BRG1, Z_BRG1)
    for a in RET_ANG:
        x, y = polar(RET_R, a)
        r -= csk_m3(x, y, Z_FRONT + 0.01)
    return r

def make_rear_ring():
    t = Z_RR1 - Z_RR0
    r = cyl(56.0, t, Z_RR0)
    r -= cyl(27.0 + PRESS, 4.0 + 0.01, Z_RR0 - 0.01)            # 6704 pocket
    r -= cyl(24.0, t + 2, Z_RR0 - 1)                            # lip bore
    for k in range(N_OUT):
        x, y = polar(R_OUT, 45 * k)
        r -= cyl(5.02, t + 2, Z_RR0 - 1, x, y)                  # shoulder fit
        r -= cyl(9.6, CB_DEPTH + 0.01, Z_RR0 - 0.01, x, y)      # head counterbore
    return r

CAM_COLLAR = GAP
def make_cam():
    """z0 = collar rear face. Eccentric toward +X. Cam B = same part rotated 180 deg."""
    c = cyl(18.6, CAM_COLLAR, 0, ECC, 0) + cyl(17.0, T_DISC, CAM_COLLAR, ECC, 0)
    c -= hexprism(10.0 + SLIP, 30, -5)
    return c

def make_input_shaft():
    s = cyl(12.0, 10.8 - 2.5, 2.5)
    s += cyl(18.0, Z_RR1 - 10.8, 10.8)
    s += hexprism(10.0, Z_D2 + T_DISC - Z_RR1, Z_RR1)
    s += cyl(8.0 - 0.01, 42.8 - (Z_D2 + T_DISC), Z_D2 + T_DISC)
    s -= cyl(5.05, 10.0 - 2.5 + 0.01, 2.5 - 0.01)               # round bore near motor
    dbore = cyl(5.05, 27.0 - 10.0, 10.0) - Pos(2.05, 0, 0) * Box(10, 10, 100, align=(MIN, C, C))
    s -= dbore                                                   # D bore (flat 4.5 across)
    s -= cyl(INS_M3, 6.0, 42.8 - 6.0)                            # end-clamp insert
    return s

def make_shaft_spacer():
    z0 = Z_D2 + T_DISC
    s = cyl(19.0, 0.5, z0) + cyl(12.0, Z_FL0 - z0 - 0.5, z0 + 0.5)
    return s - cyl(8.15, 5, z0 - 1)

def make_output_flange():
    f = cyl(58.0, Z_BRG0 - Z_FL0, Z_FL0)
    f += cyl(50.0 - 0.01, 47.9 - Z_BRG0, Z_BRG0)                # 6810 inner-race journal
    f -= cyl(22.0 + PRESS, 7.0 + 0.01, Z_FL0 - 0.01)            # 608 pocket
    f -= cyl(17.0, 1.0 + 0.02, Z_FL0 + 7.0 - 0.01)              # lip
    f -= cyl(14.0, 10, Z_FL0 + 8.0 - 0.01)                      # screw access
    for k in range(N_OUT):                                       # M4 inserts for shoulder bolts
        x, y = polar(R_OUT, 45 * k)
        f -= cyl(INS_M4, 9.0, Z_FL0 - 0.01, x, y)
    for k in range(8):                                           # M3 inserts: cap + arm
        x, y = polar(20.0, 22.5 + 45 * k)
        f -= cyl(INS_M3, 6.0, 47.9 - 6.0 + 0.01, x, y)
    for a in (0, 180):                                           # Ø4 dowels (spare ring pins)
        x, y = polar(14.0, a)
        f -= cyl(4.0, 3.0 + 0.01, 47.9 - 3.0, x, y)
    return f

def make_output_cap():
    t = Z_CAP1 - Z_BRG1
    c = cyl(58.0, t, Z_BRG1)
    c = safe_chamfer(c, c.edges().group_by(Axis.Z)[-1].sort_by(SortBy.RADIUS)[-1:], 0.6)
    c -= ring(58.2, 53.2, 0.3, Z_BRG1 - 0.01)                   # relief: touch inner race only
    c -= cyl(10.0, 10, Z_BRG1 - 1)
    for k in range(4):
        x, y = polar(20.0, 22.5 + 90 * k)
        c -= csk_m3(x, y, Z_CAP1 + 0.01)                        # cap screws
        x, y = polar(20.0, 67.5 + 90 * k)
        c -= cyl(CLR_M3, 10, Z_BRG1 - 1, x, y)                  # arm screws pass through
    for a in (0, 180):
        x, y = polar(14.0, a)
        c -= cyl(4.05, 10, Z_BRG1 - 1, x, y)
    return c

def make_sleeve(length):
    return ring(7.4, 5.1, length)

def make_fit_coupon():
    """Print in the same material/settings as the real parts, test every fit before the full print."""
    p = Box(96, 40, 5, align=(MIN, MIN, MIN))
    holes = [  # (dia, x, y, label)
        (25.9, 16, 14), (26.0, 46, 14), (26.1, 76, 14),        # 6803 in disc
        (5.0, 8, 34), (5.02, 16, 34), (5.1, 24, 34),           # shoulder bolt in rear ring
        (4.0, 36, 34), (4.04, 44, 34), (4.1, 52, 34),          # ring pin groove / dowel
        (INS_M3, 64, 34), (INS_M4, 74, 34), (8.15, 86, 34),    # insert holes, spacer
    ]
    for d, x, y in holes:
        p -= cyl(d, 10, -1, x, y)
    # hex + D-bore test hub: slide a cam-style hex socket over it and the motor shaft into it
    hub = hexprism(10.0, 12, 5) + cyl(12, 5, 0)
    hub -= cyl(5.05, 5, 0 - 0.01)
    hub -= cyl(5.05, 12.02, 5) - Pos(2.05, 0, 0) * Box(10, 10, 100, align=(MIN, C, C))
    p += Pos(-12, 20, 0) * hub
    sock = cyl(20, 8) - hexprism(10.0 + SLIP, 10, -1)
    p += Pos(-12, -6, 0) * sock
    # 6810 / 6704 seat rings
    p += Pos(48, -40, 0) * (cyl(71, 5) - cyl(65.0, 7, -1))
    p += Pos(-12, -32, 0) * (cyl(33, 4) - cyl(27.0, 6, -1))
    return p

# purchased-part stand-ins for the assembly
def brg(d, D, B, z0):
    return ring(D, d, B, z0)

def build():
    parts = {}
    print("disc...")
    disc = make_disc()
    print("others...")
    parts["back_plate"] = make_back_plate()
    parts["housing"] = make_housing()
    parts["front_retainer"] = make_front_retainer()
    parts["rear_ring"] = make_rear_ring()
    parts["disc"] = disc
    parts["cam"] = make_cam()
    parts["input_shaft"] = make_input_shaft()
    parts["shaft_spacer"] = make_shaft_spacer()
    parts["output_flange"] = make_output_flange()
    parts["output_cap"] = make_output_cap()
    parts["roller_sleeve_3p9"] = make_sleeve(3.9)
    parts["roller_sleeve_6p9"] = make_sleeve(6.9)
    parts["fit_coupon"] = make_fit_coupon()
    for name, p in parts.items():
        export_step(p, f"{OUT}/{name}.step")
        print(f"  {name:20s} vol {p.volume/1000:7.2f} cm3  valid={p.is_valid}")
    return parts

def assembly(parts, theta_deg=0.0):
    th = math.radians(theta_deg)
    out_rot = -theta_deg / (RATIO_PINS - 1)
    items = []
    def add(shape, label, color):
        shape.label = label
        shape.color = Color(color)
        items.append(shape)
    add(parts["back_plate"], "back_plate", "gray")
    add(parts["housing"], "housing", (0.25, 0.3, 0.38))
    add(parts["front_retainer"], "front_retainer", "gray")
    ex, ey = ECC * math.cos(th), ECC * math.sin(th)
    d1 = Pos(ex, ey, Z_D1) * Rot(0, 0, out_rot) * parts["disc"]
    d2 = Pos(-ex, -ey, Z_D2) * Rot(0, 0, out_rot + 90) * parts["disc"]
    add(d1, "disc_1", "orange"); add(d2, "disc_2 (rotated 90deg)", (0.95, 0.45, 0.1))
    add(Rot(0, 0, theta_deg) * Pos(0, 0, Z_RR1) * parts["cam"], "cam_A", "royalblue")
    add(Rot(0, 0, theta_deg + 180) * Pos(0, 0, Z_RR1 + CAM_COLLAR + T_DISC) * parts["cam"], "cam_B", "royalblue")
    add(Rot(0, 0, theta_deg) * parts["input_shaft"], "input_shaft", "steelblue")
    add(parts["shaft_spacer"], "shaft_spacer", "steelblue")
    add(Rot(0, 0, out_rot) * parts["rear_ring"], "rear_ring", "lightgray")
    add(Rot(0, 0, out_rot) * parts["output_flange"], "output_flange", "lightgray")
    add(Rot(0, 0, out_rot) * parts["output_cap"], "output_cap", "silver")
    # purchased
    for k in range(RATIO_PINS):
        x, y = polar(R_PIN, 360 * k / RATIO_PINS)
        add(cyl(D_PIN, 30, Z_PIN0, x, y), f"ring_pin_{k}", "darkgray")
    for k in range(N_OUT):
        x, y = polar(R_OUT, 45 * k + out_rot)
        b = cyl(9, 4.0, Z_HEAD - 4.0, x, y) + cyl(5, L_SHOULDER, Z_HEAD, x, y) + cyl(4, 8, Z_FL0, x, y)
        add(b, f"shoulder_bolt_{k}", "dimgray")
        for zc in (Z_D1 + 5, Z_D2 + 5):
            add(Pos(x, y, 0) * brg(5, 10, 4, zc - 2), f"MR105_{k}", "gold")
        add(Pos(x, y, Z_RR1) * parts["roller_sleeve_3p9"], "sleeve", "white")
        add(Pos(x, y, Z_D1 + 7) * parts["roller_sleeve_6p9"], "sleeve", "white")
        add(Pos(x, y, Z_D2 + 7) * parts["roller_sleeve_3p9"], "sleeve", "white")
    for z in (Z_D1, Z_D1 + 5):
        add(Pos(ex, ey, 0) * brg(17, 26, 5, z), "6803_disc1", "gold")
    for z in (Z_D2, Z_D2 + 5):
        add(Pos(-ex, -ey, 0) * brg(17, 26, 5, z), "6803_disc2", "gold")
    add(brg(20, 27, 4, Z_RR0), "6704_rear", "gold")
    add(brg(50, 65, 7, Z_BRG0), "6810_output", "gold")
    add(brg(8, 22, 7, Z_FL0), "608_input_front", "gold")
    motor = Pos(0, 0, -40) * Box(42.3, 42.3, 40, align=(C, C, MIN)) + cyl(22, 2, 0) + cyl(5, 24, 0)
    add(motor, "NEMA17_reference", (0.15, 0.15, 0.15))
    return Compound(children=items, label="cycloidal_30_d80_v2")

if __name__ == "__main__":
    parts = build()
    asm = assembly(parts)
    export_step(asm, f"{OUT}/ASSEMBLY_cycloidal_30_d80_v2.step")
    print("assembly written")
