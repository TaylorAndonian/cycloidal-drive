"""
Cycloidal reducer v3 -- small joint, 30:1, NEMA 17, Ø80 x 50.5 mm
Revision of v2 after the first build bound up and broke input shafts.

Changes vs v2
  * real fit system (see FITS below) instead of nominal-everywhere
  * mesh and output-hole clearance opened up -- this is what made v2 stiff
  * eccentric bearings are trapped in a pocket in the disc (relief / seat / lip)
  * one-piece input shaft with the first cam integral; thick sections, no hex
  * second cam slides on a D-drive (one flat = only one correct orientation)
  * front 608 is a floating steady bearing: it cannot fight the motor bearings
  * rear 6704 seat is a clearance fit for the same reason
  * ring pins get 0.6 mm end play

Hardware: unchanged from v2 (6810, 6704, 2x 6803, 608, 16x MR105ZZ,
31x Ø4x30 dowel, 8x Ø5x25/M4 shoulder bolt). Nothing new to buy.

Axis = Z. z=0 is the motor mounting face. Output faces +Z. All dimensions mm.
Run:  python cyc30_v3.py   ->  STEP files in ./step_v3/
"""
import os, math
import numpy as np
from build123d import *

OUT = os.environ.get("CYC_OUT", "step_v3")
os.makedirs(OUT, exist_ok=True)
C, MIN, MAX = Align.CENTER, Align.MIN, Align.MAX

# ===================================================================== FITS
# Everything that caused trouble on the first build lives here.
OD_SHRINK   = 0.125   # printed OD a bearing bore sits on: add this (your measurement)
BORE_PRESS  = 0.10    # printed bore holding a bearing OD: light press after shrink
BORE_SLIP   = 0.30    # printed bore where the bearing must float (steady bearings)
SHAFT_SLIP  = -0.15   # printed OD where the bearing must slide on freely
MESH_CLEAR  = 0.08    # equidistant clearance on the disc profile  <-- main "too stiff" knob
OUT_CLEAR   = 0.12    # extra diameter in the disc output holes
PIN_CLEAR   = 0.09    # ring-pin groove diameter clearance (v3.1: was 0.15, pins rattled;
                      # v2 was 0.04 and bound, so this sits between the two)
PIN_ENDPLAY = 0.60    # axial room for the ring pins
AX_GAP      = 0.90    # axial gap between parts that rotate relative to each other
CLR_M3, INS_M3, INS_M4 = 3.4, 4.0, 5.6
SHOULDER_HOLE = 5.15  # Ø5 shoulder bolt through the rear ring

# ================================================================ geometry
RATIO_PINS = 31            # 31 pins -> 30 lobes -> 30:1
R_PIN   = 31.5             # ring-pin pitch radius
D_PIN   = 4.0              # Ø4 x 30 dowels
ECC     = 0.75
N_OUT   = 8                # output pins
R_OUT   = 20.75
ROLLER_OD, ROLLER_W = 10.0, 4.0        # MR105ZZ 5x10x4
ECC_BRG = (17.0, 26.0, 5.0)            # 6803 -- one per disc, centred
OUT_BRG = (50.0, 65.0, 7.0)            # 6810
REAR_BRG = (20.0, 27.0, 4.0)           # 6704
NOSE_BRG = (8.0, 22.0, 7.0)            # 608
OD = 80.0
T_DISC = 10.0

# =================================================================== stack
Z_BP      = 6.0                      # back plate 0..6
Z_REG     = 7.0                      # top of the back-plate register ring
Z_RR0     = 6.6                      # rear ring
T_RR      = 6.8
Z_RR1     = Z_RR0 + T_RR             # 13.4
CB_DEPTH  = 4.4                      # shoulder-bolt head counterbore
Z_HEAD    = Z_RR0 + CB_DEPTH         # 11.0  head underside
L_SHOULDER = 25.0                    # bolts you already have
Z_FL0     = Z_HEAD + L_SHOULDER      # 36.0  output flange rear face
SPAN      = Z_FL0 - Z_RR1            # 22.6 available for both discs
GAP2      = SPAN - 2 * T_DISC - 2 * AX_GAP   # middle gap
Z_D1      = Z_RR1 + AX_GAP           # 14.3
Z_D2      = Z_D1 + T_DISC + GAP2     # 25.1
Z_PIN0, Z_PIN1 = Z_REG, Z_REG + 30.0            # pins 7.0 .. 37.0
Z_GROOVE_END  = Z_PIN1 + PIN_ENDPLAY             # 37.6
Z_BRG0, Z_BRG1 = 41.0, 48.0          # 6810
Z_FRONT   = 50.0
Z_CAP1    = 50.5

# disc bearing pocket: relief / seat / lip  (bearing centred in the disc)
POCKET_OFF = (T_DISC - ECC_BRG[2]) / 2 - 0.05      # 2.45
Z_EB1 = Z_D1 + POCKET_OFF                          # disc-1 bearing rear face
Z_EB2 = Z_D2 + POCKET_OFF

def cyl(d, h, z0=0.0, x=0.0, y=0.0):
    return Pos(x, y, z0) * Cylinder(d / 2, h, align=(C, C, MIN))

def ring(od, idia, h, z0=0.0):
    return cyl(od, h, z0) - cyl(idia, h + 2, z0 - 1)

def polar(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)

def csk_m3(x, y, ztop, depth=10):
    return (Pos(x, y, ztop) * Cone(3.4 / 2, 6.4 / 2, 1.6, align=(C, C, MAX))) \
           + cyl(CLR_M3, depth, ztop - depth, x, y)

def dshaft(d, h, z0, flat):
    """round section of diameter d with one flat at distance `flat` from the axis"""
    return cyl(d, h, z0) - (Pos(flat, 0, z0) * Box(d, d * 2, h, align=(MIN, C, MIN)))

TIE_ANG = [(k + 0.5) * 360 / RATIO_PINS for k in (0, 5, 10, 15, 20, 26)]
TIE_R = 36.6
RET_ANG = [30 + 60 * k for k in range(6)]
RET_R = 35.8

def safe_chamfer(part, edges, d):
    try:
        return chamfer(edges, d)
    except Exception as ex:
        print("  chamfer skipped:", ex)
        return part

# ============================================================ disc profile
def disc_profile_pts(n_per_lobe=48):
    n = n_per_lobe * (RATIO_PINS - 1)
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    rp = D_PIN / 2 + MESH_CLEAR
    Np = RATIO_PINS
    psi = np.arctan2(np.sin((1 - Np) * t), (R_PIN / (ECC * Np)) - np.cos((1 - Np) * t))
    x = R_PIN * np.cos(t) - rp * np.cos(t + psi) - ECC * np.cos(Np * t)
    y = -R_PIN * np.sin(t) + rp * np.sin(t + psi) + ECC * np.sin(Np * t)
    return [(float(a), float(b)) for a, b in zip(x, y)]

def make_disc():
    pts = disc_profile_pts()
    face = Face(Wire([Edge.make_spline([Vector(p[0], p[1], 0) for p in pts], periodic=True)]))
    d = extrude(face, T_DISC, dir=(0, 0, 1))
    seat = ECC_BRG[1] + BORE_PRESS                       # 26.10
    d -= cyl(seat + 0.6, POCKET_OFF, -0.01)              # rear relief (bearing slides in)
    d -= cyl(seat, ECC_BRG[2] + 0.1, POCKET_OFF)         # seat, bottoms on the lip
    d -= cyl(23.5, T_DISC + 2, -1)                       # lip bore / through bore
    hole = ROLLER_OD + 2 * ECC + OUT_CLEAR               # 11.62
    for k in range(N_OUT):
        x, y = polar(R_OUT, 45 * k)
        d -= cyl(hole, T_DISC + 2, -1, x, y)
    x, y = polar(17.3, 22.5)
    d -= cyl(2.0, 0.8, T_DISC - 0.8, x, y)               # orientation dimple
    return d

# =================================================================== parts
def make_back_plate():
    p = cyl(OD, Z_BP)
    p = safe_chamfer(p, p.edges().filter_by(GeomType.CIRCLE).group_by(Axis.Z)[0].sort_by(SortBy.RADIUS)[-1:], 0.8)
    p += ring(62.7, 58.5, Z_REG - Z_BP, Z_BP)                       # register into housing bore
    p += ring(REAR_BRG[0] + OD_SHRINK + SHAFT_SLIP, 15.6, 5.2, Z_BP)  # boss, 6704 slides on
    p -= cyl(22.4, 2.2, -0.01)                                      # NEMA 17 pilot
    p -= cyl(15.6, 20, -1)                                          # input-shaft clearance
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = 15.5 * sx, 15.5 * sy
            p -= cyl(CLR_M3, 10, -1, x, y)
            p -= cyl(6.4, Z_BP - 2.0 + 0.01, 2.0, x, y)
    for a in TIE_ANG:
        x, y = polar(TIE_R, a)
        p -= cyl(CLR_M3, 10, -1, x, y)
    return p

def make_housing():
    h = cyl(OD, Z_FRONT - Z_BP, Z_BP)
    h = safe_chamfer(h, h.edges().filter_by(GeomType.CIRCLE).sort_by(SortBy.RADIUS)[-2:], 0.8)
    h -= cyl(2 * R_PIN, Z_BRG0 - Z_BP + 0.01, Z_BP - 0.01)
    for k in range(RATIO_PINS):
        x, y = polar(R_PIN, 360 * k / RATIO_PINS)
        h -= cyl(D_PIN + PIN_CLEAR, Z_GROOVE_END - Z_BP + 0.01, Z_BP - 0.01, x, y)
    h -= cyl(OUT_BRG[1] + BORE_PRESS, Z_BRG1 - Z_BRG0 + 0.01, Z_BRG0)
    h -= cyl(78.2, Z_FRONT - Z_BRG1 + 0.01, Z_BRG1)
    for a in TIE_ANG:
        x, y = polar(TIE_R, a)
        h -= cyl(INS_M3, 6.5, Z_BP - 0.01, x, y)
    for a in RET_ANG:
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
    r = cyl(56.0, T_RR, Z_RR0)
    r -= cyl(REAR_BRG[1] + BORE_SLIP, REAR_BRG[2] + 0.2, Z_RR0 - 0.01)   # floating 6704 seat
    r -= cyl(24.0, T_RR + 2, Z_RR0 - 1)
    for k in range(N_OUT):
        x, y = polar(R_OUT, 45 * k)
        r -= cyl(SHOULDER_HOLE, T_RR + 2, Z_RR0 - 1, x, y)
        r -= cyl(9.8, CB_DEPTH + 0.01, Z_RR0 - 0.01, x, y)
    return r

# ---- rotor -------------------------------------------------------------
CAM_J = ECC_BRG[0] + OD_SHRINK          # 17.125 eccentric journal
COLLAR = 18.6
D_DRIVE, D_FLAT = 13.0, 5.1             # cam-B drive: Ø13 with one flat (one orientation only)
                                        # flat sits outside the Ø10 mid section so cam B slides over it
MID_D = 10.0                            # shaft between cam B and the 608
NOSE_D = NOSE_BRG[0] + OD_SHRINK        # 8.125
Z_CAMA0 = Z_EB1 - 0.75                  # cam A collar
Z_DD0   = Z_EB1 + ECC_BRG[2] + 0.1      # D-drive starts after cam A journal
Z_CAMB_J = Z_EB2                        # cam B journal = disc-2 bearing
Z_MID0  = Z_CAMB_J - 0.1           # Ø10 runs from the end of the D-drive to the nose
Z_NOSE0 = Z_FL0
Z_SHAFT_END = Z_FL0 + NOSE_BRG[2] + 0.2

def make_input_shaft():
    s = cyl(14.0, Z_CAMA0 - 3.0, 3.0)                               # thick body
    s += cyl(COLLAR, 0.75, Z_CAMA0, ECC, 0)                         # locates disc-1 bearing
    s += cyl(CAM_J, ECC_BRG[2] + 0.1, Z_EB1, ECC, 0)                # cam A (integral)
    s += dshaft(D_DRIVE, Z_CAMB_J - 0.1 - Z_DD0, Z_DD0, D_FLAT)     # drive for cam B
    s += cyl(MID_D, Z_NOSE0 - Z_MID0, Z_MID0)
    s += cyl(NOSE_D, Z_SHAFT_END - Z_NOSE0, Z_NOSE0)
    s -= cyl(5.1, 7.0, 3.0 - 0.01)                                  # round part of the motor bore
    s -= (cyl(5.1, 24.0 - 10.0, 10.0) - Pos(2.05, 0, 0) * Box(10, 10, 100, align=(MIN, C, C)))
    s -= cyl(INS_M3, 6.0, Z_SHAFT_END - 6.0)                        # end-clamp insert
    return s

def make_cam_b():
    """Slides on from the front. Its eccentric is machined opposite to cam A while the
    drive flat stays on the same side, so the single flat allows exactly one (correct)
    orientation -- it is impossible to fit this cam the wrong way round."""
    h_collar = Z_CAMB_J - Z_DD0 - 0.1
    c = cyl(COLLAR, h_collar, Z_DD0 + 0.1, -ECC, 0)
    c += cyl(CAM_J, ECC_BRG[2] + 0.1, Z_CAMB_J, -ECC, 0)
    c -= dshaft(D_DRIVE + 0.25, h_collar + 0.02, Z_DD0 + 0.09, D_FLAT + 0.125)
    c -= cyl(MID_D + 0.3, ECC_BRG[2] + 0.3, Z_CAMB_J - 0.01)
    return c

def make_shaft_spacer():
    z0 = Z_CAMB_J + ECC_BRG[2] + 0.1
    return cyl(11.8, Z_FL0 - z0, z0) - cyl(MID_D + 0.4, 10, z0 - 1)

def make_output_flange():
    f = cyl(57.5, Z_BRG0 - Z_FL0, Z_FL0)
    f += cyl(OUT_BRG[0] + OD_SHRINK, 47.9 - Z_BRG0, Z_BRG0)
    f -= cyl(NOSE_BRG[1] + BORE_SLIP, NOSE_BRG[2] + 0.4, Z_FL0 - 0.01)   # floating 608
    f -= cyl(17.0, 1.0 + 0.02, Z_FL0 + NOSE_BRG[2] + 0.4 - 0.01)
    f -= cyl(14.0, 10, Z_FL0 + 8.4 - 0.01)
    for k in range(N_OUT):
        x, y = polar(R_OUT, 45 * k)
        f -= cyl(INS_M4, 9.0, Z_FL0 - 0.01, x, y)
    for k in range(8):
        x, y = polar(20.0, 22.5 + 45 * k)
        f -= cyl(INS_M3, 6.0, 47.9 - 6.0 + 0.01, x, y)
    for a in (0, 180):
        x, y = polar(14.0, a)
        f -= cyl(D_PIN + 0.1, 3.0 + 0.01, 47.9 - 3.0, x, y)
    return f

def make_output_cap():
    c = cyl(58.0, Z_CAP1 - Z_BRG1, Z_BRG1)
    c = safe_chamfer(c, c.edges().group_by(Axis.Z)[-1].sort_by(SortBy.RADIUS)[-1:], 0.6)
    c -= ring(58.2, 53.2, 0.4, Z_BRG1 - 0.01)
    c -= cyl(10.0, 10, Z_BRG1 - 1)
    for k in range(4):
        x, y = polar(20.0, 22.5 + 90 * k)
        c -= csk_m3(x, y, Z_CAP1 + 0.01)
        x, y = polar(20.0, 67.5 + 90 * k)
        c -= cyl(CLR_M3, 10, Z_BRG1 - 1, x, y)
    for a in (0, 180):
        x, y = polar(14.0, a)
        c -= cyl(D_PIN + 0.15, 10, Z_BRG1 - 1, x, y)
    return c

def make_sleeve(length):
    return ring(7.4, 5.2, length)

SLEEVE_A = round(Z_D1 + T_DISC / 2 - ROLLER_W / 2 - Z_RR1, 2)        # ring -> roller 1
SLEEVE_B = round((Z_D2 + T_DISC / 2 - ROLLER_W / 2)
                 - (Z_D1 + T_DISC / 2 + ROLLER_W / 2) - 0.3, 2)      # between rollers
SLEEVE_C = round(Z_FL0 - (Z_D2 + T_DISC / 2 + ROLLER_W / 2) - 0.3, 2)

def make_fit_coupon():
    """Print FIRST, in the real material and settings. Every number here is a fit used above."""
    p = Box(104, 44, 5, align=(MIN, MIN, MIN))
    for d, x, y in [
        (ECC_BRG[1] + BORE_PRESS - 0.1, 16, 15), (ECC_BRG[1] + BORE_PRESS, 46, 15),
        (ECC_BRG[1] + BORE_PRESS + 0.1, 76, 15),                      # 6803 seat in the disc
        (SHOULDER_HOLE, 8, 36), (D_PIN + PIN_CLEAR, 18, 36),
        (INS_M3, 28, 36), (INS_M4, 38, 36),
        (REAR_BRG[1] + BORE_SLIP, 56, 36), (NOSE_BRG[1] + BORE_SLIP, 80, 36),
    ]:
        p -= cyl(d, 10, -1, x, y)
    # pegs: a 6803 and a 608 must slide onto these by hand
    p += Pos(-14, 14, 0) * (cyl(CAM_J, 12) )
    p += Pos(-14, -10, 0) * (cyl(NOSE_D, 12))
    p += Pos(-14, 34, 0) * (cyl(REAR_BRG[0] + OD_SHRINK + SHAFT_SLIP, 12))
    # motor D-bore test
    hub = cyl(14, 14) - cyl(5.1, 7, -0.01) \
          - (cyl(5.1, 8.02, 7) - Pos(2.05, 0, 0) * Box(10, 10, 100, align=(MIN, C, C)))
    p += Pos(118, 14, 0) * hub
    # cam-B D-drive test
    p += Pos(118, -12, 0) * dshaft(D_DRIVE, 10, 0, D_FLAT)
    return p

def brg(spec, z0):
    return ring(spec[1], spec[0], spec[2], z0)

def build():
    parts = {
        "back_plate": make_back_plate(), "housing": make_housing(),
        "front_retainer": make_front_retainer(), "rear_ring": make_rear_ring(),
        "disc": make_disc(), "input_shaft": make_input_shaft(), "cam_b": make_cam_b(),
        "shaft_spacer": make_shaft_spacer(), "output_flange": make_output_flange(),
        "output_cap": make_output_cap(), "fit_coupon": make_fit_coupon(),
        f"roller_sleeve_{SLEEVE_A}": make_sleeve(SLEEVE_A),
        f"roller_sleeve_{SLEEVE_B}": make_sleeve(SLEEVE_B),
        f"roller_sleeve_{SLEEVE_C}": make_sleeve(SLEEVE_C),
    }
    for name, p in parts.items():
        export_step(p, f"{OUT}/{name}.step")
        print(f"  {name:22s} vol {p.volume/1000:7.2f} cm3  valid={p.is_valid}")
    return parts

def assembly(parts, theta_deg=0.0):
    th = math.radians(theta_deg)
    out_rot = -theta_deg / (RATIO_PINS - 1)
    items = []
    def add(shape, label, color):
        shape.label = label; shape.color = Color(color); items.append(shape)
    add(parts["back_plate"], "back_plate", "gray")
    add(parts["housing"], "housing", (0.25, 0.3, 0.38))
    add(parts["front_retainer"], "front_retainer", "gray")
    ex, ey = ECC * math.cos(th), ECC * math.sin(th)
    add(Pos(ex, ey, Z_D1) * Rot(0, 0, out_rot) * parts["disc"], "disc_1", "orange")
    add(Pos(-ex, -ey, Z_D2) * Rot(0, 0, out_rot + 90) * parts["disc"], "disc_2", (0.95, 0.45, 0.1))
    add(Rot(0, 0, theta_deg) * parts["input_shaft"], "input_shaft+camA", "steelblue")
    add(Rot(0, 0, theta_deg) * parts["cam_b"], "cam_b", "royalblue")
    add(Rot(0, 0, theta_deg) * parts["shaft_spacer"], "shaft_spacer", "steelblue")
    add(Rot(0, 0, out_rot) * parts["rear_ring"], "rear_ring", "lightgray")
    add(Rot(0, 0, out_rot) * parts["output_flange"], "output_flange", "lightgray")
    add(Rot(0, 0, out_rot) * parts["output_cap"], "output_cap", "silver")
    for k in range(RATIO_PINS):
        x, y = polar(R_PIN, 360 * k / RATIO_PINS)
        add(cyl(D_PIN, 30, Z_PIN0, x, y), f"ring_pin_{k}", "darkgray")
    for k in range(N_OUT):
        x, y = polar(R_OUT, 45 * k + out_rot)
        b = cyl(9.0, 4.0, Z_HEAD - 4.0, x, y) + cyl(5.0, L_SHOULDER, Z_HEAD, x, y) + cyl(4, 8, Z_FL0, x, y)
        add(b, f"shoulder_bolt_{k}", "dimgray")
        for zc in (Z_D1 + T_DISC / 2, Z_D2 + T_DISC / 2):
            add(Pos(x, y, 0) * ring(ROLLER_OD, 5.0, ROLLER_W, zc - ROLLER_W / 2), "MR105", "gold")
        add(Pos(x, y, Z_RR1) * parts[f"roller_sleeve_{SLEEVE_A}"], "sleeve", "white")
        add(Pos(x, y, Z_D1 + T_DISC / 2 + ROLLER_W / 2) * parts[f"roller_sleeve_{SLEEVE_B}"], "sleeve", "white")
        add(Pos(x, y, Z_D2 + T_DISC / 2 + ROLLER_W / 2) * parts[f"roller_sleeve_{SLEEVE_C}"], "sleeve", "white")
    add(Pos(ex, ey, 0) * brg(ECC_BRG, Z_EB1), "6803_disc1", "gold")
    add(Pos(-ex, -ey, 0) * brg(ECC_BRG, Z_EB2), "6803_disc2", "gold")
    add(brg(REAR_BRG, Z_RR0 + 0.1), "6704_rear", "gold")
    add(brg(OUT_BRG, Z_BRG0), "6810_output", "gold")
    add(brg(NOSE_BRG, Z_FL0 + 0.2), "608_steady", "gold")
    motor = Pos(0, 0, -40) * Box(42.3, 42.3, 40, align=(C, C, MIN)) + cyl(22, 2, 0) + cyl(5, 24, 0)
    add(motor, "NEMA17_reference", (0.15, 0.15, 0.15))
    return Compound(children=items, label="cycloidal_30_d80_v3")

if __name__ == "__main__":
    print(f"stack: discs {Z_D1}..{Z_D1+T_DISC} and {Z_D2}..{Z_D2+T_DISC}, mid gap {GAP2:.2f}")
    print(f"sleeves: {SLEEVE_A} / {SLEEVE_B} / {SLEEVE_C}")
    parts = build()
    export_step(assembly(parts), f"{OUT}/ASSEMBLY_cycloidal_30_d80_v3.step")
    print("assembly written")
