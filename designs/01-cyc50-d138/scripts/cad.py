#!/usr/bin/env python3
"""
cad.py  --  Parametric build123d model of the 50:1 single-stage cycloidal gearbox.

    python cad.py            -> builds every part, exports STEP + STL to ../cad/
    python cad.py --quick    -> skips the (slow) assembly STEP

Coordinate system: Z is the motor axis, Z=0 is the motor mounting face
(back of the rear cup), +Z points toward the output flange.  All units mm.

EVERY dimension lives in the PARAMETERS block below.  Everything after it is
derived.  The cycloidal profile is generated from the epitrochoid equations in
cycloid_profile.py (shared with analysis.py) -- no hard-coded points.
"""
import sys, os, math, json

# =============================================================================
#                              PARAMETERS
# =============================================================================
P = dict(
    # ---- Ratio / mesh -------------------------------------------------------
    N_LOBES        = 50,     # lobes on each cycloidal disc
    N_PINS         = 51,     # ring pins (fixed).  Ratio = N_LOBES / (N_PINS-N_LOBES) = 50
    R_PIN          = 56.0,   # ring pin circle RADIUS (pin circle dia = 112)
    PIN_D          = 4.0,    # ring pin diameter (hardened dowel, 4 x 60 mm)
    PIN_L          = 60.0,   # ring pin length (4 x 60 hardened dowel)
    ECC            = 0.82,   # eccentricity e.  K = e*N_PINS/R_PIN = 0.75 (must be < 1; see analysis K sweep)
    PROFILE_CLR    = 0.10,   # extra equidistant offset of the lobe profile (clearance)
    DISC_T         = 12.0,   # disc thickness (contact stress ~ 1/sqrt(T); 12 mm keeps Hertz < 90 MPa at 36 Nm)
    DISC_BRG       = dict(id=15.0, od=28.0, w=7.0, name="6902-2RS"),   # eccentric bearing
    DISC_BRG_CLR   = -0.05,  # bore vs bearing OD: negative = interference (press)
    DISC_LEDGE     = 1.0,    # radial ledge that locates the eccentric bearing in the disc

    # ---- Output pins / rollers ---------------------------------------------
    N_OUT          = 6,      # output pins
    R_OUT          = 33.0,   # output pin circle RADIUS (PCD 66)
    OUT_PIN_D      = 8.0,    # output pin (hardened dowel 8 x 45 mm)
    OUT_PIN_L      = 45.0,   # 8 x 45 hardened dowel
    ROLLER         = dict(id=8.0, od=22.0, w=7.0, name="608ZZ"),
    HOLE_CLR       = 0.10,   # disc roller-hole dia = ROLLER.od + 2*ECC + HOLE_CLR
    DISC_AX_GAP    = 0.40,   # axial gap disc <-> carrier thrust face (each face)
    DISC_DISC_GAP  = 1.00,   # axial gap between the two discs

    # ---- Motor (NEMA 17) -----------------------------------------------------
    MOTOR_FACE     = 42.3,
    MOTOR_HOLE_SP  = 31.0,   # NEMA17 hole SPACING (square).  Holes at radius 31/sqrt(2)=21.92 on 45 deg
    MOTOR_BOSS_D   = 22.0,
    MOTOR_BOSS_H   = 2.0,
    MOTOR_SHAFT_D  = 5.0,
    MOTOR_FLAT     = 0.5,    # depth of the D flat (5.0 -> 4.5 across flat)
    MOTOR_SHAFT_L  = 22.0,   # usable shaft length.  Longer/shorter: see README "Motor shaft"
    MOTOR_SCREW_D  = 3.0,

    # ---- Bearings ------------------------------------------------------------
    BRG_REAR_IN    = dict(id=12.0, od=21.0, w=5.0,  name="6801-2RS"),   # input shaft rear
    BRG_FRONT_IN   = dict(id=8.0,  od=22.0, w=7.0,  name="608ZZ"),      # input shaft front (in flange)
    BRG_CARRIER    = dict(id=45.0, od=58.0, w=7.0,  name="6809-2RS"),   # carrier rear  (you have it)
    BRG_OUTPUT     = dict(id=80.0, od=100.0, w=10.0, name="6816-2RS"),  # output flange (buy)

    # ---- Input (eccentric) shaft --------------------------------------------
    SHAFT_REAR_D   = 12.0,   # journal in 6801
    SHAFT_CAM_D    = 15.0,   # cam journal = 6902 bore
    SHAFT_COLLAR_D = 17.0,   # collar between cams (locates both inner races)
    SHAFT_FRONT_D  = 8.0,    # journal in 608 in the output flange
    SHAFT_REAR_CLR = 0.5,    # gap between shaft rear end and bearing-pocket floor

    # ---- Rear cup (motor plate + shell, one print) ---------------------------
    CUP_OD         = 138.0,
    FLOOR_T        = 15.0,   # motor plate thickness
    SHELL_T        = 9.0,    # shell wall radial thickness
    PIN_SEAT_REAR  = 8.0,    # ring pin engagement in the cup floor
    PILOT_CLR      = 0.30,   # pilot recess dia = MOTOR_BOSS_D + PILOT_CLR
    PILOT_DEPTH    = 2.3,    # pilot recess depth (boss is 2.0)
    SHAFT_HOLE_D   = 11.0,   # motor shaft clearance hole through the floor (cone above the pilot recess ends here)
    MOTOR_CBORE_D  = 6.5,    # counterbore for M3 SHCS heads (head 5.5 x 3.0), reached from inside
    MOTOR_FLOOR_UNDER_HEAD = 2.5,  # floor left under the screw head (head sits below the 6809 pocket floor)
    N_HOUSING_BOLTS= 8,
    HOUSING_BOLT_PCD = 129.0,

    # ---- Carrier rear plate --------------------------------------------------
    CARRIER_OD     = 92.0,
    CARRIER_T      = 8.0,
    CARRIER_BOSS_LEN = 7.5,  # into the 6809
    CARRIER_BORE_D = 20.0,   # clearance bore around the input shaft
    CARRIER_GAP    = 0.5,    # gap carrier body <-> cup floor

    # ---- Output flange -------------------------------------------------------
    FLANGE_BODY_OD = 97.0,   # body that holds pins (inside the front plate bore, 1.5 mm gap)
    FLANGE_BODY_T  = 8.0,
    FLANGE_PROUD   = 1.5,    # flange face stands proud of the front plate lip
    FLANGE_PIN_DEPTH = 12.0, # output pin blind hole depth in flange
    FLANGE_IFACE_N = 6,      # user interface: 6x M3 inserts
    FLANGE_IFACE_PCD = 48.0,
    FLANGE_PILOT_D = 10.0,   # center pilot hole
    FLANGE_PILOT_DEPTH = 3.0,

    # ---- Front plate ---------------------------------------------------------
    FRONT_GAP      = 0.5,    # front plate rear face sits this far in front of the flange rear face
    PIN_SEAT_FRONT = 18.0,   # ring pin blind hole depth in the front plate (pin engages ~17 mm)
    LIP_T          = 2.0,    # bearing retaining lip thickness
    LIP_ID         = 94.0,
    LIP_GAP        = 0.30,   # axial play between 6816 outer race and lip (shim if needed)
    HOUSING_CBORE_D = 6.0,
    HOUSING_CBORE_DEPTH = 4.5,

    # ---- Fits / tolerances (see README "Tolerance strategy") ----------------
    FIT_PIN_PRESS_REAR  = -0.05,   # 4 mm ring pin in cup floor: interference
    FIT_PIN_PRESS_FRONT = -0.02,   # 4 mm ring pin in front plate: light press
    FIT_OUTPIN_PRESS    = -0.05,   # 8 mm output pin in carrier/flange
    FIT_BRG_OD_IN_PLASTIC = -0.05, # bearing outer race in printed pocket (press)
    FIT_BRG_ID_ON_PLASTIC = +0.03, # printed journal vs bearing bore (press: journal oversize)
    FIT_SCREW_CLR       = 0.40,    # M3 clearance hole = 3.4
    INSERT_HOLE_D       = 4.0,     # M3 x 5.7 x 4.0 heat-set insert
    INSERT_HOLE_DEPTH   = 7.0,
    INSERT_LEN          = 5.7,
    INSERT_CSK_D        = 4.6,     # entry countersink

    # ---- Printer -------------------------------------------------------------
    NOZZLE         = 0.4,
    MIN_WALL       = 0.8,    # two perimeters: no ligament may be thinner than this
)

# =============================================================================
#                          DERIVED LAYOUT (no magic numbers)
# =============================================================================
def layout(P=P):
    """Axial (Z) positions of every feature.  Returned as a dict; also dumped
    to layout.json so the README fastener table and analysis use the same numbers."""
    L = {}
    L["z_floor_top"]   = P["FLOOR_T"]
    b = P["BRG_REAR_IN"]
    L["z_brg_rear_bot"] = P["FLOOR_T"] - b["w"]                     # 6801 pocket floor
    L["z_brg_rear_top"] = P["FLOOR_T"]
    c = P["BRG_CARRIER"]
    L["pocket_6809_depth"] = c["w"] + P["LIP_GAP"]                  # 0.3 play, 6816 locates
    L["z_pocket_6809_bot"] = P["FLOOR_T"] - L["pocket_6809_depth"]
    L["z_carrier_rear"] = P["FLOOR_T"] + P["CARRIER_GAP"]           # carrier body rear face
    L["z_carrier_front"] = L["z_carrier_rear"] + P["CARRIER_T"]     # thrust face for disc A
    L["z_boss_rear"] = L["z_carrier_rear"] - P["CARRIER_BOSS_LEN"]  # carrier boss rear end
    z0 = L["z_carrier_front"]
    g, t, gd = P["DISC_AX_GAP"], P["DISC_T"], P["DISC_DISC_GAP"]
    L["z_discA"] = (z0 + g, z0 + g + t)
    L["z_discB"] = (z0 + g + t + gd, z0 + g + 2*t + gd)
    L["z_flange_rear"] = z0 + 2*g + 2*t + gd                         # thrust face for disc B
    rw = P["ROLLER"]["w"]
    L["z_rollerA"] = ((L["z_discA"][0] + L["z_discA"][1]) / 2 - rw/2, (L["z_discA"][0] + L["z_discA"][1]) / 2 + rw/2)
    L["z_rollerB"] = ((L["z_discB"][0] + L["z_discB"][1]) / 2 - rw/2, (L["z_discB"][0] + L["z_discB"][1]) / 2 + rw/2)
    L["washer_1"] = L["z_rollerA"][0] - z0
    L["washer_mid"] = L["z_rollerB"][0] - L["z_rollerA"][1]
    L["washer_3"] = L["z_flange_rear"] - L["z_rollerB"][1]
    # eccentric shaft
    L["z_shaft_rear"] = L["z_brg_rear_bot"] + P["SHAFT_REAR_CLR"]
    L["z_camA"] = L["z_rollerA"]            # 6902 is 7 wide like the 608, centred on the disc
    L["z_camB"] = L["z_rollerB"]
    L["z_collar"] = (L["z_camA"][1], L["z_camB"][0])
    fb = P["BRG_FRONT_IN"]
    L["z_flange_608_bot"] = L["z_flange_rear"] + fb["w"]
    L["z_shaft_front"] = L["z_flange_608_bot"] - 0.5
    L["d_bore_depth"] = P["MOTOR_SHAFT_L"] + 0.5 - L["z_shaft_rear"]
    # output flange
    L["z_flange_body_front"] = L["z_flange_rear"] + P["FLANGE_BODY_T"]
    ob = P["BRG_OUTPUT"]
    L["z_6816"] = (L["z_flange_body_front"], L["z_flange_body_front"] + ob["w"])
    L["z_flange_face"] = L["z_6816"][1] + P["LIP_GAP"] + P["LIP_T"] + P["FLANGE_PROUD"]
    L["flange_boss_len"] = L["z_flange_face"] - L["z_flange_body_front"]
    # front plate
    L["z_front_rear"] = L["z_flange_rear"] + P["FRONT_GAP"]
    L["z_lip"] = (L["z_6816"][1] + P["LIP_GAP"], L["z_6816"][1] + P["LIP_GAP"] + P["LIP_T"])
    L["z_front_face"] = L["z_lip"][1]
    L["front_plate_T"] = L["z_front_face"] - L["z_front_rear"]
    L["z_shell_top"] = L["z_front_rear"]
    # pins
    L["z_ringpin"] = (P["FLOOR_T"] - P["PIN_SEAT_REAR"], P["FLOOR_T"] - P["PIN_SEAT_REAR"] + P["PIN_L"])
    L["ringpin_front_engage"] = L["z_ringpin"][1] - L["z_front_rear"]
    L["ringpin_free_span"] = L["z_front_rear"] - P["FLOOR_T"]
    L["z_outpin"] = (L["z_carrier_rear"], L["z_carrier_rear"] + P["OUT_PIN_L"])
    L["outpin_flange_engage"] = L["z_outpin"][1] - L["z_flange_rear"]
    # radii
    L["r_shell_in"] = P["CUP_OD"]/2 - P["SHELL_T"]
    L["disc_hole_d"] = P["ROLLER"]["od"] + 2*P["ECC"] + P["HOLE_CLR"]
    L["overall_length"] = L["z_flange_face"]
    return L


def check_layout(P, L):
    """Assertions that catch parameter combinations that do not fit."""
    assert P["ECC"]*P["N_PINS"] < P["R_PIN"], "K>=1: undercut / cusps"
    assert L["ringpin_front_engage"] > 6, "ring pin too short for front engagement"
    assert L["ringpin_front_engage"] < P["PIN_SEAT_FRONT"], "ring pin bottoms in front plate hole"
    assert L["outpin_flange_engage"] > 6 and L["outpin_flange_engage"] < P["FLANGE_PIN_DEPTH"], "output pin length"
    assert L["d_bore_depth"] > 8, "motor shaft engagement < 8 mm: use a longer shaft or coupler"
    assert P["FLANGE_BODY_OD"]/2 < P["BRG_OUTPUT"]["od"]/2 - 1.0, "flange body hits front plate bore"
    assert P["R_OUT"] + P["OUT_PIN_D"]/2 < P["BRG_OUTPUT"]["id"]/2 - 2.5, "output pins too close to 6816 seat"
    assert P["R_OUT"] - L["disc_hole_d"]/2 > P["DISC_BRG"]["od"]/2 + 4, "roller hole hits disc bore"
    root_r = P["R_PIN"] - P["ECC"] - P["PIN_D"]/2 - P["PROFILE_CLR"]
    assert root_r - (P["R_OUT"] + L["disc_hole_d"]/2) > 4, "roller hole too close to lobe root"
    assert P["R_PIN"] + P["ECC"] + P["PIN_D"]/2 < L["r_shell_in"] - 1, "disc tips hit shell"
    assert P["BRG_OUTPUT"]["od"]/2 + 3 < P["R_PIN"] - P["PIN_D"]/2, "6816 seat too close to ring pins"
    r_ms = P["MOTOR_HOLE_SP"]/math.sqrt(2)
    assert r_ms + P["MOTOR_CBORE_D"]/2 < P["BRG_CARRIER"]["od"]/2 - 1.5, "motor screw counterbore cuts the 6809 pocket wall"
    assert r_ms - P["MOTOR_CBORE_D"]/2 > P["BRG_REAR_IN"]["od"]/2 + 1.0, "motor screw counterbore hits the 6801 pocket"
    assert P["MOTOR_FLOOR_UNDER_HEAD"] >= P["PILOT_DEPTH"], "screw head pocket breaks into the pilot recess"


# =============================================================================
#                                CAD BUILD   (build123d algebra mode)
# =============================================================================
def build_all(out_dir, quick=False):
    from build123d import (Cylinder, Cone, Box, Align, Pos, Axis, Spline, Wire, Face, extrude,
                           export_step, export_stl)
    from cycloid_profile import cycloid_profile
    L = layout(P)
    check_layout(P, L)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    parts = {}
    A0 = (Align.CENTER, Align.CENTER, Align.MIN)

    def cyl(d, h, z0, x=0.0, y=0.0):          # cylinder dia d from z0 to z0+h at (x, y)
        return Cylinder(d/2, h, align=A0).moved(Pos(x, y, z0))

    def polar(n, r, phase=0.0):
        return [(r*math.cos(phase + 2*math.pi*i/n), r*math.sin(phase + 2*math.pi*i/n)) for i in range(n)]

    def csk(d_top, d_hole, z_face):            # entry countersink for a heat-set insert, from face z (facing +Z)
        h = (d_top - d_hole)/2
        return Cone(d_top/2, d_hole/2, h, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(Pos(0, 0, z_face + 0.001))

    # ------------------------------------------------------------ rear cup ---
    cup = cyl(P["CUP_OD"], L["z_shell_top"], 0)
    cup -= cyl(2*L["r_shell_in"], L["z_shell_top"] - P["FLOOR_T"] + 1, P["FLOOR_T"])
    pd = P["MOTOR_BOSS_D"] + P["PILOT_CLR"]
    cup -= cyl(pd, P["PILOT_DEPTH"] + 0.001, -0.001)                                   # pilot recess
    cone_h = (pd - P["SHAFT_HOLE_D"])/2                                                # 45 deg, self-supporting
    cup -= Cone(pd/2, P["SHAFT_HOLE_D"]/2, cone_h, align=A0).moved(Pos(0, 0, P["PILOT_DEPTH"] - 0.001))
    cup -= cyl(P["SHAFT_HOLE_D"], P["FLOOR_T"] + 2, -1)                                 # motor shaft hole
    b = P["BRG_REAR_IN"]
    cup -= cyl(b["od"] - P["FIT_BRG_OD_IN_PLASTIC"], b["w"] + 1, L["z_brg_rear_bot"])   # 6801 pocket
    c = P["BRG_CARRIER"]
    cup -= cyl(c["od"] - P["FIT_BRG_OD_IN_PLASTIC"], L["pocket_6809_depth"] + 1, L["z_pocket_6809_bot"])  # 6809 pocket
    for (x, y) in polar(4, P["MOTOR_HOLE_SP"]/math.sqrt(2), math.radians(45)):       # NEMA17: 31 mm square
        cup -= cyl(P["MOTOR_SCREW_D"] + P["FIT_SCREW_CLR"], P["FLOOR_T"] + 2, -1, x, y)
        # counterbore from the inside face all the way down: the head sits BELOW the 6809 pocket floor,
        # the bore notches the pocket floor (not its wall) -> install motor screws before the 6809
        cup -= cyl(P["MOTOR_CBORE_D"], P["FLOOR_T"], P["MOTOR_FLOOR_UNDER_HEAD"], x, y)
    for (x, y) in polar(P["N_PINS"], P["R_PIN"]):
        cup -= cyl(P["PIN_D"] + P["FIT_PIN_PRESS_REAR"], P["PIN_SEAT_REAR"] + 1, P["FLOOR_T"] - P["PIN_SEAT_REAR"], x, y)
    for (x, y) in polar(P["N_HOUSING_BOLTS"], P["HOUSING_BOLT_PCD"]/2):
        cup -= cyl(P["INSERT_HOLE_D"], P["INSERT_HOLE_DEPTH"] + 1, L["z_shell_top"] - P["INSERT_HOLE_DEPTH"], x, y)
        cup -= csk(P["INSERT_CSK_D"], P["INSERT_HOLE_D"], L["z_shell_top"]).moved(Pos(x, y, 0))
    parts["rear_cup"] = cup

    # ---------------------------------------------------------- front plate ---
    fp = cyl(P["CUP_OD"], L["front_plate_T"], L["z_front_rear"])
    ob = P["BRG_OUTPUT"]
    fp -= cyl(ob["od"] - P["FIT_BRG_OD_IN_PLASTIC"], L["z_lip"][0] - L["z_front_rear"] + 1, L["z_front_rear"] - 1)  # bearing bore, open to rear
    fp -= cyl(P["LIP_ID"], L["front_plate_T"] + 2, L["z_front_rear"] - 1)                                       # lip opening
    for (x, y) in polar(P["N_PINS"], P["R_PIN"]):
        fp -= cyl(P["PIN_D"] + P["FIT_PIN_PRESS_FRONT"], P["PIN_SEAT_FRONT"] + 1, L["z_front_rear"] - 1, x, y)
    for (x, y) in polar(P["N_HOUSING_BOLTS"], P["HOUSING_BOLT_PCD"]/2):
        fp -= cyl(P["MOTOR_SCREW_D"] + P["FIT_SCREW_CLR"], L["front_plate_T"] + 2, L["z_front_rear"] - 1, x, y)
        fp -= cyl(P["HOUSING_CBORE_D"], P["HOUSING_CBORE_DEPTH"] + 1, L["z_front_face"] - P["HOUSING_CBORE_DEPTH"], x, y)
    parts["front_plate"] = fp

    # ------------------------------------------------------ eccentric shaft ---
    e = P["ECC"]
    fit = P["FIT_BRG_ID_ON_PLASTIC"]
    sh = cyl(P["SHAFT_REAR_D"], L["z_camA"][0] - L["z_shaft_rear"] + 0.01, L["z_shaft_rear"])
    sh += cyl(P["SHAFT_CAM_D"] + fit, L["z_camA"][1] - L["z_camA"][0], L["z_camA"][0], +e, 0)
    # 45-degree cones under the steps so the shaft prints rear-end-down without supports
    r_cam = (P["SHAFT_CAM_D"] + fit)/2
    rb = P["SHAFT_REAR_D"]/2 - e                          # largest circle about the cam axis inside the journal
    sh += Cone(rb, r_cam, r_cam - rb, align=A0).moved(Pos(+e, 0, L["z_camA"][0] - (r_cam - rb)))
    r_col = P["SHAFT_COLLAR_D"]/2
    rb2 = r_cam - e                                       # largest circle about the axis inside cam A
    sh += Cone(rb2, r_col - 0.5, r_col - 0.5 - rb2, align=A0).moved(Pos(0, 0, L["z_collar"][0] - (r_col - 0.5 - rb2)))
    sh += cyl(P["SHAFT_COLLAR_D"], L["z_collar"][1] - L["z_collar"][0] + 0.02, L["z_collar"][0] - 0.01)
    sh += cyl(P["SHAFT_CAM_D"] + fit, L["z_camB"][1] - L["z_camB"][0], L["z_camB"][0], -e, 0)
    sh += cyl(P["SHAFT_FRONT_D"] + fit, L["z_shaft_front"] - L["z_camB"][1] + 0.01, L["z_camB"][1] - 0.01)
    d = P["MOTOR_SHAFT_D"] + 0.10                      # D-bore, 0.1 dia clearance, flat on +X
    flat = P["MOTOR_FLAT"] - 0.05
    dbore = cyl(d, L["d_bore_depth"] + 1, L["z_shaft_rear"] - 1)
    dbore -= Box(4, d + 2, L["d_bore_depth"] + 3, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
        Pos(d/2 - flat, 0, L["z_shaft_rear"] - 2))
    parts["eccentric_shaft"] = sh - dbore

    # ------------------------------------------------------------ disc (x2) ---
    # One spline edge per lobe (a single 2000-point periodic spline gives a bad face in OCC),
    # each through pts_per_lobe+1 points of the equidistant profile; adjacent edges share endpoints.
    npl = 80
    xs, ys = cycloid_profile(P["R_PIN"], P["PIN_D"]/2, P["ECC"], P["N_LOBES"], P["N_PINS"],
                             P["PROFILE_CLR"], pts_per_lobe=npl)
    ntot = len(xs)
    edges = []
    for i in range(P["N_LOBES"]):
        idx = [(i*npl + k) % ntot for k in range(npl + 1)]
        edges.append(Spline(*[(float(xs[j]), float(ys[j]), 0.0) for j in idx]))
    face = Face(Wire(edges))
    body = extrude(face, amount=P["DISC_T"])
    db = P["DISC_BRG"]
    bore_d = db["od"] + P["DISC_BRG_CLR"]
    ledge_z = (P["DISC_T"] - db["w"])/2                        # bearing centred in the disc thickness
    body -= cyl(bore_d, P["DISC_T"] - ledge_z + 1, ledge_z)    # bore from the open face down to the ledge
    body -= cyl(bore_d - 2*P["DISC_LEDGE"], P["DISC_T"] + 2, -1)
    for (x, y) in polar(P["N_OUT"], P["R_OUT"]):
        body -= cyl(L["disc_hole_d"], P["DISC_T"] + 2, -1, x, y)
    parts["disc"] = body

    # --------------------------------------------------- carrier rear plate ---
    cr = cyl(P["CARRIER_OD"], P["CARRIER_T"], L["z_carrier_rear"])
    cr += cyl(P["BRG_CARRIER"]["id"] + fit, P["CARRIER_BOSS_LEN"] + 0.01, L["z_boss_rear"])
    cr -= cyl(P["CARRIER_BORE_D"], 40, L["z_boss_rear"] - 1)
    for (x, y) in polar(P["N_OUT"], P["R_OUT"]):
        cr -= cyl(P["OUT_PIN_D"] + P["FIT_OUTPIN_PRESS"], P["CARRIER_T"] + 2, L["z_carrier_rear"] - 1, x, y)
    parts["carrier_rear_plate"] = cr

    # -------------------------------------------------------- output flange ---
    fl = cyl(P["FLANGE_BODY_OD"], P["FLANGE_BODY_T"], L["z_flange_rear"])
    fl += cyl(P["BRG_OUTPUT"]["id"] + fit, L["flange_boss_len"] + 0.01, L["z_flange_body_front"] - 0.01)
    fb = P["BRG_FRONT_IN"]
    fl -= cyl(fb["od"] - P["FIT_BRG_OD_IN_PLASTIC"], fb["w"] + 1, L["z_flange_rear"] - 1)      # 608 pocket from rear
    for (x, y) in polar(P["N_OUT"], P["R_OUT"]):
        fl -= cyl(P["OUT_PIN_D"] + P["FIT_OUTPIN_PRESS"], P["FLANGE_PIN_DEPTH"] + 1, L["z_flange_rear"] - 1, x, y)
    for (x, y) in polar(P["FLANGE_IFACE_N"], P["FLANGE_IFACE_PCD"]/2):
        fl -= cyl(P["INSERT_HOLE_D"], P["INSERT_HOLE_DEPTH"] + 1, L["z_flange_face"] - P["INSERT_HOLE_DEPTH"], x, y)
        fl -= csk(P["INSERT_CSK_D"], P["INSERT_HOLE_D"], L["z_flange_face"]).moved(Pos(x, y, 0))
    fl -= cyl(P["FLANGE_PILOT_D"], P["FLANGE_PILOT_DEPTH"] + 1, L["z_flange_face"] - P["FLANGE_PILOT_DEPTH"])
    parts["output_flange"] = fl

    # -------------------------------------------------------------- washers ---
    rid = P["ROLLER"]["id"]
    for name, t in [("washer_end", L["washer_1"]), ("washer_mid", L["washer_mid"])]:
        parts[name] = cyl(11.5, t, 0) - cyl(rid + 0.3, t + 2, -1)     # bears on the inner race only (race OD ~12.1)

    # ---------------------------------------------------------- fit coupons ---
    parts["coupon_pins_inserts"] = coupon_pins_inserts()
    parts["coupon_bearings"] = coupon_bearings()

    # ---------------------------------------------------------------- export ---
    if not out_dir:                      # build only (used by check_geometry.py / render_section.py)
        return parts, L
    with open(os.path.join(out_dir, "layout.json"), "w") as f:
        json.dump(L, f, indent=1)
    for name, solid in parts.items():
        export_step(solid, os.path.join(out_dir, f"{name}.step"))
        export_stl(solid, os.path.join(out_dir, f"{name}.stl"), tolerance=0.02, angular_tolerance=0.1)
        print(f"exported {name}: volume {solid.volume:.0f} mm^3, valid={solid.is_valid}", flush=True)

    if not quick:
        asm = assemble(parts, L)
        export_step(asm, os.path.join(out_dir, "assembly.step"))
        export_stl(asm, os.path.join(out_dir, "assembly.stl"), tolerance=0.05, angular_tolerance=0.2)
        print("exported assembly", flush=True)
    return parts, L


def assemble(parts, L):
    """Place the parts in their nominal positions (input angle theta = 0)."""
    from build123d import Compound, Pos, Cylinder, Align, Axis
    e = P["ECC"]
    ratio = P["N_LOBES"]
    items = []
    items.append(parts["rear_cup"])
    items.append(parts["front_plate"])
    items.append(parts["eccentric_shaft"])
    items.append(parts["carrier_rear_plate"])
    items.append(parts["output_flange"])
    # discs: A on cam A (+e), B on cam B (-e), rotated 180/N so that B's lobes are phased for -e
    dA = parts["disc"].moved(Pos(+e, 0, L["z_discA"][0]))
    dB = parts["disc"].rotate(Axis.Z, 180.0/ratio).moved(Pos(-e, 0, L["z_discB"][0]))
    items += [dA, dB]
    # steel pins / bearings as simple cylinders for the section view
    def c(d, z0, z1, x=0, y=0):
        return Cylinder(d/2, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(x, y, z0))
    for i in range(P["N_PINS"]):
        a = 2*math.pi*i/P["N_PINS"]
        items.append(c(P["PIN_D"], *L["z_ringpin"], P["R_PIN"]*math.cos(a), P["R_PIN"]*math.sin(a)))
    for i in range(P["N_OUT"]):
        a = 2*math.pi*i/P["N_OUT"]
        x, y = P["R_OUT"]*math.cos(a), P["R_OUT"]*math.sin(a)
        items.append(c(P["OUT_PIN_D"], *L["z_outpin"], x, y))
        for zz in (L["z_rollerA"], L["z_rollerB"]):
            items.append(c(P["ROLLER"]["od"], *zz, x, y) - c(P["ROLLER"]["id"], zz[0]-1, zz[1]+1, x, y))
    db = P["DISC_BRG"]
    items.append(c(db["od"], *L["z_camA"], +e, 0) - c(db["id"], L["z_camA"][0]-1, L["z_camA"][1]+1, +e, 0))
    items.append(c(db["od"], *L["z_camB"], -e, 0) - c(db["id"], L["z_camB"][0]-1, L["z_camB"][1]+1, -e, 0))
    for b, zz in [(P["BRG_REAR_IN"], (L["z_brg_rear_bot"], L["z_brg_rear_top"])),
                  (P["BRG_CARRIER"], (L["z_pocket_6809_bot"], L["z_pocket_6809_bot"] + P["BRG_CARRIER"]["w"])),
                  (P["BRG_OUTPUT"], L["z_6816"]),
                  (P["BRG_FRONT_IN"], (L["z_flange_rear"], L["z_flange_608_bot"]))]:
        items.append(c(b["od"], *zz) - c(b["id"], zz[0]-1, zz[1]+1))
    return Compound(children=items)


# ------------------------------------------------------------ fit coupons ---
def coupon_pins_inserts():
    """Small plate: 4 mm and 8 mm pin holes in 0.05 steps, M3 insert holes, M3 clearance.
    Print in the SAME material/profile as the cup/front plate/carrier."""
    from build123d import Box, Cylinder, Align, Pos, Text, extrude, BuildPart, BuildSketch, Mode, Plane
    T = 10.0
    plate = Box(64, 40, T, align=(Align.MIN, Align.MIN, Align.MIN))
    def hole(d, x, y, depth=T+2, z0=-1):
        return Cylinder(d/2, depth, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(x, y, z0))
    for i, d in enumerate([3.90, 3.95, 4.00, 4.05, 4.10]):
        plate = plate - hole(d, 8 + 12*i, 8)                              # 4 mm ring pins
    for i, d in enumerate([7.90, 7.95, 8.00, 8.05, 8.10]):
        plate = plate - hole(d, 8 + 12*i, 20)                             # 8 mm output pins
    for i, d in enumerate([3.9, 4.0, 4.1, 4.2]):
        plate = plate - hole(d, 8 + 12*i, 32, depth=7 + 1, z0=T - 7)      # M3 insert holes (7 deep)
    plate = plate - hole(3.4, 56, 32)                                     # M3 clearance
    return plate


def coupon_bearings():
    """Bearing pocket test (608 OD, 6902 OD, 6801 OD) and journal test (608 ID, 6902 ID, 6809 ID boss).
    Print in the material of the discs / shaft / flange respectively."""
    from build123d import Box, Cylinder, Align, Pos
    T = 8.0
    plate = Box(130, 58, T, align=(Align.MIN, Align.MIN, Align.MIN))
    def pocket(d, x, y):
        return Cylinder(d/2, 8, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(x, y, T - 7))
    for i, d in enumerate([21.85, 21.95, 22.05]):      # 608 outer-race pockets (disc/flange material)
        plate = plate - pocket(d, 14 + 26*i, 16)
    plate = plate - pocket(27.95, 100, 16)             # 6902 outer-race pocket, nominal press (disc material)
    plate = plate - pocket(20.95, 118, 44)             # 6801 pocket (cup material)
    # journals (pegs): 608 bore (front journal) and 6902 bore (cams), 6 mm tall
    def peg(d, x, y):
        return Cylinder(d/2, 6, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(x, y, T))
    for i, d in enumerate([7.95, 8.00, 8.05, 8.10]):
        plate = plate + peg(d, 8 + 12*i, 44)
    for i, d in enumerate([14.95, 15.00, 15.05]):
        plate = plate + peg(d, 62 + 16*i, 44)
    return plate


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "..", "cad")
    build_all(out, quick="--quick" in sys.argv)
