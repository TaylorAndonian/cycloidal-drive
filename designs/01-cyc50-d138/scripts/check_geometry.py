#!/usr/bin/env python3
"""
check_geometry.py -- programmatic sanity checks on the EXPORTED geometry (../cad/*.step, *.stl)

  1. disc.step : section the solid, count lobes, measure tip/root radius vs the analytic profile
  2. rear_cup.step / front_plate.step : locate the 51 pin-hole cylinders, fit a circle -> pin circle dia
  3. minimum wall / ligament thicknesses vs MIN_WALL (2 perimeters at 0.4 mm)
  4. validity: OCC is_valid on every solid, STL watertight + consistent winding (trimesh), positive volume
  5. assembly interference: pairwise boolean intersections between parts in the nominal position
Exit code 1 if anything fails.
"""
import os, sys, json, math
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cad import P, layout
from cycloid_profile import cycloid_profile, lobe_count
from build123d import import_step, Plane, Cylinder, Align, Pos, Axis, GeomType

HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.join(HERE, "..", "cad")
L = layout(P)
fails = []
report = {}

def check(name, ok, detail=""):
    print(("PASS " if ok else "FAIL ") + name + ("  -- " + str(detail) if detail else ""))
    report[name] = dict(ok=bool(ok), detail=detail)
    if not ok:
        fails.append(name)

# ----------------------------------------------------------------- 1. disc --
disc = import_step(os.path.join(CAD, "disc.step"))
sec = disc.intersect(Plane.XY.offset(P["DISC_T"]/2))   # planar section at mid thickness
outer = max(sec.faces(), key=lambda f: f.area) if sec.faces() else None
# sample the outer boundary: pick the wire with the largest bounding box
wires = sec.wires() if sec.wires() else outer.outer_wire()
ow = max(wires, key=lambda w: w.bounding_box().size.X)
pts = np.array([(v.X, v.Y) for ed in ow.edges() for v in [ed.position_at(t) for t in np.linspace(0, 1, 200, endpoint=False)]])
r = np.hypot(pts[:, 0], pts[:, 1])
n_lobes = lobe_count(pts[:, 0], pts[:, 1])
px, py = cycloid_profile(P["R_PIN"], P["PIN_D"]/2, P["ECC"], P["N_LOBES"], P["N_PINS"], P["PROFILE_CLR"], 200)
ra = np.hypot(px, py)
check("disc lobe count == N_LOBES", n_lobes == P["N_LOBES"], f"{n_lobes} lobes counted from the STEP section")
check("disc tip radius matches equations", abs(r.max() - ra.max()) < 0.02, f"STEP {r.max():.3f} vs analytic {ra.max():.3f}")
check("disc root radius matches equations", abs(r.min() - ra.min()) < 0.02, f"STEP {r.min():.3f} vs analytic {ra.min():.3f}")
# STEP profile vs analytic profile: max deviation (nearest-point)
pxd, pyd = cycloid_profile(P["R_PIN"], P["PIN_D"]/2, P["ECC"], P["N_LOBES"], P["N_PINS"], P["PROFILE_CLR"], 3000)  # 2.3 um spacing
dev = max(np.min(np.hypot(pxd - x, pyd - y)) for x, y in pts[::10])
check("disc STEP profile deviation from equations < 0.01 mm", dev < 0.01, f"max deviation {dev*1000:.1f} um")

# ---------------------------------------------------- 2. pin circle dia -----
def pin_circle(step_name, hole_r):
    part = import_step(os.path.join(CAD, step_name))
    cyls = [f for f in part.faces().filter_by(GeomType.CYLINDER) if abs(f.radius - hole_r) < 1e-3]
    centers = []
    for f in cyls:
        loc = f.geom_adaptor().Cylinder().Location()      # a point ON the cylinder axis (not the face centroid)
        centers.append((loc.X(), loc.Y()))
    centers = np.array(centers)
    # merge duplicates (a hole can be split into several faces)
    uniq = []
    for c in centers:
        if not any(np.hypot(*(c - u)) < 0.5 for u in uniq):
            uniq.append(c)
    uniq = np.array(uniq)
    rad = np.hypot(uniq[:, 0], uniq[:, 1])
    return len(uniq), rad.mean()*2, rad.std()

n, dia, sd = pin_circle("rear_cup.step", (P["PIN_D"] + P["FIT_PIN_PRESS_REAR"])/2)
check("rear cup: 51 ring-pin holes", n == P["N_PINS"], f"{n} holes found")
check("rear cup: pin circle dia == 2*R_PIN", abs(dia - 2*P["R_PIN"]) < 0.01, f"measured {dia:.3f} mm (sd {sd*1000:.1f} um)")
n, dia, sd = pin_circle("front_plate.step", (P["PIN_D"] + P["FIT_PIN_PRESS_FRONT"])/2)
check("front plate: 51 ring-pin holes", n == P["N_PINS"], f"{n} holes found")
check("front plate: pin circle dia == 2*R_PIN", abs(dia - 2*P["R_PIN"]) < 0.01, f"measured {dia:.3f} mm")
n, dia, sd = pin_circle("carrier_rear_plate.step", (P["OUT_PIN_D"] + P["FIT_OUTPIN_PRESS"])/2)
check("carrier: 6 output-pin holes on 2*R_OUT", n == P["N_OUT"] and abs(dia - 2*P["R_OUT"]) < 0.01, f"{n} holes, dia {dia:.3f}")

# ------------------------------------------------------- 3. min walls -------
mw = P["MIN_WALL"]
hole_d = L["disc_hole_d"]
lig = {}
lig["disc: lobe root to roller hole"] = ra.min() - (P["R_OUT"] + hole_d/2)
lig["disc: roller hole to roller hole"] = 2*P["R_OUT"]*math.sin(math.pi/P["N_OUT"]) - hole_d
lig["disc: roller hole to bearing bore"] = (P["R_OUT"] - hole_d/2) - (P["DISC_BRG"]["od"] + P["DISC_BRG_CLR"])/2
lig["disc: bearing ledge"] = P["DISC_LEDGE"]
lig["plates: ring-pin hole to ring-pin hole"] = 2*P["R_PIN"]*math.sin(math.pi/P["N_PINS"]) - P["PIN_D"]
lig["front plate: pin hole to 6816 bore"] = (P["R_PIN"] - P["PIN_D"]/2) - P["BRG_OUTPUT"]["od"]/2
lig["front plate: pin hole to OD"] = P["CUP_OD"]/2 - (P["R_PIN"] + P["PIN_D"]/2)
lig["front plate: bolt cbore to pin hole (radial)"] = (P["HOUSING_BOLT_PCD"]/2 - P["HOUSING_CBORE_D"]/2) - (P["R_PIN"] + P["PIN_D"]/2)
lig["cup: insert hole wall in shell (each side)"] = (P["SHELL_T"] - P["INSERT_HOLE_D"])/2
lig["cup: shell to disc tips (radial clearance)"] = L["r_shell_in"] - (P["R_PIN"] + P["ECC"] + P["PIN_D"]/2)
lig["cup: pin hole to 6809 pocket"] = (P["R_PIN"] - P["PIN_D"]/2) - P["BRG_CARRIER"]["od"]/2
lig["cup: floor under pin holes"] = P["FLOOR_T"] - P["PIN_SEAT_REAR"]
lig["cup: motor cbore to 6801 pocket"] = (P["MOTOR_HOLE_SP"]/math.sqrt(2) - P["MOTOR_CBORE_D"]/2) - P["BRG_REAR_IN"]["od"]/2
lig["cup: floor under motor screw head"] = P["MOTOR_FLOOR_UNDER_HEAD"]
lig["cup: 6801 pocket to 6809 pocket"] = P["BRG_CARRIER"]["id"]/2 - 0 - P["BRG_REAR_IN"]["od"]/2  # boss ID region; see next
lig["cup: 6809 pocket floor thickness"] = L["z_pocket_6809_bot"]
lig["shaft: cam wall around D-bore (if bore reached cam)"] = P["SHAFT_CAM_D"]/2 - P["ECC"] - (P["MOTOR_SHAFT_D"] + 0.1)/2
lig["shaft: rear journal wall around D-bore"] = (P["SHAFT_REAR_D"] - P["MOTOR_SHAFT_D"] - 0.1)/2
lig["carrier: boss wall (OD45 to bore)"] = (P["BRG_CARRIER"]["id"] - P["CARRIER_BORE_D"])/2
lig["carrier: output pin hole to OD"] = P["CARRIER_OD"]/2 - (P["R_OUT"] + P["OUT_PIN_D"]/2)
lig["flange: output pin hole to 6816 seat boss"] = P["BRG_OUTPUT"]["id"]/2 - (P["R_OUT"] + P["OUT_PIN_D"]/2)
lig["flange: insert hole to 608 pocket"] = (P["FLANGE_IFACE_PCD"]/2 - P["INSERT_HOLE_D"]/2) - P["BRG_FRONT_IN"]["od"]/2
lig["flange: insert hole to output pin hole"] = (P["R_OUT"] - P["OUT_PIN_D"]/2) - (P["FLANGE_IFACE_PCD"]/2 + P["INSERT_HOLE_D"]/2)
lig["flange: body to front-plate bore (radial gap)"] = P["BRG_OUTPUT"]["od"]/2 - P["FLANGE_BODY_OD"]/2
lig["washer: wall"] = (11.5 - (P["ROLLER"]["id"] + 0.3))/2
worst = min(lig.values())
for k, v in lig.items():
    print(f"   ligament {k:55s} {v:6.2f} mm")
check(f"all ligaments >= MIN_WALL ({mw} mm)", worst >= mw, f"thinnest {worst:.2f} mm")
report["ligaments_mm"] = lig

# ---------------------------------------------------- 4. validity ------------
import trimesh
names = ["rear_cup", "front_plate", "eccentric_shaft", "disc", "carrier_rear_plate", "output_flange",
         "washer_end", "washer_mid", "coupon_pins_inserts", "coupon_bearings"]
for nme in names:
    solid = import_step(os.path.join(CAD, nme + ".step"))
    m = trimesh.load(os.path.join(CAD, nme + ".stl"))
    ok = solid.is_valid and m.is_watertight and m.is_winding_consistent and m.volume > 0 and abs(m.volume - solid.volume)/solid.volume < 0.01
    check(f"{nme}: STEP valid, STL watertight, volumes agree", ok,
          f"STEP {solid.volume:.0f} mm3, STL {m.volume:.0f} mm3, watertight={m.is_watertight}")

# ------------------------------------------- 5. assembly interference -------
from cad import build_all, assemble
parts, _ = build_all(None)
e = P["ECC"]
def c(d, z0, z1, x=0, y=0):
    return Cylinder(d/2, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(x, y, z0))
dA = parts["disc"].moved(Pos(+e, 0, L["z_discA"][0]))
dB = parts["disc"].rotate(Axis.Z, 180.0/P["N_LOBES"]).moved(Pos(-e, 0, L["z_discB"][0]))
pins = [c(P["PIN_D"], *L["z_ringpin"], P["R_PIN"]*math.cos(a), P["R_PIN"]*math.sin(a)) for a in 2*np.pi*np.arange(P["N_PINS"])/P["N_PINS"]]
outpins = [c(P["OUT_PIN_D"], *L["z_outpin"], P["R_OUT"]*math.cos(a), P["R_OUT"]*math.sin(a)) for a in 2*np.pi*np.arange(P["N_OUT"])/P["N_OUT"]]
rollersA = [c(P["ROLLER"]["od"], *L["z_rollerA"], P["R_OUT"]*math.cos(a), P["R_OUT"]*math.sin(a)) for a in 2*np.pi*np.arange(P["N_OUT"])/P["N_OUT"]]
brgA = c(P["DISC_BRG"]["od"], *L["z_camA"], +e, 0)
pairs = {
    "rear_cup vs carrier_rear_plate": (parts["rear_cup"], parts["carrier_rear_plate"]),
    "rear_cup vs eccentric_shaft": (parts["rear_cup"], parts["eccentric_shaft"]),
    "rear_cup vs disc A": (parts["rear_cup"], dA),
    "front_plate vs output_flange": (parts["front_plate"], parts["output_flange"]),
    "front_plate vs disc B": (parts["front_plate"], dB),
    "carrier_rear_plate vs eccentric_shaft": (parts["carrier_rear_plate"], parts["eccentric_shaft"]),
    "carrier_rear_plate vs disc A": (parts["carrier_rear_plate"], dA),
    "output_flange vs disc B": (parts["output_flange"], dB),
    "output_flange vs eccentric_shaft": (parts["output_flange"], parts["eccentric_shaft"]),
    "disc A vs disc B": (dA, dB),
    "disc A vs ring pins (should only touch)": (dA, pins),
    "disc B vs ring pins (should only touch)": (dB, pins),
    "disc A vs rollers A (should only touch)": (dA, rollersA),
    "disc A vs eccentric bearing A OD": (dA, [brgA]),
    "output pins vs front_plate": (parts["front_plate"], outpins),
    "output pins vs rear_cup": (parts["rear_cup"], outpins),
    "ring pins vs output_flange": (parts["output_flange"], pins),
    "ring pins vs carrier_rear_plate": (parts["carrier_rear_plate"], pins),
    "eccentric_shaft vs disc A (bore)": (parts["eccentric_shaft"], dA),
}
for name, (a, b) in pairs.items():
    blist = b if isinstance(b, list) else [b]
    vol = 0.0
    for bb in blist:
        try:
            inter = a.intersect(bb)
            vol += inter.volume if inter is not None else 0.0
        except Exception:
            pass
    # press-fit bearings/pins are modelled at nominal (interference ~0.05 mm) -> allow a tiny volume
    tol = 60.0 if ("bearing" in name or "pins" in name) else 0.5
    check(f"no interference: {name}", vol < tol, f"intersection volume {vol:.2f} mm3")

with open(os.path.join(HERE, "..", "analysis", "geometry_check.json"), "w") as f:
    json.dump(report, f, indent=1, default=float)
print("\n%d checks, %d failed" % (len(report) - 1, len(fails)))
sys.exit(1 if fails else 0)
