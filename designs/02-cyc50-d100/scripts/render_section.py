#!/usr/bin/env python3
"""
render_section.py -- true cross-section of the CAD assembly (plane y = 0, through the eccentric axis,
output pin 0 / 3 and ring pin 0), rendered to ../renders/assembly_section.png, plus a labelled
axial layout diagram built from layout.json (../renders/axial_layout.png).
"""
import os, sys, math, json
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
from build123d import Plane, Cylinder, Align, Pos, Axis
from cad import P, layout, build_all

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "renders"); os.makedirs(OUT, exist_ok=True)
L = layout(P)
parts, _ = build_all(None)
e = P["ECC"]

def c(d, z0, z1, x=0, y=0):
    return Cylinder(d/2, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(x, y, z0))
def ring(od, idd, z0, z1, x=0, y=0):
    return c(od, z0, z1, x, y) - c(idd, z0 - 1, z1 + 1, x, y)

COL = {"printed": "#7fb3d5", "printed2": "#a9cce3", "disc": "#f5b041", "steel": "#7b7d7d", "bearing": "#58d68d", "shaft": "#e59866"}
items = []   # (solid, colour, label)
items.append((parts["rear_cup"], COL["printed"], "rear cup (motor plate + shell)"))
items.append((parts["front_plate"], COL["printed"], "front plate"))
items.append((parts["eccentric_shaft"], COL["shaft"], "eccentric shaft"))
items.append((parts["carrier_rear_plate"], COL["printed2"], "carrier rear plate"))
items.append((parts["output_flange"], COL["printed2"], "output flange"))
items.append((parts["disc"].moved(Pos(+e, 0, L["z_discA"][0])), COL["disc"], "disc A (+e)"))
items.append((parts["disc"].rotate(Axis.Z, 180.0/P["N_LOBES"]).moved(Pos(-e, 0, L["z_discB"][0])), COL["disc"], "disc B (-e)"))
for i in range(P["N_PINS"]):
    a = 2*math.pi*i/P["N_PINS"]
    items.append((c(P["PIN_D"], *L["z_ringpin"], P["R_PIN"]*math.cos(a), P["R_PIN"]*math.sin(a)), COL["steel"], "ring pin" if i == 0 else None))
for i in range(P["N_OUT"]):
    a = 2*math.pi*i/P["N_OUT"]; x, y = P["R_OUT"]*math.cos(a), P["R_OUT"]*math.sin(a)
    items.append((c(P["OUT_PIN_D"], *L["z_outpin"], x, y), COL["steel"], "output pin" if i == 0 else None))
    for zz in (L["z_rollerA"], L["z_rollerB"]):
        items.append((ring(P["ROLLER"]["od"], P["ROLLER"]["id"], *zz, x, y), COL["bearing"], (P["ROLLER"]["name"]+" x%d"%P["ROLLER_N"]) if i == 0 and zz == L["z_rollerA"] else None))
db = P["DISC_BRG"]
items.append((ring(db["od"], db["id"], *L["z_camA"], +e, 0), COL["bearing"], P["DISC_BRG"]["name"]+" (cam A)"))
items.append((ring(db["od"], db["id"], *L["z_camB"], -e, 0), COL["bearing"], P["DISC_BRG"]["name"]+" (cam B)"))
items.append((ring(P["BRG_REAR_IN"]["od"], P["BRG_REAR_IN"]["id"], L["z_brg_rear_bot"], L["z_brg_rear_top"]), COL["bearing"], P["BRG_REAR_IN"]["name"]))
items.append((ring(P["BRG_CARRIER"]["od"], P["BRG_CARRIER"]["id"], L["z_pocket_carrier_bot"], L["z_pocket_carrier_bot"] + P["BRG_CARRIER"]["w"]), COL["bearing"], P["BRG_CARRIER"]["name"]))
items.append((ring(P["BRG_OUTPUT"]["od"], P["BRG_OUTPUT"]["id"], *L["z_brg_output"]), COL["bearing"], P["BRG_OUTPUT"]["name"]))
items.append((ring(P["BRG_FRONT_IN"]["od"], P["BRG_FRONT_IN"]["id"], L["z_flange_rear"], L["z_flange_608_bot"]), COL["bearing"], P["BRG_FRONT_IN"]["name"]+" (front journal)"))
# washers on output pin 0
x0 = P["R_OUT"]
for (z0, t) in [(L["z_carrier_front"], L["washer_1"]), (L["z_rollerA"][1], L["washer_mid"]), (L["z_rollerB"][1], L["washer_3"])]:
    items.append((ring(P["ROLLER"]["id"] + 2.5, P["ROLLER"]["id"] + 0.3, z0, z0 + t, x0, 0), "#c39bd3", "washer" if z0 == L["z_carrier_front"] else None))
if P["MID_RING"]:
    items.append((parts["mid_pin_ring"], "#f1948a", "mid pin ring"))
# motor outline (not printed)
mot = Cylinder(P["MOTOR_BOSS_D"]/2, P["MOTOR_BOSS_H"], align=(Align.CENTER, Align.CENTER, Align.MAX))
items.append((mot, "#d5d8dc", "motor pilot boss"))
items.append((c(P["MOTOR_SHAFT_D"], -P["MOTOR_BOSS_H"] - 0.001, P["MOTOR_SHAFT_L"]), "#d5d8dc", "motor shaft"))

fig, ax = plt.subplots(figsize=(16, 11))
sec_plane = Plane.XZ          # y = 0
labels_done = {}
for solid, colour, label in items:
    try:
        sec = solid.intersect(sec_plane)
    except Exception:
        continue
    faces = sec.faces() if sec is not None else []
    for f in faces:
        for w in [f.outer_wire()] + list(f.inner_wires()):
            pts = []
            for ed in w.edges():
                for t in np.linspace(0, 1, 12, endpoint=False):
                    p = ed.position_at(t); pts.append((p.X, p.Z))
            if len(pts) < 3:
                continue
            is_outer = (w == f.outer_wire()) or (w.length == f.outer_wire().length)
            ax.add_patch(Polygon(pts, closed=True, facecolor=colour if is_outer else "white", edgecolor="k", lw=0.5, zorder=2 if is_outer else 3))
        if label and label not in labels_done:
            cc = f.center(); labels_done[label] = (cc.X, cc.Z)
for label, (x, z) in labels_done.items():
    ax.annotate(label, (x, z), fontsize=7, ha="center", va="center", zorder=5,
                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.75))
ax.set_aspect("equal"); ax.set_xlim(-72, 72); ax.set_ylim(-6, L["overall_length"] + 4)
ax.set_xlabel("x [mm]  (section plane y = 0; eccentric of cam A points +x)"); ax.set_ylabel("z [mm]  (0 = motor face)")
ax.set_title("True CAD cross-section of the assembly at theta = 0  (steel: grey, bearings: green, discs: orange, printed: blue)")
ax.grid(alpha=0.3)
fig.tight_layout(); fig.savefig(os.path.join(OUT, "assembly_section.png"), dpi=140); plt.close(fig)
print("wrote assembly_section.png")

# ------------------------------------------------ axial layout diagram (from layout.json) --
fig, ax = plt.subplots(figsize=(10, 12))
rows = [
    ("motor face", 0, 0), ("pilot recess", 0, P["PILOT_DEPTH"]),
    (P["BRG_REAR_IN"]["name"]+" (rear input brg)", L["z_brg_rear_bot"], L["z_brg_rear_top"]),
    (P["BRG_CARRIER"]["name"]+" pocket (carrier)", L["z_pocket_carrier_bot"], P["FLOOR_T"]),
    ("cup floor", 0, P["FLOOR_T"]), ("ring pin seat (rear)", L["z_ringpin"][0], P["FLOOR_T"]),
    ("carrier rear plate", L["z_carrier_rear"], L["z_carrier_front"]),
    ("disc A", *L["z_discA"]), ("cam A / "+P["DISC_BRG"]["name"], *L["z_camA"]),
    ("collar", *L["z_collar"]),
    ("disc B", *L["z_discB"]), ("cam B / "+P["DISC_BRG"]["name"], *L["z_camB"]),
    ("output flange body", L["z_flange_rear"], L["z_flange_body_front"]),
    (P["BRG_FRONT_IN"]["name"]+" front journal", L["z_flange_rear"], L["z_flange_608_bot"]),
    ("front plate", L["z_front_rear"], L["z_front_face"]),
    ("ring pin seat (front)", L["z_front_rear"], L["z_ringpin"][1]),
    (P["BRG_OUTPUT"]["name"]+" output bearing", *L["z_brg_output"]), ("lip", *L["z_lip"]),
    ("flange bearing boss", L["z_flange_body_front"], L["z_face_plate"][0]),
    ("arm face plate", *L["z_face_plate"]),
    ("motor shaft (usable)", 0, P["MOTOR_SHAFT_L"]), ("D-bore", L["z_shaft_rear"], L["z_shaft_rear"] + L["d_bore_depth"]),
    ("eccentric shaft", L["z_shaft_rear"], L["z_shaft_front"]),
    ("ring pins %gx%g" % (P["PIN_D"], P["PIN_L"]), *L["z_ringpin"]), ("output pins %gx%g" % (P["OUT_PIN_D"], P["OUT_PIN_L"]), *L["z_outpin"]),
]
for i, (name, z0, z1) in enumerate(rows):
    ax.barh(i, max(z1 - z0, 0.3), left=z0, height=0.6, color=plt.cm.tab20(i % 20))
    ax.text(z1 + 0.5, i, f"{name}: {z0:.1f} .. {z1:.1f}", va="center", fontsize=8)
ax.set_yticks([]); ax.set_xlabel("z [mm] from motor face"); ax.set_xlim(-2, L["overall_length"] + 45)
ax.set_title("Axial stack (all numbers from layout.json)"); ax.grid(axis="x", alpha=0.3)
fig.tight_layout(); fig.savefig(os.path.join(OUT, "axial_layout.png"), dpi=130); plt.close(fig)
print("wrote axial_layout.png")
