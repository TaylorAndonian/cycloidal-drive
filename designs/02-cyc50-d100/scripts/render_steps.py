#!/usr/bin/env python3
"""
render_steps.py -- one cross-section per assembly stage, as SVG, for the build guide.
Every stage is the SAME true CAD section (plane y = 0); parts not yet fitted are omitted and the
final outline is drawn faintly behind so you can see what is still to come.
Writes ../renders/step_1..7.svg
"""
import os, sys, math
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
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

COL = dict(cup="#7fb3d5", plate="#7fb3d5", carrier="#a9cce3", flange="#a9cce3",
           disc="#f5b041", steel="#7b7d7d", brg="#58d68d", shaft="#e59866",
           washer="#c39bd3", motor="#d5d8dc")

# ---- every item in the machine, tagged with the stage it first appears in -----------------
ITEMS = []   # (stage, solid, colour, label)
def add(stage, solid, colour, label=None):
    ITEMS.append((stage, solid, colour, label))

mot_boss = Cylinder(P["MOTOR_BOSS_D"]/2, P["MOTOR_BOSS_H"], align=(Align.CENTER, Align.CENTER, Align.MAX))
add(1, mot_boss, COL["motor"], "motor")
add(1, c(P["MOTOR_SHAFT_D"], -P["MOTOR_BOSS_H"] - 0.001, P["MOTOR_SHAFT_L"]), COL["motor"], None)
add(1, parts["rear_cup"], COL["cup"], "rear cup")
add(3, ring(P["BRG_REAR_IN"]["od"], P["BRG_REAR_IN"]["id"], L["z_brg_rear_bot"], L["z_brg_rear_top"]), COL["brg"], P["BRG_REAR_IN"]["name"])
add(1, ring(P["BRG_CARRIER"]["od"], P["BRG_CARRIER"]["id"], L["z_pocket_carrier_bot"], L["z_pocket_carrier_bot"] + P["BRG_CARRIER"]["w"]), COL["brg"], P["BRG_CARRIER"]["name"])
for i in range(P["N_PINS"]):
    a = 2*math.pi*i/P["N_PINS"]
    add(2, c(P["PIN_D"], *L["z_ringpin"], P["R_PIN"]*math.cos(a), P["R_PIN"]*math.sin(a)), COL["steel"], "ring pins" if i == 0 else None)

add(3, parts["eccentric_shaft"], COL["shaft"], "eccentric shaft")
db = P["DISC_BRG"]
add(3, ring(db["od"], db["id"], *L["z_camA"], +e, 0), COL["brg"], db["name"] + " cam A")
add(3, ring(db["od"], db["id"], *L["z_camB"], -e, 0), COL["brg"], db["name"] + " cam B")
add(3, parts["carrier_rear_plate"], COL["carrier"], "carrier plate")

for i in range(P["N_OUT"]):
    a = 2*math.pi*i/P["N_OUT"]; x, y = P["R_OUT"]*math.cos(a), P["R_OUT"]*math.sin(a)
    add(4, c(P["OUT_PIN_D"], *L["z_outpin"], x, y), COL["steel"], "output pins" if i == 0 else None)
    for j, zz in enumerate((L["z_rollerA"], L["z_rollerB"])):
        mid = (zz[0] + zz[1])/2
        for k, (za, zb) in enumerate(((zz[0], mid), (mid, zz[1]))):
            add(4, ring(P["ROLLER"]["od"], P["ROLLER"]["id"], za, zb, x, y), COL["brg"],
                (P["ROLLER"]["name"] + " x4/pin") if (i == 0 and j == 0 and k == 0) else None)
    for (z0, t) in ((L["z_carrier_front"], L["washer_1"]), (L["z_rollerA"][1], L["washer_mid"]), (L["z_rollerB"][1], L["washer_3"])):
        add(4, ring(P["ROLLER"]["id"] + 2.5, P["ROLLER"]["id"] + 0.3, z0, z0 + t, x, y), COL["washer"],
            "washers" if (i == 0 and z0 == L["z_carrier_front"]) else None)

add(5, parts["disc"].moved(Pos(+e, 0, L["z_discA"][0])), COL["disc"], "disc A  (+e)")
add(6, parts["disc"].rotate(Axis.Z, 180.0/P["N_LOBES"]).moved(Pos(-e, 0, L["z_discB"][0])), COL["disc"], "disc B  (-e)")

add(7, parts["output_flange"], COL["flange"], "output flange")
add(7, ring(P["BRG_FRONT_IN"]["od"], P["BRG_FRONT_IN"]["id"], L["z_flange_rear"], L["z_flange_608_bot"]), COL["brg"], P["BRG_FRONT_IN"]["name"])
add(7, ring(P["BRG_OUTPUT"]["od"], P["BRG_OUTPUT"]["id"], *L["z_brg_output"]), COL["brg"], P["BRG_OUTPUT"]["name"])
add(8, parts["front_plate"], COL["plate"], "front plate")

TITLES = {
 1: "1 - Rear cup: inserts, motor, carrier bearing",
 2: "2 - Press the 51 ring pins",
 3: "3 - Shaft, cams, rear bearing + carrier plate",
 4: "4 - Output pins, rollers and washers",
 5: "5 - Disc A",
 6: "6 - Disc B (3.6 deg out of phase)",
 7: "7 - Output flange + its two bearings",
 8: "8 - Front plate closes the ring",
}

def section_polys(solid):
    """Return [(pts, is_outer)] of the y=0 section of one solid."""
    try:
        sec = solid.intersect(Plane.XZ)
    except Exception:
        return []
    out = []
    for f in (sec.faces() if sec is not None else []):
        ow = f.outer_wire()
        for w in [ow] + list(f.inner_wires()):
            pts = []
            for ed in w.edges():
                for t in np.linspace(0, 1, 7, endpoint=False):
                    p = ed.position_at(t); pts.append((p.X, p.Z))
            if len(pts) >= 3:
                out.append((pts, w.length == ow.length))
    return out

CACHE = {}
def polys(idx, solid):
    if idx not in CACHE:
        CACHE[idx] = section_polys(solid)
    return CACHE[idx]

for stage in sorted(TITLES):
    fig, ax = plt.subplots(figsize=(10.4, 6.6))
    # faint ghost of everything still to come
    for idx, (st, solid, colour, label) in enumerate(ITEMS):
        if st <= stage:
            continue
        for pts, is_outer in polys(idx, solid):
            if is_outer:
                ax.add_patch(Polygon(pts, closed=True, facecolor="none", edgecolor="#c8ccd0",
                                     lw=0.5, ls=(0, (3, 3)), zorder=1))
    labels_done = {}
    for idx, (st, solid, colour, label) in enumerate(ITEMS):
        if st > stage:
            continue
        new = (st == stage)
        for pts, is_outer in polys(idx, solid):
            ax.add_patch(Polygon(pts, closed=True,
                                 facecolor=(colour if is_outer else "white"),
                                 edgecolor=("#111111" if new else "#8a8f94"),
                                 lw=(1.1 if new else 0.5),
                                 alpha=(1.0 if new else 0.45),
                                 zorder=(3 if is_outer else 4)))
        if label and new and label not in labels_done:
            pts_all = [p for pl, _ in polys(idx, solid) for p in pl]
            right = [p for p in pts_all if p[0] > 0.5]
            use = right if right else pts_all
            if use:
                xs = [p[0] for p in use]; zs = [p[1] for p in use]
                labels_done[label] = (float((min(xs) + max(xs))/2), float((min(zs) + max(zs))/2))

    # ---- lay the callouts out so they never overlap: anchor stays put, the text is
    # ---- pushed out to a column and spread vertically, joined by a leader line
    items_lbl = sorted(labels_done.items(), key=lambda kv: kv[1][1])
    placed = []
    for label, (ax_, az_) in items_lbl:
        side = 1 if ax_ >= 0 else -1
        tx = side * 62.0
        tz = az_
        for _, pz in placed:
            if abs(tz - pz) < 4.6:
                tz = pz + 4.6
        placed.append((tx, tz))
        ax.annotate(label, xy=(ax_, az_), xytext=(tx, tz),
                    fontsize=7.5, ha=("left" if side > 0 else "right"), va="center", zorder=9,
                    bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#7a8288", lw=0.45, alpha=0.95),
                    arrowprops=dict(arrowstyle="-", lw=0.5, color="#7a8288",
                                    shrinkA=1, shrinkB=1, connectionstyle="arc3,rad=0"))
    ax.set_aspect("equal"); ax.set_xlim(-54, 94); ax.set_ylim(-5, L["overall_length"] + 3)
    ax.set_title(TITLES[stage], fontsize=10)
    ax.set_xlabel("mm from the axis", fontsize=8); ax.set_ylabel("mm from the motor face", fontsize=8)
    ax.tick_params(labelsize=7); ax.grid(alpha=0.18, lw=0.5)
    for s in ax.spines.values():
        s.set_linewidth(0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, f"step_{stage}.svg"))
    fig.savefig(os.path.join(OUT, f"step_{stage}.png"), dpi=160)
    plt.close(fig)
    print("wrote step_%d.svg" % stage, flush=True)
