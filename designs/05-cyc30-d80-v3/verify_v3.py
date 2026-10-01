import itertools, math
import numpy as np
import cyc30_v3 as CYC
from build123d import *

names = ["back_plate","housing","front_retainer","rear_ring","disc","input_shaft","cam_b",
         "shaft_spacer","output_flange","output_cap",
         f"roller_sleeve_{CYC.SLEEVE_A}", f"roller_sleeve_{CYC.SLEEVE_B}", f"roller_sleeve_{CYC.SLEEVE_C}"]
parts = {n: import_step(f"step_v3/{n}.step") for n in names}

print("=== 0. every printed part is one connected solid ===")
for n, p in parts.items():
    ns = len(p.solids())
    print(f"  {n:22s} solids={ns}" + ("" if ns == 1 else "   <<< DISCONNECTED"))
print("=== 1. mesh clearance (disc profile vs ring pins) ===")
P = np.array(CYC.disc_profile_pts(120))
pins = np.array([[CYC.R_PIN*np.cos(a), CYC.R_PIN*np.sin(a)] for a in 2*np.pi*np.arange(CYC.RATIO_PINS)/CYC.RATIO_PINS])
worst = 9
for th in np.linspace(0, 2*np.pi, 25):
    c = CYC.ECC*np.array([np.cos(th), np.sin(th)]); rot = -th/30
    M = np.array([[np.cos(rot), np.sin(rot)], [-np.sin(rot), np.cos(rot)]])
    Q = P @ M + c
    d = np.sqrt(((pins[:, None, :]-Q[None])**2).sum(-1)).min(1) - CYC.D_PIN/2
    worst = min(worst, d.min())
print(f"  min gap pin-surface to disc flank over a full turn: {worst*1000:.0f} microns (target ~{CYC.MESH_CLEAR*1000:.0f})")

print("=== 2. output roller clearance in the disc holes ===")
hole = CYC.ROLLER_OD + 2*CYC.ECC + CYC.OUT_CLEAR
print(f"  hole {hole:.2f} - roller {CYC.ROLLER_OD} - 2e {2*CYC.ECC} = {hole-CYC.ROLLER_OD-2*CYC.ECC:.2f} mm diametral slack")

print("=== 3. assembly interference ===")
INTENTIONAL = {("input_shaft+camA","6803_disc1"), ("cam_b","6803_disc2"),
               ("output_flange","6810_output"), ("input_shaft+camA","608_steady")}
for th in (0, 37, 90, 143, 212, 300):
    asm = CYC.assembly(parts, th)
    ch = [c for c in asm.children if c.label != "NEMA17_reference"]
    bbs = [c.bounding_box() for c in ch]
    hard, press = [], []
    for i, j in itertools.combinations(range(len(ch)), 2):
        a, b = bbs[i], bbs[j]
        if (a.max.X < b.min.X or b.max.X < a.min.X or a.max.Y < b.min.Y
                or b.max.Y < a.min.Y or a.max.Z < b.min.Z or b.max.Z < a.min.Z):
            continue
        r = ch[i] & ch[j]
        v = 0 if r is None else r.volume
        if v > 0.01:
            key = (ch[i].label, ch[j].label)
            (press if (key in INTENTIONAL or key[::-1] in INTENTIONAL) else hard).append((key, round(v, 2)))
    print(f"  theta {th:4}:  unexpected {len(hard)}   intended press fits {len(press)}")
    for h in hard[:10]:
        print("      !!", h)

print("=== 4. can the parts actually go together? ===")
shaft = parts["input_shaft"]
# max radius of the shaft forward of cam A -> disc-1 bearing (bore 17) must slide over it
for z in np.arange(CYC.Z_EB1 + CYC.ECC_BRG[2] + 0.2, CYC.Z_SHAFT_END, 0.5):
    sl = shaft & (Pos(0, 0, z) * Box(60, 60, 0.02))
    if sl is None or sl.volume <= 0: continue
    bb = sl.bounding_box()
    rmax = max(abs(bb.min.X), abs(bb.max.X), abs(bb.min.Y), abs(bb.max.Y))
    if rmax > 17.0/2:
        print(f"  !! shaft radius {rmax:.2f} at z={z:.1f} blocks the disc-1 bearing"); break
else:
    print("  disc-1 bearing (bore 17) slides over everything forward of cam A: OK")
# cam B slid on from the front: no clash with the shaft at any axial position
clash = []
for dz in np.arange(0.0, 14.0, 0.5):
    r = (Pos(0, 0, dz) * parts["cam_b"]) & shaft
    if r is not None and r.volume > 0.01:
        clash.append((round(dz, 1), round(r.volume, 1)))
print(f"  cam B slide-on path: {'clear' if not clash else 'BLOCKED at ' + str(clash[:4])}")
print(f"  cam B in place (dz=0) is the installed position; D-flat allows one orientation only")

print("=== 5. stack arithmetic ===")
print(f"  shoulder span needed {CYC.Z_FL0 - CYC.Z_HEAD:.2f} mm (bolts are {CYC.L_SHOULDER})")
print(f"  ring-pin end play {CYC.PIN_ENDPLAY} mm; groove ends at z={CYC.Z_GROOVE_END}")
print(f"  rotating-face gaps: ring-disc1 {CYC.Z_D1-CYC.Z_RR1:.2f}, disc1-disc2 {CYC.GAP2:.2f}, disc2-flange {CYC.Z_FL0-CYC.Z_D2-CYC.T_DISC:.2f}")
print(f"  total height {CYC.Z_CAP1} mm")
