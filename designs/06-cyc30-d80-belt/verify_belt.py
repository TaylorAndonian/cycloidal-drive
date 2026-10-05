import itertools, math
import numpy as np
import cyc30_belt as B
import cyc30_v3 as V3
from build123d import *

names = ["back_plate_belt","input_shaft_belt","motor_bracket","idler_standoff","tower_bearing_spacer"]
parts = {n: import_step(f"step_belt/{n}.step") for n in names}

print("=== interference in the belt assembly ===")
asm = B.assembly(parts)
ch = list(asm.children)
bbs = [c.bounding_box() for c in ch]
hits = []
for i, j in itertools.combinations(range(len(ch)), 2):
    a, b = bbs[i], bbs[j]
    if (a.max.X<b.min.X or b.max.X<a.min.X or a.max.Y<b.min.Y or b.max.Y<a.min.Y
            or a.max.Z<b.min.Z or b.max.Z<a.min.Z): continue
    r = ch[i] & ch[j]
    v = 0 if r is None else r.volume
    if v > 0.01: hits.append((ch[i].label, ch[j].label, round(v,1)))
print(f"  pairs overlapping: {len(hits)}")
for h in hits: print("   ", h)

print("=== shaft / tower ===")
sh = parts["input_shaft_belt"].bounding_box()
print(f"  shaft spans z {sh.min.Z:.1f} .. {sh.max.Z:.1f}; 608 seats at {B.Z_B2:.1f}..{B.Z_B2+7:.1f} and {B.Z_B1:.1f}..{B.Z_B1+7:.1f}")
print(f"  pulley seat (flat) from {B.Z_SHAFT_END:.1f} to {B.BELT_PLANE+5:.1f}; belt plane {B.BELT_PLANE}")
print(f"  journal Ø{B.JOURNAL} into 608 bore 8 (shrink allowance {B.OD_SHRINK})")

print("=== belt geometry ===")
C, pd = B.CENTRES, B.PD_MOTOR
print(f"  centres {C:.2f} mm, 20T/20T, belt {B.BELT_L:.0f} mm  (check: 2C+pi*PD = {2*C+math.pi*pd:.1f})")
print(f"  runs are tangent at y = +/-{pd/2:.2f}; idler slot centre y={B.IDLER_Y}, travel +/-7")
print(f"    idler (OD {B.IDLER_BRG[1]}) touches the belt at y = {-(pd/2 + B.IDLER_BRG[1]/2):.2f}; "
      f"slot reaches y = {B.IDLER_Y-7:.1f} .. {B.IDLER_Y+7:.1f}")
print(f"  motor slots give centre distance {C-B.SLOT_TRAVEL:.0f} .. {C+B.SLOT_TRAVEL:.0f} mm")
print(f"  belt length needed at those extremes: {2*(C-B.SLOT_TRAVEL)+math.pi*pd:.0f} .. {2*(C+B.SLOT_TRAVEL)+math.pi*pd:.0f} mm")

print("=== loads ===")
T_motor = 0.45                      # NEMA 17 holding torque, N*m
F = T_motor / (pd/2000)             # belt tension, N
a = abs(B.BELT_PLANE - B.Z_B2 - B.SUP_BRG[2]/2)      # pulley overhang past bearing 2
L = abs(B.Z_B1 + B.SUP_BRG[2]/2 - (B.Z_B2 + B.SUP_BRG[2]/2))
print(f"  belt tension at full motor torque: {F:.0f} N  (2GT 6 mm steel-core working load ~110 N)")
print(f"  bearing spacing {L:.1f} mm, pulley overhang {a:.1f} mm")
print(f"  reaction on 608 #2 (nearest pulley): {F*(1+a/L):.0f} N;  on #1: {F*a/L:.0f} N  (608 C0 = 1960 N)")
M = F*a
W = math.pi*8.0**3/32
print(f"  bending in the Ø8 journal at bearing 2: {M:.0f} N*mm -> {M/W:.1f} MPa (PA-CF interlayer ~40 MPa)")
