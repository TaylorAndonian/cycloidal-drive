"""Preview renders: front-view mesh diagram, section stack-up, 3D exploded."""
import math
import os
import struct

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from params import *
from cad import disc_profile

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

INK, ACC, ACC2, MUTE = "#1d1d1f", "#c2410c", "#0e7490", "#9ca3af"


# ------------------------------------------------------------------- front view
def front_view(ax):
    ax.add_patch(Circle((0, 0), HOUSING_OD / 2, fc="#f4f4f5", ec=INK, lw=1.4))
    ax.add_patch(Circle((0, 0), R_PIN_CIRCLE, fc="white", ec=MUTE,
                        lw=0.7, ls=(0, (6, 4))))
    for k in range(N_TIE):
        a = 2 * math.pi * k / N_TIE
        ax.add_patch(Circle((TIE_PCD / 2 * math.cos(a), TIE_PCD / 2 * math.sin(a)),
                            TIE_D / 2, fc="white", ec=MUTE, lw=0.8))
    for k in range(N_PINS):
        a = 2 * math.pi * k / N_PINS
        ax.add_patch(Circle((R_PIN_CIRCLE * math.cos(a), R_PIN_CIRCLE * math.sin(a)),
                            R_PIN, fc="#e4e4e7", ec=INK, lw=0.8, zorder=4))

    for phase, ecc_dir, col, lw in ((0.0, +1, ACC, 1.6),
                                    (180.0 / N_LOBES, -1, ACC2, 1.1)):
        pts = np.array([(x, y) for x, y, _ in disc_profile(phase, n=2000)])
        cx = ecc_dir * ECC
        ax.plot(pts[:, 0] + cx, pts[:, 1], color=col, lw=lw, zorder=5,
                alpha=1.0 if ecc_dir > 0 else 0.55)
        for k in range(N_OUT_PINS):
            a = 2 * math.pi * k / N_OUT_PINS
            ax.add_patch(Circle((OUT_PCD_R * math.cos(a) + cx,
                                 OUT_PCD_R * math.sin(a)), OUT_HOLE_D / 2,
                                fc="none", ec=col, lw=0.8,
                                alpha=1.0 if ecc_dir > 0 else 0.5, zorder=5))
        ax.add_patch(Circle((cx, 0), DISC_BORE / 2, fc="none", ec=col,
                            lw=0.9, alpha=0.8, zorder=5))

    for k in range(N_OUT_PINS):
        a = 2 * math.pi * k / N_OUT_PINS
        ax.add_patch(Circle((OUT_PCD_R * math.cos(a), OUT_PCD_R * math.sin(a)),
                            OUT_PIN_D / 2, fc="#3f3f46", ec=INK, lw=0.6, zorder=6))

    ax.set_aspect("equal")
    ax.set_xlim(-44, 44)
    ax.set_ylim(-44, 44)
    ax.axis("off")
    ax.set_title(f"{N_LOBES}:1  —  {N_PINS} pins / {N_LOBES} lobes,  "
                 f"e = {ECC} mm", fontsize=9, color=INK, pad=6)
    ax.text(0, -41.5, f"disc A (cam +e)   disc B (cam −e, {180/N_LOBES:.0f}° phase)"
                      f"   ·   Ø{HOUSING_OD:.0f} housing",
            ha="center", fontsize=7, color=MUTE)


# ----------------------------------------------------------------- section view
def section(ax):
    def band(z, t, r0, r1, col, label=None, ec=INK):
        for s in (+1, -1):
            ax.add_patch(Rectangle((z, s * r0 if s > 0 else -s * r1 * 0 - r1),
                                   t, r1 - r0, fc=col, ec=ec, lw=0.7))
        ax.add_patch(Rectangle((z, r0), t, r1 - r0, fc=col, ec=ec, lw=0.7))
        ax.add_patch(Rectangle((z, -r1), t, r1 - r0, fc=col, ec=ec, lw=0.7))
        if label:
            ax.text(z + t / 2, r1 + 1.6, label, ha="center", va="bottom",
                    fontsize=6.2, color=INK, rotation=90)

    H = HOUSING_OD / 2
    band(0, MOTOR_PLATE_T, 0, H, "#e4e4e7", "motor plate 7")
    band(Z_SLEEVE, SLEEVE_T, R_PIN_CIRCLE, H, "#e4e4e7", "sleeve 32")
    band(Z_RINGPL, RING_PLATE_T, BOSS_D / 2 + 2, H, "#e4e4e7", "ring plate 8")
    band(Z_DISC_A, DISC_T, DISC_BORE / 2, 29.7, "#fed7aa", "disc A 12.5")
    band(Z_DISC_B, DISC_T, DISC_BORE / 2, 29.7, "#bae6fd", "disc B 12.5")
    band(Z_CARRIER, CARRIER_T, 9, CARRIER_OD / 2, "#d9f99d", "carrier 6")
    band(Z_CARRIER + CARRIER_T, BOSS_LEN, 11, BOSS_D / 2, "#d9f99d")
    band(Z_FACE, FACE_PLATE_T, 5, FACE_PLATE_OD / 2, "#d9f99d", "face 4")
    # bearings
    for z, t, r0, r1, name in (
            (2, REAR_BRG[3], REAR_BRG[1] / 2, REAR_BRG[2] / 2, "6801"),
            (Z_DISC_A + 1.5, 10, 6, 10.5, "2×6801"),
            (Z_DISC_B + 1.5, 10, 6, 10.5, "2×6801"),
            (Z_RINGPL, OUT_BRG[3], OUT_BRG[1] / 2, OUT_BRG[2] / 2, "6808"),
            (Z_RINGPL + 0.5, NOSE_BRG[3], NOSE_BRG[1] / 2, NOSE_BRG[2] / 2, "608")):
        for s in (1, -1):
            ax.add_patch(Rectangle((z, s * r0 if s > 0 else -r1), t, r1 - r0,
                                   fc="#52525b", ec=INK, lw=0.5, zorder=3))
        ax.text(z + t / 2, 0.8 if name == "608" else -r1 - 0.6, name,
                ha="center", va="top", fontsize=5.5, color=ACC2, zorder=6)
    # shaft
    ax.add_patch(Rectangle((2, -6), 41.5, 12, fc="#fca5a5", ec=INK, lw=0.6,
                           zorder=2))
    ax.plot([-2, OVERALL_L + 3], [0, 0], color=MUTE, lw=0.6, ls=(0, (8, 3, 1, 3)))
    ax.annotate("", xy=(0, -H - 5), xytext=(OVERALL_L, -H - 5),
                arrowprops=dict(arrowstyle="<->", lw=0.8, color=INK))
    ax.text(OVERALL_L / 2, -H - 7.5, f"{OVERALL_L:.1f} mm", ha="center",
            va="top", fontsize=7.5, color=INK)
    ax.set_aspect("equal")
    ax.set_xlim(-4, OVERALL_L + 5)
    ax.set_ylim(-H - 14, H + 13)
    ax.axis("off")
    ax.set_title("section — Ø80 × 51.5", fontsize=9, color=INK, pad=6)


# ------------------------------------------------------------------ 3d exploded
def read_stl(path):
    with open(path, "rb") as f:
        head = f.read(84)
        n = struct.unpack("<I", head[80:84])[0]
        data = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
    tri = data[:, 12:48].copy().view("<f4").reshape(n, 3, 3)
    return tri.astype(float)


def exploded(ax):
    gap = 34.0
    items = [("motor_plate", 0.0, "#d4d4d8"), ("sleeve", 1.0, "#e4e4e7"),
             ("disc_a", 2.15, "#fdba74"), ("disc_b", 3.0, "#7dd3fc"),
             ("carrier", 3.85, "#bef264"), ("ring_plate", 4.7, "#e4e4e7"),
             ("face_plate", 5.55, "#bef264"), ("shaft", 6.6, "#fca5a5")]
    zoff = {"motor_plate": 0, "sleeve": Z_SLEEVE, "disc_a": Z_DISC_A,
            "disc_b": Z_DISC_B, "carrier": Z_CARRIER, "ring_plate": Z_RINGPL,
            "face_plate": Z_FACE, "shaft": 0}
    for name, i, col in items:
        p = os.path.join(OUT, "stl", f"{name}.stl")
        if not os.path.exists(p):
            continue
        tri = read_stl(p)
        tri[:, :, 2] += zoff[name] + i * gap
        if name == "disc_a":
            tri[:, :, 0] += ECC
        if name == "disc_b":
            tri[:, :, 0] -= ECC
        ax.add_collection3d(Poly3DCollection(
            tri, facecolors=col, alpha=1.0, shade=True, zsort="min",
            lightsource=matplotlib.colors.LightSource(azdeg=235, altdeg=50)))
        ax.text(48, 0, tri[:, :, 2].mean(), name.replace("_", " "),
                fontsize=6.0, color=INK, ha="left", va="center", zorder=99)
    ax.set_xlim(-45, 45)
    ax.set_ylim(-45, 45)
    ax.set_zlim(-5, 300)
    ax.set_box_aspect((1, 1, 2.6))
    ax.view_init(elev=12, azim=-68)
    ax.axis("off")
    ax.set_title("exploded", fontsize=9, color=INK, pad=0)


def main():
    fig = plt.figure(figsize=(14.5, 6.4), dpi=170)
    fig.patch.set_facecolor("white")
    gs = fig.add_gridspec(1, 3, width_ratios=[1.05, 1.05, 0.9])
    front_view(fig.add_subplot(gs[0]))
    section(fig.add_subplot(gs[1]))
    exploded(fig.add_subplot(gs[2], projection="3d"))
    fig.tight_layout(w_pad=1.0)
    out = os.path.join(OUT, "preview.png")
    fig.savefig(out, facecolor="white", bbox_inches="tight")
    print("wrote", out)


if __name__ == "__main__":
    main()
