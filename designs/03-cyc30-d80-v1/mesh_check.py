"""
Meshing check: place the disc at its real eccentric position and measure the
gap between every ring pin and the disc flank, over a full input revolution.

Pass = min gap >= 0 (no gouging) and several pins near 0 (real contact).
"""
import math
from params import *
from cad import disc_profile

N = 1440
PROF = [(x, y) for x, y, _ in disc_profile(0.0, n=3600)]


def gaps(theta):
    """theta = input shaft angle (rad).  Returns gap per ring pin, mm."""
    cx, cy = ECC * math.cos(theta), ECC * math.sin(theta)   # cam centre
    phi = -theta / N_LOBES                                  # disc rotation
    c, s = math.cos(-phi), math.sin(-phi)
    out = []
    for k in range(N_PINS):
        a = 2 * math.pi * k / N_PINS
        px, py = R_PIN_CIRCLE * math.cos(a) - cx, R_PIN_CIRCLE * math.sin(a) - cy
        qx, qy = px * c - py * s, px * s + py * c          # into disc frame
        d = min(math.hypot(qx - x, qy - y) for x, y in PROF)
        out.append(d - R_PIN)
    return out


def main():
    worst = 1e9
    worst_th = 0.0
    contacts = []
    for i in range(120):
        th = 2 * math.pi * i / 120
        g = gaps(th)
        m = min(g)
        if m < worst:
            worst, worst_th = m, th
        contacts.append(sum(1 for v in g if v < DISC_CLEARANCE + 0.02))

    print(f"  profile points        {len(PROF)}")
    print(f"  worst gap             {worst:+.4f} mm  at theta = "
          f"{math.degrees(worst_th):.1f} deg")
    print(f"  target backlash       {DISC_CLEARANCE:+.4f} mm")
    print(f"  pins at backlash      {min(contacts)} .. {max(contacts)}"
          f"  of {N_PINS}")

    g0 = gaps(0.0)
    near = sorted(range(N_PINS), key=lambda k: g0[k])[:6]
    print("  closest pins at theta=0:")
    for k in near:
        print(f"      pin {k:2d}  gap {g0[k]:+.4f} mm")

    # a correct profile clears every pin by exactly DISC_CLEARANCE: the disc
    # is the envelope of the whole pin ring, so load is shared, not on 2 teeth.
    ok = (abs(worst - DISC_CLEARANCE) < 0.01) and min(contacts) >= N_PINS - 1
    print("\n  MESH OK" if ok else "\n  *** MESH PROBLEM ***")
    return ok


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
