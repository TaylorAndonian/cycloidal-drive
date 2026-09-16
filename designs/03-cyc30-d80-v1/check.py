"""Numeric sanity + capacity check.  Run before you print anything."""
import math
from params import *


def profile(n=4000, ecc=ECC, R=R_PIN_CIRCLE, Rr=R_PIN, N=N_PINS):
    """Standard epitrochoid-with-roller cycloidal disc profile."""
    pts = []
    for i in range(n):
        t = 2 * math.pi * i / n
        num = math.sin((1 - N) * t)
        den = (R / (ecc * N)) - math.cos((1 - N) * t)
        psi = math.atan2(num, den)
        x = R * math.cos(t) - Rr * math.cos(t + psi) - ecc * math.cos(N * t)
        y = -R * math.sin(t) + Rr * math.sin(t + psi) + ecc * math.sin(N * t)
        pts.append((x, y))
    return pts


def report():
    ok = True

    def chk(label, value, lo=None, hi=None, unit="mm"):
        nonlocal ok
        good = True
        if lo is not None and value < lo:
            good = False
        if hi is not None and value > hi:
            good = False
        ok &= good
        bound = ""
        if lo is not None and hi is not None:
            bound = f"  (want {lo}..{hi})"
        elif lo is not None:
            bound = f"  (want >= {lo})"
        elif hi is not None:
            bound = f"  (want <= {hi})"
        print(f"  [{'OK ' if good else 'BAD'}] {label:<42} {value:8.3f} {unit}{bound}")

    print("=" * 74)
    print(f"  CYCLOIDAL {N_LOBES}:1   --   {N_PINS} pins / {N_LOBES} lobes")
    print("=" * 74)

    print("\nProfile")
    chk("curtate ratio R/(e*N)", CURTATE, 1.10, 1.60, "")
    pts = profile()
    r = [math.hypot(x, y) for x, y in pts]
    r_max, r_min = max(r), min(r)
    chk("disc crest radius", r_max)
    chk("disc valley radius", r_min)
    print(f"        disc OD = {2*r_max:.2f} mm,  lobe height = {r_max-r_min:.2f} mm")
    chk("crest vs analytic (R-Rr+e)", abs(r_max - (R_PIN_CIRCLE - R_PIN + ECC)),
        hi=0.02)

    print("\nRing")
    pitch = 2 * math.pi * R_PIN_CIRCLE / N_PINS
    chk("ring pin pitch", pitch)
    chk("gap between adjacent pins", pitch - 2 * R_PIN, 1.5)
    chk("pin OD to housing OD (wall)", HOUSING_OD / 2 - (R_PIN_CIRCLE + R_PIN), 4.0)
    tie_out = TIE_PCD / 2 + TIE_D / 2
    chk("tie screw to housing OD", HOUSING_OD / 2 - tie_out, 1.5)
    chk("tie screw to pin pocket", TIE_PCD / 2 - TIE_D / 2
        - (R_PIN_CIRCLE + R_PIN + PIN_POCKET_CLR / 2), 1.0)
    chk("tie screw length vs stack",
        TIE_LEN - (RING_PLATE_T + SLEEVE_T), 2.0)

    print("\nDisc webs")
    hole_r = OUT_HOLE_D / 2
    chk("output hole Ø", OUT_HOLE_D)
    chk("web: bearing bore -> output hole", OUT_PCD_R - hole_r - DISC_BORE / 2, 3.0)
    chk("rim: output hole -> valley", r_min - (OUT_PCD_R + hole_r), 3.0)
    lig = 2 * math.pi * OUT_PCD_R / N_OUT_PINS - OUT_HOLE_D
    chk("ligament between output holes", lig, 4.0)

    print("\nShaft")
    cam_env = CAM_JOURNAL_D / 2 + ECC
    chk("cam envelope radius", cam_env)
    chk("min wall, cam journal -> motor bore",
        CAM_JOURNAL_D / 2 - ECC - SHAFT_BORE_D / 2, 2.0)
    chk("eccentric brg OD vs disc bore", ECC_BRG[2] - DISC_BORE, -0.01, 0.01)

    print("\nStack-up")
    chk("disc stack inside sleeve",
        SLEEVE_T - (N_DISCS * DISC_T + DISC_SHIM + CARRIER_T), 0.2, 1.0)
    chk("sleeve bore clearance over disc crest",
        (R_PIN_CIRCLE - r_max), 0.5)
    chk("carrier OD clearance in sleeve",
        R_PIN_CIRCLE - CARRIER_OD / 2, 0.5)
    chk("boss length vs ring plate", BOSS_LEN - RING_PLATE_T, 0.2, 1.5)
    chk("output brg OD vs housing", HOUSING_OD / 2 - OUT_BRG[2] / 2, 8.0)
    chk("boss wall over nose bearing", (BOSS_D - NOSE_BRG[2]) / 2, 5.0)
    chk("overall length", OVERALL_L, hi=52.0)
    chk("overall diameter", max(HOUSING_OD, FACE_PLATE_OD), hi=80.0)

    print("\nCapacity")
    # output pin bending, worst case: half the pins share the tangential load
    for T in (8.0, 12.0):
        F_tot = T * 1000.0 / OUT_PCD_R
        F_pin = F_tot / (N_OUT_PINS / 2)
        lever = (N_DISCS * DISC_T + DISC_SHIM) / 2
        M = F_pin * lever
        Z = math.pi * OUT_PIN_D ** 3 / 32
        print(f"        @{T:>5.1f} Nm out:  {F_pin:6.1f} N/pin,"
              f"  bending {M/Z:6.1f} MPa  (dowel ~700 MPa yield)")

    # lobe contact, scaled from the measured Ø100 unit (18 Nm @ R40, 51 pins
    # Ø3, 28 mm of disc)
    ref = dict(T=18.0, R=40.0, N=51, d=3.0, t=28.0)
    scale = ((R_PIN_CIRCLE / ref["R"]) * (N_PINS / ref["N"])
             * ((N_DISCS * DISC_T) / ref["t"]) * math.sqrt(2 * R_PIN / ref["d"]))
    print(f"        lobe-contact limited torque ~ {ref['T']*scale:.1f} Nm"
          f"  (scaled from the Ø100 unit)")

    ratio = N_LOBES
    print(f"\n        ratio {ratio}:1   ->  NEMA 17 at 0.45 Nm holding"
          f" = {0.45*ratio*0.75:.1f} Nm at the joint (75% eff.)")

    print("\n" + "=" * 74)
    print("  ALL CHECKS PASS" if ok else "  *** CHECK FAILURES ABOVE ***")
    print("=" * 74)
    return ok


if __name__ == "__main__":
    raise SystemExit(0 if report() else 1)
