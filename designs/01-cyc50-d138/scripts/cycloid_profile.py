"""
cycloid_profile.py -- cycloidal disc profile from the standard parametric equations.

Kinematics (ring fixed, N pins = Zp, disc lobes N = Zp-1, eccentricity e):
    disc centre  C(theta) = e * (cos theta, sin theta)          theta = input angle
    disc rotation phi(theta) = -theta / N                         (ratio N : 1, reversed)
In the disc frame a ring pin centre at radius R traces the epitrochoid ("hypocycloid"
disc curve in cycloidal-drive literature):
    x(s) = R cos(s)  - e cos(Zp s)
    y(s) = R sin(s)  - e sin(Zp s)          s = theta / N  in [0, 2 pi)
The lobe profile is the EQUIDISTANT (parallel) curve of that base curve, offset
INWARD by  d = Rr + c   (pin radius + clearance).  With c = 0 every pin is exactly
tangent to the profile at every input angle; c > 0 leaves a uniform normal gap c.
Undercut occurs where the base curve's convex radius of curvature is < d.
"""
import numpy as np


def base_curve(R, e, N, Zp, s):
    x = R*np.cos(s) - e*np.cos(Zp*s)
    y = R*np.sin(s) - e*np.sin(Zp*s)
    dx = -R*np.sin(s) + e*Zp*np.sin(Zp*s)
    dy = R*np.cos(s) - e*Zp*np.cos(Zp*s)
    ddx = -R*np.cos(s) + e*Zp*Zp*np.cos(Zp*s)
    ddy = -R*np.sin(s) + e*Zp*Zp*np.sin(Zp*s)
    return x, y, dx, dy, ddx, ddy


def cycloid_profile(R, Rr, e, N, Zp, clearance=0.0, pts_per_lobe=40, return_extra=False):
    """Return (x, y) of the offset lobe profile, CCW, N*pts_per_lobe points."""
    n = N * pts_per_lobe
    s = np.linspace(0, 2*np.pi, n, endpoint=False)
    x, y, dx, dy, ddx, ddy = base_curve(R, e, N, Zp, s)
    sp = np.hypot(dx, dy)
    # CCW curve: left normal (-dy, dx) points inward
    nx, ny = -dy/sp, dx/sp
    d = Rr + clearance
    px, py = x + d*nx, y + d*ny
    if not return_extra:
        return px, py
    # signed curvature of the base curve (positive = turning left = convex outward for CCW)
    kappa = (dx*ddy - dy*ddx) / sp**3
    with np.errstate(divide="ignore"):
        rho = 1.0/np.abs(kappa)
    return dict(x=px, y=py, s=s, bx=x, by=y, nx=nx, ny=ny, kappa=kappa, rho=rho, d=d)


def undercut_check(R, Rr, e, N, Zp, clearance=0.0):
    """Minimum convex radius of curvature of the base curve vs offset distance."""
    ex = cycloid_profile(R, Rr, e, N, Zp, clearance, pts_per_lobe=400, return_extra=True)
    convex = ex["kappa"] > 0
    rho_min_convex = ex["rho"][convex].min()
    return dict(rho_min_convex=float(rho_min_convex), offset=float(ex["d"]),
                ok=bool(rho_min_convex > ex["d"]), K=float(e*Zp/R))


def lobe_count(px, py):
    r = np.hypot(px, py)
    rr = np.roll(r, 1); rl = np.roll(r, -1)
    return int(np.sum((r > rr) & (r >= rl)))


if __name__ == "__main__":
    px, py = cycloid_profile(56, 2, 0.9, 50, 51, 0.1)
    print("lobes:", lobe_count(px, py), "r range", np.hypot(px, py).min(), np.hypot(px, py).max())
    print(undercut_check(56, 2, 0.9, 50, 51, 0.1))
