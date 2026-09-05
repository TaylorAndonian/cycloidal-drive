#!/usr/bin/env python3
"""
analysis.py -- numerical design analysis of the 50:1 cycloidal gearbox.

Everything is computed from the parameters in cad.py (single source of truth):
  1. Lobe profile from the epitrochoid + equidistant offset; undercut check; K sweep
  2. Mesh geometry over the cycle: contact normal, lever arm, pressure angle, engagement depth
  3. Compliant load-sharing model (pins + output rollers + eccentric bearing) -> per-pin forces
  4. Hertz contact stress on the printed lobes
  5. Radial-disengagement ("skip") torque estimate with a loss-of-engagement cascade
  6. Ring/plate ovalisation (Fourier ring solution) and metal-band comparison
  7. Bearing loads, output pin loads, shaft bending, efficiency estimate
  8. Two discs @180 vs three discs @120
Outputs: ../analysis/*.png, ../analysis/results.json, ../analysis/report.md
"""
import os, sys, json, math
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cad import P, layout
from cycloid_profile import cycloid_profile, undercut_check, lobe_count, base_curve

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "analysis")
os.makedirs(OUT, exist_ok=True)
L = layout(P)

# ----------------------------------------------------------------- materials --
# Printed, XY direction, approximate datasheet values (verify against the spool you use)
MAT = {
    "PA-CF": dict(E=4000.0, nu=0.40, sig_c=90.0,  note="Bambu PAHT-CF / PA6-CF class"),
    "PC":    dict(E=2300.0, nu=0.38, sig_c=70.0,  note="Bambu PC"),
    "PLA":   dict(E=3000.0, nu=0.36, sig_c=60.0,  note="creeps, softens ~55C: not for housing"),
    "PETG":  dict(E=1900.0, nu=0.38, sig_c=50.0,  note="too compliant for structure"),
}
E_STEEL, NU_STEEL = 200e3, 0.30

T_NOMINAL = 18.0      # Nm, expected at output (0.45 Nm * 50 * ~0.8)
T_DESIGN  = 36.0      # Nm, 2x margin

R, Rr, e, N, Zp = P["R_PIN"], P["PIN_D"]/2, P["ECC"], P["N_LOBES"], P["N_PINS"]
c_prof = P["PROFILE_CLR"]
d_off = Rr + c_prof
DISC_T = P["DISC_T"]
r_o, r_roll = P["R_OUT"], P["ROLLER"]["od"]/2
c_hole = P["HOLE_CLR"]/2          # radial slop of roller in hole

results = {}

# =============================================================================
# 1. Profile
# =============================================================================
px, py = cycloid_profile(R, Rr, e, N, Zp, c_prof, pts_per_lobe=100)
rad = np.hypot(px, py)
uc = undercut_check(R, Rr, e, N, Zp, c_prof)
results["profile"] = dict(lobes=lobe_count(px, py), r_tip=float(rad.max()), r_root=float(rad.min()),
                          lobe_height=float(rad.max() - rad.min()), K=uc["K"], undercut=uc,
                          pitch_radius_disc=N*e, pitch_radius_ring=Zp*e)
print("profile:", json.dumps(results["profile"], indent=1))
assert results["profile"]["lobes"] == N
assert uc["ok"], "UNDERCUT: reduce ECC or PIN_D"

# K sweep: undercut margin and lobe height
ks = np.linspace(0.5, 0.98, 25)
ksweep = []
for K in ks:
    ee = K*R/Zp
    u_ = undercut_check(R, Rr, ee, N, Zp, c_prof)
    ksweep.append(dict(K=float(K), e=float(ee), rho_min=u_["rho_min_convex"], ok=u_["ok"], lobe_h=2*ee))
results["K_sweep"] = ksweep

# =============================================================================
# 2. Mesh geometry at input angle theta   (disc frame, disc centre at origin)
# =============================================================================
def mesh_geometry(theta):
    """Per-pin geometry at input angle theta. Returns dict of arrays (length Zp)."""
    beta = 2*np.pi*np.arange(Zp)/Zp
    s = beta + theta/N                       # base-curve parameter of pin j (see profile.py)
    x, y, dx, dy, ddx, ddy = base_curve(R, e, N, Zp, s)
    sp = np.hypot(dx, dy)
    nx, ny = -dy/sp, dx/sp                   # inward normal = direction of pin force ON the disc
    q = np.stack([x, y], 1)                  # pin centres in disc frame
    n = np.stack([nx, ny], 1)
    k = q + d_off*n                          # contact points (on the disc surface)
    l = k[:, 0]*n[:, 1] - k[:, 1]*n[:, 0]    # lever arm about disc centre (z of k x n)
    rho_q = np.hypot(q[:, 0], q[:, 1])
    rhat = q / rho_q[:, None]
    that = np.stack([-rhat[:, 1], rhat[:, 0]], 1)
    cos_alpha = np.abs(np.sum(n*that, 1))
    alpha = np.degrees(np.arccos(np.clip(cos_alpha, 0, 1)))       # pressure angle
    h = rad.max() - (rho_q - Rr)             # radial engagement depth of pin in the lobe envelope
    kappa = (dx*ddy - dy*ddx)/sp**3
    rho_b = 1/np.abs(kappa)
    rho_prof = np.where(kappa > 0, rho_b - d_off, rho_b + d_off)   # profile curvature radius at contact
    convex = kappa > 0
    # eccentric direction in the disc frame
    th_d = theta*(1 + 1/N)
    u = np.array([np.cos(th_d), np.sin(th_d)])
    t = np.array([-u[1], u[0]])
    return dict(q=q, n=n, k=k, l=l, alpha=alpha, h=h, rho_prof=rho_prof, convex=convex,
                u=u, t=t, rhat=rhat, s=s)

def roller_geometry(theta):
    """Output rollers in the disc frame.  Roller centres O_i relative to disc centre C."""
    th_d = theta*(1 + 1/N)
    u = np.array([np.cos(th_d), np.sin(th_d)])
    ang = 2*np.pi*np.arange(P["N_OUT"])/P["N_OUT"]
    hole = r_o*np.stack([np.cos(ang), np.sin(ang)], 1)           # hole centres (disc frame)
    O = hole - e*u                                               # roller centres = hole centre - e u
    mu = O[:, 0]*u[1] - O[:, 1]*u[0]                             # z of (O x u)
    return dict(O=O, mu=mu, u=u)

# =============================================================================
# 3. Stiffness model
# =============================================================================
def hertz_line_stiffness(E_p, nu_p, Lc):
    """Approach stiffness of a steel cylinder on a plastic body, line contact (N/mm).
    delta ~ F/(pi Lc) * ((1-nu^2)/E)*(2/3 + ln(...)) -> use ~2.5 as the bracket (Johnson)."""
    comp = (1 - nu_p**2)/E_p + (1 - NU_STEEL**2)/E_STEEL
    return math.pi*Lc/(2.5*comp)

CANTILEVER_FREE_LEN = None      # set to a length (mm) to model pins supported at ONE end only
THIN_RING = None                # set to (t_radial, h_axial) to model a thin printed ring instead of the plates

MID_RING_ON = None      # None -> use P["MID_RING"]; True/False to force, for the comparison table

def mid_ring_stiffness():
    """Radial stiffness (N/mm) the floating mid ring offers to ONE pin.
    The ring is loaded by all 51 pins at once; its n=2 ovalisation dominates, so the effective
    per-pin support is taken from the same thin-ring solution used for the plates."""
    t = P["MID_RING_OR"] - P["MID_RING_IR"]; h = P["MID_RING_T"]
    Rm = (P["MID_RING_OR"] + P["MID_RING_IR"])/2
    I = h*t**3/12; E = MAT["PA-CF"]["E"]
    # unit radial load on one pin -> w = F R^3 / (E I) * k2, k2 ~ 1/(3 pi) for a self-equilibrated pair
    return 3*math.pi*E*I/Rm**3

def pin_bending_stiffness(d, Lspan, x):
    """Steel ring pin. Default: simply supported in the two plates, load at x from the rear support.
    With the mid ring: an extra elastic support at mid-span (two half-spans in parallel through it).
    With CANTILEVER_FREE_LEN set: cantilever, load at the free end (what a one-plate printed ring does)."""
    I = math.pi*d**4/64
    if CANTILEVER_FREE_LEN:
        return 3*E_STEEL*I/CANTILEVER_FREE_LEN**3
    k_ss = 3*E_STEEL*I*Lspan/(x**2*(Lspan - x)**2)
    use_mid = P["MID_RING"] if MID_RING_ON is None else MID_RING_ON
    if not use_mid:
        return k_ss
    # load sits in one half-span: that half behaves as a beam of length Lh between the plate and the
    # ring, and the ring itself is elastic -> series, then in parallel with the un-shortened beam
    Lh = Lspan/2
    xh = min(max(x, 1.0), Lh - 1.0) if x < Lh else min(max(Lspan - x, 1.0), Lh - 1.0)
    k_half = 3*E_STEEL*I*Lh/(xh**2*(Lh - xh)**2)
    k_ring = mid_ring_stiffness()
    return k_ss + 1/(1/k_half + 1/k_ring)

def seat_stiffness(E_p, d, Lseat):
    """Pin end bearing on the printed hole wall, both ends in series -> per contact ~ E*L."""
    return E_p*Lseat*1.0

def ball_bearing_stiffness(F, nballs, Dball):
    """Secant radial stiffness of a deep-groove ball bearing (Harris approx), N/mm."""
    F = max(F, 20.0)
    Q = 5*F/nballs
    delta = 4.36e-4*Q**(2/3)/Dball**(1/3)
    return F/delta

def per_pin_stiffness(E_p, nu_p, x_disc, Lspan, Fguess=50.0):
    kc = hertz_line_stiffness(E_p, nu_p, DISC_T)
    kb = pin_bending_stiffness(P["PIN_D"], Lspan, x_disc)
    ks_rear = seat_stiffness(MAT["PA-CF"]["E"], P["PIN_D"], P["PIN_SEAT_REAR"])
    ks_front = seat_stiffness(MAT["PA-CF"]["E"], P["PIN_D"], L["ringpin_front_engage"])
    k = 1/(1/kc + 1/kb + 1/ks_rear + 1/ks_front)
    return k, dict(k_hertz=kc, k_bend=kb, k_seat_rear=ks_rear, k_seat_front=ks_front, k_total=k,
                   k_mid_ring=(mid_ring_stiffness() if P["MID_RING"] else None))

def roller_stiffness(Fguess=300.0):
    k608 = ball_bearing_stiffness(Fguess, 7, 3.97)
    kpin = pin_bending_stiffness(P["OUT_PIN_D"], L["z_flange_rear"] - L["z_carrier_front"] + 4, 8.0)
    kseat = seat_stiffness(MAT["PA-CF"]["E"], P["OUT_PIN_D"], P["CARRIER_T"])
    khole = hertz_line_stiffness(MAT["PA-CF"]["E"], 0.4, P["ROLLER"]["w"])*3   # conformal -> stiffer
    return 1/(1/k608 + 1/kpin + 1/kseat + 1/khole)

def ecc_bearing_stiffness(Fguess=400.0):
    return ball_bearing_stiffness(Fguess, 9, 3.5)      # 6902: ~9 balls of ~3.5 mm (verify)

# =============================================================================
# 4. Load-sharing solve (active-set linear complementarity)
# =============================================================================
_PROF = None
def _profile_pts():
    global _PROF
    if _PROF is None or _PROF[2] != e:
        px_, py_ = cycloid_profile(R, Rr, e, N, Zp, c_prof, pts_per_lobe=120)
        _PROF = (px_, py_, e)
    return _PROF[0], _PROF[1]

def geometric_gap(x, q, sel, return_depth=False):
    """Exact normal gap (mm) between each selected pin and the disc profile after the rigid
    disc motion x = (dphi, dCu, dCt) expressed in disc-frame displacement dC (passed in q_dc)."""
    px_, py_ = _profile_pts()
    dphi, dC = x[0], x[1]
    c_, s_ = np.cos(dphi), np.sin(dphi)
    qx = c_*px_ - s_*py_ + dC[0]; qy = s_*px_ + c_*py_ + dC[1]
    gap = np.full(len(q), np.inf); depth = np.full(len(q), np.nan)
    rtip = np.hypot(px_, py_).max()
    for j in np.where(sel)[0]:
        dd = np.hypot(qx - q[j, 0], qy - q[j, 1]); i = int(np.argmin(dd))
        gap[j] = dd[i] - Rr
        # radial depth of the contact point below the lobe tip, measured from the DISPLACED disc centre
        depth[j] = rtip - np.hypot(qx[i] - dC[0], qy[i] - dC[1])
    if return_depth:
        return gap, depth
    return gap

def solve_load(theta, T_disc, E_p, nu_p, x_disc, Lspan, k_e=None, c_pin=c_prof,
               lost=None, g_ecc=0.0, newton=3):
    """Linear-complementarity solve, then Newton refinement with the exact geometric gaps."""
    sol = _solve_load_lin(theta, T_disc, E_p, nu_p, x_disc, Lspan, k_e, c_pin, lost, g_ecc)
    if sol is None:
        return None
    g = sol["g"]
    for it in range(newton):
        x = sol["x"]; dC = x[1]*g["u"] + x[2]*g["t"]
        sel = (g["l"] < 0) & ~(np.zeros(Zp, bool) if lost is None else lost)
        gap = geometric_gap((x[0], dC), g["q"], sel)
        A = np.stack([-g["l"], -(g["n"] @ g["u"]), -(g["n"] @ g["t"])], 1)
        # closure(x_k + dx) = -gap(x_k) + A dx  ==  A (x_k+dx) - c_eff   ->  c_eff = gap + A x_k
        c_eff = np.where(np.isfinite(gap), gap + A @ x, 1e9)
        sol2 = _solve_load_lin(theta, T_disc, E_p, nu_p, x_disc, Lspan, k_e, c_eff, lost, g_ecc, x0=x)
        if sol2 is None:
            break
        sol = sol2
    return sol

def _solve_load_lin(theta, T_disc, E_p, nu_p, x_disc, Lspan, k_e=None, c_pin=c_prof,
               lost=None, g_ecc=0.0, x0=None):
    """Quasi-static equilibrium of one disc carrying torque T_disc (Nm) at input angle theta.
    Unknowns: dphi (rad), dCu, dCt (mm, disc centre displacement along u and t), Fe (N).
    Returns per-pin forces F_j (N), roller forces F_i, displacements."""
    g = mesh_geometry(theta); rg = roller_geometry(theta)
    u, t = g["u"], g["t"]
    n, l, mu = g["n"], g["l"], rg["mu"]
    T = T_disc*1000.0                                   # N mm
    kj, _ = per_pin_stiffness(E_p, nu_p, x_disc, Lspan)
    kr = roller_stiffness()
    if k_e is None:
        k_e = ecc_bearing_stiffness()
    nu_ = n @ u; nt_ = n @ t
    cvec = np.broadcast_to(np.asarray(c_pin, float), (Zp,))
    # closure_j = a_j . x - c_j,  x = (dphi, dCu, dCt)
    A = np.stack([-l, -nu_, -nt_], 1)
    B = np.stack([mu, np.ones(len(mu)), np.zeros(len(mu))], 1)    # roller closure = mu dphi + dCu - c_hole
    act_p = np.zeros(Zp, bool); act_r = np.zeros(len(mu), bool)
    lost = np.zeros(Zp, bool) if lost is None else lost
    x = np.zeros(3) if x0 is None else np.array(x0, float)
    if x0 is not None:
        act_p = (A @ x - cvec > 0) & ~lost; act_r = (B @ x - c_hole) > 0
    for it in range(60):
        # linear system in (x, Fe):  F_j = kj (A_j x - c),  F_i = kr (B_i x - c_hole)
        Ap = A[act_p]; Br = B[act_r]
        # R1: sum F_j nt + Fe = 0 ; R2: sum F_j nu - sum F_i - k_e dCu = 0 ; R3: sum F_j l - sum F_i mu = 0 ; R4: -sum F_i mu = T
        M = np.zeros((4, 4)); rhs = np.zeros(4)
        ntp, nup, lp, mup = nt_[act_p], nu_[act_p], l[act_p], mu[act_r]
        cp = cvec[act_p]
        M[0, :3] = kj*(ntp @ Ap); M[0, 3] = 1;          rhs[0] = kj*np.sum(ntp*cp)
        M[1, :3] = kj*(nup @ Ap) - kr*(np.ones(act_r.sum()) @ Br); M[1, 1] -= k_e
        rhs[1] = kj*np.sum(nup*cp) - kr*act_r.sum()*c_hole
        M[2, :3] = kj*(lp @ Ap) - kr*(mup @ Br);         rhs[2] = kj*np.sum(lp*cp) - kr*np.sum(mup)*c_hole
        M[3, :3] = -kr*(mup @ Br);                       rhs[3] = T - kr*np.sum(mup)*c_hole
        try:
            sol = np.linalg.lstsq(M, rhs, rcond=None)[0]
        except np.linalg.LinAlgError:
            return None
        x = sol[:3]; Fe = sol[3]
        clos_p = A @ x - cvec; clos_r = B @ x - c_hole
        new_p = (clos_p > 0) & ~lost; new_r = clos_r > 0
        if np.array_equal(new_p, act_p) and np.array_equal(new_r, act_r) and it > 0:
            break
        act_p, act_r = new_p, new_r
        if act_p.sum() == 0 or act_r.sum() == 0:
            # bootstrap: activate the best pins/rollers
            if act_p.sum() == 0:
                act_p = (l < 0) & ~lost
            if act_r.sum() == 0:
                act_r = mu < 0
    Fj = np.where(act_p, kj*np.clip(A @ x - cvec, 0, None), 0.0)
    Fi = np.where(act_r, kr*np.clip(B @ x - c_hole, 0, None), 0.0)
    T_check = -np.sum(Fi*mu)
    # loss of engagement per pin (radial, mm)
    kc = hertz_line_stiffness(E_p, nu_p, DISC_T)
    kb = pin_bending_stiffness(P["PIN_D"], Lspan, x_disc)
    ks = 1/(1/seat_stiffness(MAT["PA-CF"]["E"], P["PIN_D"], P["PIN_SEAT_REAR"]) +
            1/seat_stiffness(MAT["PA-CF"]["E"], P["PIN_D"], L["ringpin_front_engage"]))
    k_pin_disp = 1/(1/kb + 1/ks)                  # pin body displacement (excl. local Hertz)
    outward = -(np.sum(n*g["rhat"], 1))           # >0 : pin force has an inward component on the disc
    pin_out = Fj/k_pin_disp*outward               # pin moves outward by this
    dC = x[1]*u + x[2]*t
    disc_away = -(g["rhat"] @ dC)                 # disc moves away from pin
    net_pin_force = Fj @ n - np.sum(Fi)*u         # force on disc from pins + rollers
    Fbear = -(net_pin_force + Fe*t)               # plus bearing tangential drive... total bearing reaction
    return dict(F=Fj, Fr=Fi, x=x, Fe=Fe, T_check=T_check/1000, act=act_p, g=g, rg=rg,
                pin_out=pin_out, disc_away=disc_away, k_pin=kj, k_e=k_e,
                F_bearing=np.hypot(*( -net_pin_force)), F_bearing_vec=-net_pin_force,
                net_roller=np.sum(Fi), dC=dC)

def hertz_pmax(F, E_p, nu_p, rho_prof, convex):
    Es = 1/((1 - nu_p**2)/E_p + (1 - NU_STEEL**2)/E_STEEL)
    Reff = np.where(convex, 1/(1/Rr + 1/np.abs(rho_prof)), 1/np.abs(1/Rr - 1/np.abs(rho_prof)))
    return np.sqrt(np.clip(F, 0, None)*Es/(math.pi*DISC_T*Reff))

# =============================================================================
# 5. Ring / plate ovalisation (Fourier thin-ring solution)
# =============================================================================
def ring_deflection(load_angles, load_radial, Rm, E_p, I, A, nharm=30):
    """Radial displacement w(phi) of a thin ring under radial point loads (N) at angles.
    w_n = q_n R^4/(E I (n^2-1)^2) for n>=2 (bending), w_0 = q_0 R^2/(E A) (hoop).
    Returns w at the load angles (mm, outward positive)."""
    phis = np.linspace(0, 2*np.pi, 720, endpoint=False)
    q = np.zeros_like(phis)
    dphi = phis[1] - phis[0]
    for a, F in zip(load_angles, load_radial):
        i = int(round((a % (2*np.pi))/dphi)) % len(phis)
        q[i] += F/(Rm*dphi)                       # N/mm along circumference
    Q = np.fft.rfft(q)/len(phis)
    W = np.zeros_like(Q)
    W[0] = Q[0]*Rm**2/(E_p*A)
    for nn in range(2, min(nharm, len(Q))):
        W[nn] = Q[nn]*Rm**4/(E_p*I*(nn**2 - 1)**2)
    w = np.fft.irfft(W*len(phis), n=len(phis))
    return np.interp(np.mod(load_angles, 2*np.pi), phis, w), phis, w

def plate_props(which, E_p):
    if THIN_RING:
        t, h = THIN_RING
        r_in = P["R_PIN"] + P["PIN_D"]/2 + 1.0; r_out = r_in + t
        return dict(Rm=(r_in + r_out)/2, I=h*t**3/12, A=h*t, E=E_p, t=t, h=h)
    if which == "front":
        r_in, r_out, h = P["BRG_OUTPUT"]["od"]/2, P["CUP_OD"]/2, L["front_plate_T"]
    else:
        r_in, r_out, h = P["BRG_CARRIER"]["od"]/2, P["CUP_OD"]/2, P["FLOOR_T"]
    t = r_out - r_in
    return dict(Rm=(r_in + r_out)/2, I=h*t**3/12, A=h*t, E=E_p, t=t, h=h)

# =============================================================================
# 6. Full evaluation at a torque
# =============================================================================
def geom_for(n_discs):
    extra = (n_discs - 2)*(DISC_T + P["DISC_DISC_GAP"])
    Lspan = L["ringpin_free_span"] + extra
    x_disc = (L["z_discA"][0] + L["z_discA"][1])/2 - P["FLOOR_T"]
    if n_discs == 3:
        x_disc = Lspan/2               # middle disc = worst pin bending
    return Lspan, x_disc

def compute_loss(sol, n_discs, Epl, x_disc, Lspan, g_ecc, g_pin):
    """Radial loss of engagement per pin = pin bending/seat + disc moving away + plate ovalisation + clearances."""
    g = sol["g"]; F = sol["F"]
    pin_ang = np.arctan2(g["q"][:, 1], g["q"][:, 0])
    radial_load = F*(-(np.sum(g["n"]*g["rhat"], 1)))        # outward force on pins
    angs, loads = [pin_ang], [radial_load]
    for kdisc in range(1, n_discs):                          # other discs load the opposite side(s)
        angs.append(pin_ang + 2*np.pi*kdisc/n_discs); loads.append(radial_load)
    angs = np.concatenate(angs); loads = np.concatenate(loads)
    w_pl = {}
    for which in ("front", "rear"):
        pp = plate_props(which, Epl)
        wl, _, _ = ring_deflection(angs, loads/2, pp["Rm"], pp["E"], pp["I"], pp["A"])
        w_pl[which] = wl[:Zp]
    wA = (1 - x_disc/Lspan)*w_pl["rear"] + (x_disc/Lspan)*w_pl["front"]
    # elastic + clearance radial separation between pin and disc (the rigid-body disc motion is
    # already inside the geometric contact depth used for the margin)
    loss = sol["pin_out"] + wA + (g_ecc + g_pin)*(F > 0)
    return loss, w_pl

def contact_depth(sol):
    """Radial depth of each loaded pin's contact point below the lobe tip, for the displaced disc."""
    g = sol["g"]; x = sol["x"]; dC = sol["dC"]
    _, depth = geometric_gap((x[0], dC), g["q"], sol["F"] > 0, return_depth=True)
    return depth

def evaluate(T_out, n_discs=2, mat_disc="PA-CF", mat_plate="PA-CF", theta=None, verbose=False,
             g_ecc=0.02, g_pin=0.0):
    """Evaluate one disc of an n-disc drive at total output torque T_out (Nm).
    Returns key numbers incl. per-pin loss of engagement and margin to skip."""
    Ep, nup = MAT[mat_disc]["E"], MAT[mat_disc]["nu"]
    Epl = MAT[mat_plate]["E"]
    T_disc = T_out/n_discs
    Lspan, x_disc = geom_for(n_discs)
    # the mesh pattern repeats every pin pitch 2*pi/Zp (up to relabelling), so sweep that
    thetas = np.linspace(0, 2*np.pi/Zp, 8, endpoint=False) if theta is None else [theta]
    worst = None
    for th in thetas:
        sol = solve_load(th, T_disc, Ep, nup, x_disc, Lspan, g_ecc=g_ecc)
        if sol is None:
            continue
        g = sol["g"]; F = sol["F"]
        loss, w_pl = compute_loss(sol, n_discs, Epl, x_disc, Lspan, g_ecc, g_pin)
        depth = contact_depth(sol)
        # margin to skip is judged on pins that actually carry load (>= 20 % of the max pin force);
        # a barely-touching shallow pin walking out does not cause a skip
        carrying = F >= 0.2*F.max()
        margin = np.where(carrying, depth - loss, np.inf)
        pmax = hertz_pmax(F, Ep, nup, g["rho_prof"], g["convex"])
        rec = dict(theta=th, sol=sol, loss=loss, margin=margin, pmax=pmax, w_plate=w_pl, depth=depth,
                   Fmax=F.max(), n_loaded=int((F > 0).sum()), alpha_loaded=g["alpha"][F > 0])
        if worst is None or margin.min() < worst["margin"].min():
            worst = rec
    return worst

def skip_torque(n_discs=2, mat_disc="PA-CF", mat_plate="PA-CF", g_ecc=0.02, g_pin=0.0, Tmax=400):
    """Increase torque until the loss-of-engagement cascade removes every loaded pin."""
    Ts = np.linspace(5, Tmax, 40)
    Ep, nup = MAT[mat_disc]["E"], MAT[mat_disc]["nu"]
    T_first = None
    for T in Ts:
        rec = evaluate(T, n_discs, mat_disc, mat_plate, theta=None, g_ecc=g_ecc, g_pin=g_pin)
        if rec is None:
            return T, T_first
        if rec["margin"].min() < 0 and T_first is None:
            T_first = float(T)
        # cascade at the worst theta: pins that have walked out are removed and the rest re-solved
        th = rec["theta"]
        Lspan, x_disc = geom_for(n_discs)
        Epl = MAT[mat_plate]["E"]
        lost = np.zeros(Zp, bool)
        for _ in range(Zp):
            sol = solve_load(th, T/n_discs, Ep, nup, x_disc, Lspan, lost=lost, g_ecc=g_ecc)
            if sol is None or (sol["F"] > 0).sum() == 0:
                return float(T), T_first
            g = sol["g"]; F = sol["F"]
            loss, _ = compute_loss(sol, n_discs, Epl, x_disc, Lspan, g_ecc, g_pin)
            depth = contact_depth(sol)
            bad = (F >= 0.2*F.max()) & (loss >= depth)
            if not bad.any():
                break
            lost |= bad
            if (lost | (g["l"] >= 0)).all():
                return float(T), T_first
    return float(Tmax), T_first

# =============================================================================
# run
# =============================================================================

RES_PATH = os.path.join(OUT, "results.json")
def load_results():
    if os.path.exists(RES_PATH):
        with open(RES_PATH) as f:
            return json.load(f)
    return {}
def save_results(r):
    with open(RES_PATH, "w") as f:
        json.dump(r, f, indent=1, default=lambda o: float(o) if isinstance(o, (np.floating, np.integer)) else (o.tolist() if isinstance(o, np.ndarray) else str(o)))

def x_discA():
    return (L["z_discA"][0] + L["z_discA"][1])/2 - P["FLOOR_T"]

if __name__ == "__main__":
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    stage = sys.argv[1] if len(sys.argv) > 1 else "all"
    R_ = load_results(); R_.update(results)

    if stage in ("1", "all"):
        # ---- geometry: pressure angle & engagement over the cycle
        g0 = mesh_geometry(0.0)
        R_["mesh_at_theta0"] = dict(alpha_min=float(g0["alpha"].min()), alpha_max=float(g0["alpha"].max()),
                                    lever_max=float(np.abs(g0["l"]).max()), h_max=float(g0["h"].max()))
        thetas = np.linspace(0, 2*np.pi/Zp, 60)
        a_max_lever, a_wmean = [], []
        for th in thetas:
            g = mesh_geometry(th); j = np.argmin(g["l"]); a_max_lever.append(g["alpha"][j])
            sel = g["l"] < 0; a_wmean.append(np.average(g["alpha"][sel], weights=-g["l"][sel]))
        R_["pressure_angle"] = dict(at_max_lever_pin_min=float(np.min(a_max_lever)), at_max_lever_pin_max=float(np.max(a_max_lever)),
                                    lever_weighted_mean=float(np.mean(a_wmean)))
        # ---- load sharing at nominal and design torque
        for label, T in (("nominal", T_NOMINAL), ("design", T_DESIGN)):
            rec = evaluate(T, 2); sol = rec["sol"]; g = sol["g"]
            F = sol["F"]
            R_[f"load_{label}"] = dict(
                T_out=T, T_per_disc=T/2, T_check=float(sol["T_check"]), theta_worst_deg=float(np.degrees(rec["theta"])),
                n_pins_loaded=rec["n_loaded"], F_pin_max=float(rec["Fmax"]), F_pin_sum=float(F.sum()),
                pmax_hertz_MPa=float(rec["pmax"].max()), F_ecc_bearing=float(sol["F_bearing"]), F_e_tangential=float(sol["Fe"]),
                F_roller_max=float(sol["Fr"].max()), F_roller_sum=float(sol["net_roller"]),
                windup_deg=float(np.degrees(sol["x"][0])), dC_u_mm=float(sol["x"][1]), dC_t_mm=float(sol["x"][2]),
                loss_max_mm=float(rec["loss"][F > 0].max()), margin_min_mm=float(rec["margin"].min()),
                contact_depth_min_mm=float(np.nanmin(rec["depth"])), contact_depth_max_mm=float(np.nanmax(rec["depth"])),
                plate_w_front_max=float(rec["w_plate"]["front"].max()), plate_w_rear_max=float(rec["w_plate"]["rear"].max()),
                pin_outward_max=float(sol["pin_out"].max()), disc_away_max=float(sol["disc_away"][F > 0].max()),
                per_pin_stiffness=per_pin_stiffness(MAT["PA-CF"]["E"], 0.4, x_discA(), L["ringpin_free_span"])[1],
                k_ecc=float(sol["k_e"]), k_roller=float(roller_stiffness()),
                alpha_loaded_min=float(rec["alpha_loaded"].min()), alpha_loaded_max=float(rec["alpha_loaded"].max()))
            print(label, json.dumps(R_[f"load_{label}"], indent=1, default=float))
        alpha_eff = math.radians(R_["pressure_angle"]["lever_weighted_mean"])
        F_t = T_DESIGN*1000/2/(N*e)
        R_["hand_estimate"] = dict(alpha_eff_deg=math.degrees(alpha_eff), F_t_per_disc_N=F_t, F_r_per_disc_N=F_t*math.tan(alpha_eff),
                                   F_bearing_est_N=F_t/math.cos(alpha_eff))
        # plots
        px_, py_ = cycloid_profile(R, Rr, e, N, Zp, c_prof, pts_per_lobe=100)
        fig, ax = plt.subplots(1, 2, figsize=(14, 7))
        g = mesh_geometry(0.0); rg = roller_geometry(0.0)
        for a in ax:
            a.plot(px_, py_, "b-", lw=0.8)
            for j in range(Zp):
                a.add_patch(plt.Circle(g["q"][j], Rr, color="k", fill=False, lw=0.8))
            a.add_patch(plt.Circle((0, 0), P["DISC_BRG"]["od"]/2, color="g", fill=False))
            for i in range(P["N_OUT"]):
                an = 2*np.pi*i/P["N_OUT"]
                a.add_patch(plt.Circle((r_o*np.cos(an), r_o*np.sin(an)), L["disc_hole_d"]/2, color="g", fill=False))
                a.add_patch(plt.Circle(rg["O"][i], r_roll, color="r", fill=False, ls="--"))
            a.set_aspect("equal")
        ax[0].set_title(f"Disc frame, theta=0: {N} lobes, {Zp} pins Ø{P['PIN_D']}, R={R}, e={e}, K={R_['profile']['K']:.2f}, clr={c_prof}\n"
                        "black: ring pins, green: bore + roller holes, red dashed: 608 rollers")
        ax[0].set_xlim(-62, 62); ax[0].set_ylim(-62, 62)
        ax[1].set_xlim(30, 62); ax[1].set_ylim(-16, 16); ax[1].set_title("zoom: deepest-engagement side")
        ax[1].plot(px_, py_, "b.", ms=2)
        fig.tight_layout(); fig.savefig(os.path.join(OUT, "profile.png"), dpi=130); plt.close(fig)

        g = mesh_geometry(0.0); idx = np.arange(Zp)
        fig, ax = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
        ax[0].plot(idx, g["alpha"], "o-"); ax[0].set_ylabel("pressure angle [deg]"); ax[0].grid()
        ax[1].plot(idx, g["l"], "o-"); ax[1].axhline(0, color="k"); ax[1].set_ylabel("lever arm about disc centre [mm]\n(negative = carries load)"); ax[1].grid()
        ax[2].plot(idx, g["h"], "o-"); ax[2].set_ylabel("radial engagement depth [mm]"); ax[2].set_xlabel("pin index (theta = 0; pin 0 on the eccentric axis)"); ax[2].grid()
        fig.suptitle("Mesh geometry across all 51 pins"); fig.tight_layout(); fig.savefig(os.path.join(OUT, "pressure_angle_pins.png"), dpi=130); plt.close(fig)

        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(np.degrees(thetas), a_max_lever, label="at the max-lever pin")
        ax.plot(np.degrees(thetas), a_wmean, label="lever-weighted mean of loadable pins")
        ax.set_xlabel("input angle theta [deg]  (one pin pitch = full mesh period)"); ax.set_ylabel("pressure angle [deg]"); ax.legend(); ax.grid()
        ax.set_title("Pressure angle over the mesh cycle"); fig.tight_layout(); fig.savefig(os.path.join(OUT, "pressure_angle_cycle.png"), dpi=130); plt.close(fig)

        rec = evaluate(T_DESIGN, 2); sol = rec["sol"]; g = sol["g"]
        fig, ax = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
        ax[0].bar(idx, sol["F"]); ax[0].set_ylabel("pin force [N]"); ax[0].set_title(f"T_out = {T_DESIGN} Nm total, 2 discs ({T_DESIGN/2} Nm per disc), theta = {np.degrees(rec['theta']):.1f} deg")
        ax[1].bar(idx, rec["pmax"]); ax[1].axhline(MAT['PA-CF']['sig_c'], color="r", ls="--", label="PA-CF compressive ~90 MPa"); ax[1].set_ylabel("Hertz p_max [MPa]"); ax[1].legend()
        ax[2].bar(idx, np.where(sol["F"] > 0, rec["loss"], 0), label="radial separation: pin bending + plate + clearances"); ax[2].plot(idx, np.nan_to_num(rec["depth"], nan=0), "k.", label="contact depth below lobe tip (loaded pins)"); ax[2].set_ylabel("[mm]"); ax[2].legend(); ax[2].set_xlabel("pin index")
        fig.tight_layout(); fig.savefig(os.path.join(OUT, "load_sharing_design.png"), dpi=130); plt.close(fig)
        save_results(R_)

    if stage in ("2", "all"):
        sk = R_.get("skip", {})
        cases = {
            "baseline_2disc_PACF": dict(n_discs=2),
            "3disc_PACF": dict(n_discs=3),
            "2disc_PC_plates": dict(n_discs=2, mat_plate="PC"),
            "2disc_PETG_plates": dict(n_discs=2, mat_plate="PETG"),
            "2disc_PC_discs": dict(n_discs=2, mat_disc="PC"),
            "2disc_slipfit_pins_0.05": dict(n_discs=2, g_pin=0.05),
            "2disc_loose_ecc_0.10": dict(n_discs=2, g_ecc=0.10),
        }
        for name, kw in cases.items():
            if name in sk: continue
            T_skip, T_first = skip_torque(**kw)
            sk[name] = dict(T_skip=T_skip, T_first_pin_lost=T_first); R_["skip"] = sk; save_results(R_)
            print("skip", name, sk[name], flush=True)
        # what a typical printed cycloidal looks like in the same model: 4 mm pins cantilevered 20 mm from a
        # single 6 mm plate, thin 8 mm printed ring, slip-fit pins, PETG ring
        globals()["CANTILEVER_FREE_LEN"] = 20.0; globals()["THIN_RING"] = (8.0, 14.0)
        T_skip, T_first = skip_torque(n_discs=1, mat_plate="PETG", g_pin=0.05, g_ecc=0.05, Tmax=120)
        rec = evaluate(T_NOMINAL, 1, mat_plate="PETG", g_pin=0.05, g_ecc=0.05)
        sk["REFERENCE_cantilever_pins_thin_PETG_ring_1disc"] = dict(T_skip=T_skip, T_first_pin_lost=T_first,
            loss_at_18Nm_mm=float(rec["loss"][rec["sol"]["F"] > 0].max()), margin_at_18Nm_mm=float(rec["margin"].min()))
        globals()["CANTILEVER_FREE_LEN"] = None; globals()["THIN_RING"] = None
        R_["skip"] = sk; save_results(R_); print("skip reference", sk["REFERENCE_cantilever_pins_thin_PETG_ring_1disc"], flush=True)
        # ---- does the floating mid ring earn its 4 mm of length? -----------------
        cmp_ = {}
        for on in (False, True):
            globals()["MID_RING_ON"] = on
            rec = evaluate(T_NOMINAL, 2); recd = evaluate(T_DESIGN, 2)
            kk = per_pin_stiffness(MAT["PA-CF"]["E"], 0.4, x_discA(), L["ringpin_free_span"])[1]
            cmp_["mid_ring_" + ("on" if on else "off")] = dict(
                k_pin_N_per_mm=kk["k_total"], k_bend_N_per_mm=kk["k_bend"],
                n_pins_loaded_18Nm=rec["n_loaded"], F_pin_max_18Nm=float(rec["Fmax"]),
                margin_18Nm_mm=float(rec["margin"].min()), loss_18Nm_mm=float(rec["loss"][rec["sol"]["F"] > 0].max()),
                margin_36Nm_mm=float(recd["margin"].min()), pmax_18Nm=float(rec["pmax"].max()))
        globals()["MID_RING_ON"] = None
        R_["mid_ring_comparison"] = cmp_; save_results(R_)
        print("mid ring", json.dumps(cmp_, indent=1, default=float), flush=True)
        # margin vs torque plot
        Ts = np.linspace(5, 250, 25); mins = []; pm = []
        for T in Ts:
            r_ = evaluate(T, 2); mins.append(r_["margin"].min()); pm.append(r_["pmax"].max())
        R_["margin_vs_T"] = dict(T=Ts.tolist(), margin=[float(v) for v in mins], pmax=[float(v) for v in pm])
        # material-limited torques from p_max ~ sqrt(T) (Hertz line contact)
        p36 = float(np.interp(T_DESIGN, Ts, pm))
        R_["torque_limits"] = dict(
            T_design=T_DESIGN, pmax_at_design=p36,
            T_at_PACF_yield_90MPa=T_DESIGN*(90/p36)**2,
            T_at_PC_yield_70MPa=T_DESIGN*(70/p36)**2,
            T_flank_flow_180MPa_est=T_DESIGN*(180/p36)**2,
            T_skip_elastic_model=sk["baseline_2disc_PACF"]["T_skip"],
            note="skip torque in the elastic model exceeds the search cap; the lobe surface yields long before radial disengagement")
        fig, ax1 = plt.subplots(figsize=(10, 4.5)); ax2 = ax1.twinx()
        ax1.plot(Ts, mins, "b-", label="min engagement margin [mm]"); ax1.axhline(0, color="b", ls=":")
        ax2.plot(Ts, pm, "r-", label="max Hertz p_max [MPa]"); ax2.axhline(90, color="r", ls=":")
        ax1.axvline(T_NOMINAL, color="g", ls="--"); ax1.axvline(T_DESIGN, color="g", ls="--")
        ax1.set_xlabel("output torque [Nm]"); ax1.set_ylabel("margin to skip [mm]", color="b"); ax2.set_ylabel("p_max [MPa]", color="r")
        b = sk["baseline_2disc_PACF"]
        ax1.set_title(f"Skip margin & contact stress vs torque  (elastic model: no skip up to the {b['T_skip']:.0f} Nm search cap; lobe contact stress is the real limit)")
        fig.tight_layout(); fig.savefig(os.path.join(OUT, "margin_vs_torque.png"), dpi=130); plt.close(fig)
        save_results(R_)

    if stage in ("3", "all"):
        # ---- K sweep (ECC choice) and profile-clearance / disc-thickness sensitivity at design torque
        ksk = []; e_saved = e; rad_saved = rad
        for K in [0.6, 0.7, 0.75, 0.8, 0.82, 0.85, 0.9]:
            globals()["e"] = K*R/Zp
            px_, py_ = cycloid_profile(R, Rr, e, N, Zp, c_prof, pts_per_lobe=100); globals()["rad"] = np.hypot(px_, py_)
            u_ = undercut_check(R, Rr, e, N, Zp, c_prof)
            rec = evaluate(T_DESIGN, 2)
            ksk.append(dict(K=K, e=float(e), lobe_h=float(rad.max()-rad.min()), undercut_ok=u_["ok"], rho_min=u_["rho_min_convex"],
                            margin_min=float(rec["margin"].min()), pmax=float(rec["pmax"].max()), Fmax=float(rec["Fmax"]),
                            F_bearing=float(rec["sol"]["F_bearing"]), n_loaded=rec["n_loaded"]))
            print("Ksweep", ksk[-1], flush=True)
        globals()["e"] = e_saved; globals()["rad"] = rad_saved; globals()["_PROF"] = None
        R_["K_sweep_at_design_T"] = ksk
        csk = []; c_saved = c_prof
        for cc in [0.03, 0.05, 0.08, 0.10, 0.15, 0.20]:
            globals()["c_prof"] = cc; globals()["d_off"] = Rr + cc; globals()["_PROF"] = None
            px_, py_ = cycloid_profile(R, Rr, e, N, Zp, cc, pts_per_lobe=100); globals()["rad"] = np.hypot(px_, py_)
            rec = evaluate(T_DESIGN, 2)
            csk.append(dict(clearance=cc, n_loaded=rec["n_loaded"], Fmax=float(rec["Fmax"]), pmax=float(rec["pmax"].max()),
                            windup_deg=float(np.degrees(rec["sol"]["x"][0])), dC_t_mm=float(rec["sol"]["x"][2]), margin_min=float(rec["margin"].min())))
            print("csweep", csk[-1], flush=True)
        globals()["c_prof"] = c_saved; globals()["d_off"] = Rr + c_saved; globals()["rad"] = rad_saved; globals()["_PROF"] = None
        R_["clearance_sweep_at_design_T"] = csk
        tsk = []; t_saved = DISC_T
        for tt in [7.0, 10.0, 12.0, 14.0]:
            globals()["DISC_T"] = tt
            rec = evaluate(T_DESIGN, 2)
            tsk.append(dict(disc_T=tt, pmax=float(rec["pmax"].max()), Fmax=float(rec["Fmax"]), n_loaded=rec["n_loaded"]))
        globals()["DISC_T"] = t_saved
        R_["disc_thickness_sweep"] = tsk
        save_results(R_)

    if stage in ("4", "all"):
        def ripple(n):
            Fe, Fb = [], []
            for th in np.linspace(0, 2*np.pi/Zp, 12):
                tot = 0; vec = np.zeros(2)
                for k in range(n):
                    thk = th + 2*np.pi*k/n
                    sol = solve_load(thk, T_DESIGN/n, MAT["PA-CF"]["E"], 0.4, x_discA(), L["ringpin_free_span"])
                    tot += sol["Fe"]
                    a = -thk/N + 0*thk   # disc-frame -> ring-frame rotation
                    Rm = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
                    vec += Rm @ sol["F_bearing_vec"]
                Fe.append(tot); Fb.append(np.hypot(*vec))
            Fe = np.array(Fe); Fb = np.array(Fb)
            return dict(Fe_total_mean_N=float(Fe.mean()), Fe_ripple_pct=float(100*(Fe.max()-Fe.min())/Fe.mean()),
                        net_radial_force_on_shaft_max_N=float(Fb.max()))
        R_["ripple_2disc"] = ripple(2); R_["ripple_3disc"] = ripple(3)
        print("ripple", R_["ripple_2disc"], R_["ripple_3disc"])
        Ld = R_["load_design"]; Fb = Ld["F_ecc_bearing"]
        spacing = (L["z_camB"][0]+L["z_camB"][1])/2 - (L["z_camA"][0]+L["z_camA"][1])/2
        # both shaft journals now live in the carrier assembly (rear bearing in the carrier plate,
        # front 608 in the flange); the carrier is grounded on the 6804 + 6810 pair
        brg_span = (L["z_flange_rear"] + L["z_flange_608_bot"])/2 - (L["z_brg_rear_bot"] + L["z_brg_rear_top"])/2
        couple = Fb*spacing; R_shaft = couple/brg_span
        Mmax = Fb*spacing/2 + R_shaft*((L["z_camA"][0]+L["z_camA"][1])/2 - (L["z_brg_rear_bot"] + L["z_brg_rear_top"])/2)
        carrier_couple = Ld["F_roller_sum"]*spacing
        carrier_span = (L["z_brg_output"][0]+L["z_brg_output"][1])/2 - (L["z_pocket_carrier_bot"] + P["BRG_CARRIER"]["w"]/2)
        R_["loads"] = dict(
            per_disc_ecc_bearing_N=Fb, C0_rear_brg_N_verify=1000, C0_disc_brg_N_verify=1270,
            shaft_couple_Nmm=couple, shaft_bearing_span_mm=brg_span, shaft_support_reaction_N=R_shaft,
            shaft_Mmax_Nmm=Mmax, shaft_sigma_collar_MPa=Mmax/(math.pi*P["SHAFT_COLLAR_D"]**3/32),
            shaft_sigma_cam_MPa=Mmax/(math.pi*(P["SHAFT_CAM_D"] - 2*e)**3/32),
            shaft_sigma_rear12_MPa=(R_shaft*((L["z_camA"][0]) - L["z_brg_rear_top"]))/(math.pi*(P["SHAFT_REAR_D"]**4 - (P["MOTOR_SHAFT_D"]+0.1)**4)/(32*P["SHAFT_REAR_D"])),
            roller_max_N=Ld["F_roller_max"], C0_roller_N_verify=600, roller_sum_per_disc_N=Ld["F_roller_sum"],
            output_pin_span_mm=L["z_flange_rear"] - L["z_carrier_front"],
            output_pin_bending_MPa=Ld["F_roller_max"]*(L["z_flange_rear"] - L["z_carrier_front"])/4/(math.pi*P["OUT_PIN_D"]**3/32),
            output_pin_seat_pressure_MPa=Ld["F_roller_max"]/(P["OUT_PIN_D"]*P["CARRIER_T"]),
            carrier_couple_Nmm=carrier_couple, carrier_bearing_span_mm=carrier_span, carrier_bearing_reaction_N=carrier_couple/carrier_span,
            C0_carrier_brg_N_verify=1500, C0_output_brg_N_verify=8300,
            ring_pin_bending_MPa=Ld["F_pin_max"]*L["ringpin_free_span"]/4/(math.pi*P["PIN_D"]**3/32),
            ring_pin_seat_pressure_MPa=Ld["F_pin_max"]/2/(P["PIN_D"]*P["PIN_SEAT_REAR"]),
            motor_shaft_torque_Nm=T_DESIGN/N, motor_D_flat_pressure_MPa=(T_DESIGN/N*1000/2.0)/(4.0*L["d_bore_depth"]),
            housing_bolt_shear_N=T_DESIGN*1000/(P["HOUSING_BOLT_PCD"]/2)/P["N_HOUSING_BOLTS"])
        # ---- external arm load (a Z force at a reach along X) -------------------
        m_kg, reach_mm, dyn = 2.0, 300.0, 2.0
        W = m_kg*9.81
        M_static = W*reach_mm                     # N mm
        M_dyn = M_static*dyn
        span = (L["z_brg_output"][0]+L["z_brg_output"][1])/2 - (L["z_pocket_carrier_bot"] + P["BRG_CARRIER"]["w"]/2)
        pin_span = L["z_flange_rear"] - L["z_carrier_front"]
        # the couple is reacted by the two grounded bearings; the output pins carry the part of it
        # that has to cross from the flange to the carrier
        R_["arm_load"] = dict(
            mass_kg=m_kg, reach_mm=reach_mm, dynamic_factor=dyn,
            W_N=W, M_static_Nm=M_static/1000, M_dynamic_Nm=M_dyn/1000,
            bearing_span_mm=span,
            R_front_6810_N=M_dyn/span + W, R_rear_6804_N=M_dyn/span,
            C0_6810_N_verify=8300, C0_6804_N_verify=2240,
            pin_couple_share_N=M_dyn/pin_span/P["N_OUT"],
            flange_tilt_deg_est=math.degrees(M_dyn/ (MAT["PA-CF"]["E"]*math.pi*(P["FLANGE_BODY_OD"]/2)**3*P["FLANGE_BODY_T"]/4 / (P["FLANGE_BODY_OD"]/2))) ,
            note="reach measured from the output face; dynamic factor covers acceleration and knocks")
        print("arm", json.dumps(R_["arm_load"], indent=1, default=float))
        print("loads", json.dumps(R_["loads"], indent=1, default=float))
        def efficiency(mu_f, T=T_NOMINAL):
            loss = 0; pout = T*1000/N
            ths = np.linspace(0, 2*np.pi/Zp, 8)
            for th in ths:
                sol = solve_load(th, T/2, MAT["PA-CF"]["E"], 0.4, x_discA(), L["ringpin_free_span"]); g = sol["g"]
                for j in np.where(sol["F"] > 0)[0]:
                    kx, ky = g["k"][j]
                    v = np.array([ky/N, -kx/N]) + e*g["t"]
                    vt = v - (v @ g["n"][j])*g["n"][j]
                    loss += mu_f*sol["F"][j]*np.hypot(*vt)
            loss = loss/len(ths)*2
            return float(pout/(pout + loss))
        R_["efficiency_mesh_only"] = dict(mu_0p10_greased=efficiency(0.10), mu_0p20=efficiency(0.20), mu_0p30_dry=efficiency(0.30),
                                          note="ring-pin sliding only; add ~2-4% for bearings/rollers and seals")
        ppf = plate_props("front", MAT["PA-CF"]["E"]); EI_plate = ppf["E"]*ppf["I"]
        bands = {}
        for name, (E_b, t_b) in {"Al 6061 band 3 mm wall": (69e3, 3.0), "Al 6061 band 6 mm wall": (69e3, 6.0),
                                 "steel band 1 mm (hose clamp)": (200e3, 1.0), "steel band 2 mm": (200e3, 2.0)}.items():
            EI_b = E_b*ppf["h"]*t_b**3/12
            bands[name] = dict(EI_add_Nmm2=EI_b, EI_plate_Nmm2=EI_plate, bending_stiffness_gain_pct=100*EI_b/EI_plate)
        R_["band_comparison_front_plate"] = bands
        R_["plate_front"] = ppf; R_["plate_rear"] = plate_props("rear", MAT["PA-CF"]["E"])
        R_["efficiency_note"] = "0.45 Nm x 50 x eta -> expected output torque"
        save_results(R_)
        print("wrote", RES_PATH)
