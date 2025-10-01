import numpy as np
from physics.contact import circle_curve_contact

def _find_road_intersection(p, n_chassis, road):
    # Solve p(2) + t*n_chassis(2) = road.h(p(1) + t*n_chassis(1))
    # Newton-Raphson with numeric derivative
    def f(t):
        x = p[0] + t*n_chassis[0]
        y = p[1] + t*n_chassis[1]
        return y - road.h(x)
    t = 0.0
    for _ in range(20):
        ft = f(t)
        if abs(ft) < 1e-12:
            break
        dt = 1e-6
        dfdt = (f(t+dt) - f(t-dt)) / (2*dt)
        if abs(dfdt) < 1e-12:
            t -= ft*0.1
        else:
            t -= ft/dfdt
        if abs(t) > 1e6:
            t = 0.0
            break
    return t

def _circle_curve_contact(ux, uy, R, x_grid, y_road, road):
    # Circle lower boundary at x: y_circ(x) = uy - sqrt(R^2 - (x-ux)^2), valid when |x-ux|<=R
    dx = x_grid - ux
    inside = np.abs(dx) <= R
    if not np.any(inside):
        return ux, uy-R, 0.0, np.array([0.0,1.0]), False
    yy = np.empty_like(x_grid); yy[:] = -np.inf
    yy[inside] = uy - np.sqrt(np.maximum(0.0, R*R - dx[inside]**2))
    delta = y_road - yy  # penetration if positive
    idx = np.argmax(delta)
    delta_max = float(delta[idx])
    x_contact = float(x_grid[idx])
    y_contact = float(y_road[idx])
    if delta_max > 0.0:
        s = road.hpC(x_contact)
        d = np.sqrt(1.0 + s*s)
        n = np.array([-s, 1.0]) / d
        return x_contact, y_contact, delta_max, n, True
    else:
        return ux, uy-R, 0.0, np.array([0.0,1.0]), False

def compute_accelerations(t, y, P, road, debug=False, return_output=False):
    # Unpack
    X,Y,th, zuf, zur, Xd,Yd,thd, zdf, zdr = y

    # Body axes
    u = np.array([np.cos(th), np.sin(th)])
    u_perp = np.array([-np.sin(th), np.cos(th)])

    # Chassis normal
    n_chassis = np.array([-np.sin(th), np.cos(th)])
    nd_chassis = np.array([-np.cos(th), -np.sin(th)]) * thd

    # Pickups and velocities
    pF = np.array([X,Y]) + P.lf * u
    pR = np.array([X,Y]) - P.lr * u
    vF = np.array([Xd,Yd]) + P.lf * thd * u_perp
    vR = np.array([Xd,Yd]) - P.lr * thd * u_perp

    # Intersection with road along chassis normal
    tF_param = _find_road_intersection(pF, n_chassis, road)
    tR_param = _find_road_intersection(pR, n_chassis, road)
    rF = pF + tF_param * n_chassis
    rR = pR + tR_param * n_chassis

    # Road slopes, normals, tangents
    sF = float(road.hpC(rF[0]));  sR = float(road.hpC(rR[0]))
    dF = np.sqrt(1.0 + sF*sF);    dR = np.sqrt(1.0 + sR*sR)
    nF = np.array([-sF, 1.0]) / dF;   nR = np.array([-sR, 1.0]) / dR
    tF = np.array([1.0, sF]) / dF;    tR = np.array([1.0, sR]) / dR

    # Road velocities at contact points (assuming road stationary, param by x)
    rFd = np.array([Xd, sF*Xd])
    rRd = np.array([Xd, sR*Xd])

    # Optional n_dot
    if P.use_n_dot:
        sFp = float(road.hppC(rF[0]))
        sRp = float(road.hppC(rR[0]))
        nFd = -(sFp*rFd[0])/(dF**3) * np.array([1.0, sF])
        nRd = -(sRp*rRd[0])/(dR**3) * np.array([1.0, sR])
    else:
        nFd = np.array([0.0,0.0]); nRd = np.array([0.0,0.0])

    # Suspension endpoints
    suspF = pF - n_chassis * P.LsF
    suspR = pR - n_chassis * P.LsR

    # Unsprung mass positions and velocities (projected geometry)
    # uFx,uFy via vertical coords zuf and pitch geometry
    uFy = zuf
    uFx = pF[0] + (pF[1] - zuf) * np.tan(th)
    uRy = zur
    uRx = pR[0] + (pR[1] - zur) * np.tan(th)
    uF = np.array([uFx, uFy]); uR = np.array([uRx, uRy])

    vUFy = zdf
    vUFx = (vF[0] + (vF[1]-zdf)*np.tan(th) + (pF[1]-zuf)*(1/np.cos(th))**2 * thd)
    vURy = zdr
    vURx = (vR[0] + (vR[1]-zdr)*np.tan(th) + (pR[1]-zur)*(1/np.cos(th))**2 * thd)
    vUF = np.array([vUFx, vUFy]); vUR = np.array([vURx, vURy])

    # Wheel contact points from unsprung masses
    cF = uF - P.LtF * n_chassis
    cR = uR - P.LtR * n_chassis
    vCF = vUF - P.LtF * nd_chassis
    vCR = vUR - P.LtR * nd_chassis

    # Suspension deflections
    etaF  = float(n_chassis @ (uF - suspF))
    etaR  = float(n_chassis @ (uR - suspR))
    detaF = float(n_chassis @ (vUF - vF) + nd_chassis @ (uF - suspF))
    detaR = float(n_chassis @ (vUR - vR) + nd_chassis @ (uR - suspR))

    # Suspension forces
    FsF = P.ksf*etaF + P.csf*detaF
    FsR = P.ksr*etaR + P.csr*detaR

    # Contact model
    contact_state_f = False
    contact_state_r = False

    if P.contact_method.lower() == "point":
        # Point contact (unilateral default)
        deltaF = float(nF @ (rF - cF))
        deltaR = float(nR @ (rR - cR))
        deldF = float(nF @ (rFd - vCF) + nFd @ (rF - cF))
        deldR = float(nR @ (rRd - vCR) + nRd @ (rR - cR))
        if getattr(P,'contact','unilateral').lower() == 'bilateral':
            Nf = P.ktf*deltaF + P.ctf*deldF; contact_state_f = True
            Nr = P.ktr*deltaR + P.ctr*deldR; contact_state_r = True
        else:
            if deltaF > 0:
                Nf = max(0.0, P.ktf*deltaF + P.ctf*deldF); contact_state_f = True
            else:
                Nf = 0.0; deltaF = 0.0
            if deltaR > 0:
                Nr = max(0.0, P.ktr*deltaR + P.ctr*deldR); contact_state_r = True
            else:
                Nr = 0.0; deltaR = 0.0
    elif P.contact_method.lower() == "circle":
        # --- Front tyre (unilateral circle contact) ---
        xf_road = np.linspace(cF[0]-0.5, cF[0]+0.5, 100)
        zf_road = road.h(xf_road)
        xf_contact, yf_contact, deltaF, nf_contact, contact_state_f = \
            circle_curve_contact(uF[0], uF[1], P.LtF, xf_road, zf_road)

        # NOTE: your MATLAB forms deldF differently in circle mode:
        deldF = nF @ (rFd - vUF)   # unchanged from your script

        if contact_state_f:
            Nf = max(0.0, (P.ktf*deltaF + P.ctf*deldF) * nf_contact[1])
        else:
            Nf = 0.0
            deltaF = 0.0

        # --- Rear tyre ---
        xr_road = np.linspace(cR[0]-0.5, cR[0]+0.5, 100)
        zr_road = road.h(xr_road)
        xr_contact, yr_contact, deltaR, nr_contact, contact_state_r = \
            circle_curve_contact(uR[0], uR[1], P.LtR, xr_road, zr_road)

        deldR = nR @ (rRd - vUR)

        if contact_state_r:
            Nr = max(0.0, (P.ktr*deltaR + P.ctr*deldR) * nr_contact[1])
        else:
            Nr = 0.0
            deltaR = 0.0

    # Rolling resistance
    RrF = 0.0; RrR = 0.0
    if getattr(P,'contact','unilateral').lower() == 'bilateral' or (locals().get('deltaF',0.0) > 0.0):
        RrF = -np.sign(Xd) * P.Crr * Nf
    if getattr(P,'contact','unilateral').lower() == 'bilateral' or (locals().get('deltaR',0.0) > 0.0):
        RrR = -np.sign(Xd) * P.Crr * Nr

    # Aerodynamic resistance
    Raero = np.array([0.0, 0.0])
    V_mag = np.hypot(Xd, Yd)
    if V_mag > 0.0:
        V_unit = np.array([Xd, Yd]) / V_mag
        Raero = -0.5 * P.rho * (V_mag**2) * P.CdA * V_unit

    # Propulsive power
    Fprop = 0.0
    if contact_state_r and contact_state_f:
        # road.P and road.T_max may be arrays or Nones (stubs). Guard:
        try:
            Pprop = float(np.interp(X, road.x_course, getattr(road, 'P', np.zeros_like(road.x_course))))
            Tmax  = float(np.interp(X, road.x_course, getattr(road, 'T_max', np.full_like(road.x_course, P.Tmax))))
            if abs(Xd) > 1e-6:
                Fprop = max(0.0, min(Pprop/max(1e-6,Xd), Tmax/max(1e-6,P.LtR)))
        except Exception:
            Fprop = 0.0

    # Accelerations
    Xdd = (FsF*n_chassis[0] + FsR*n_chassis[0] + RrF*tF[0] + RrR*tR[0] + Raero[0] + Fprop * tR[0]) / P.ms
    Ydd = (FsF*n_chassis[1] + FsR*n_chassis[1] + RrF*tF[1] + RrR*tR[1] + Raero[1] + Fprop * tR[1] + P.ms*P.g*n_chassis[1]) / P.ms

    cross2 = lambda a,b: a[0]*b[1] - a[1]*b[0]
    Mz = cross2(pF-np.array([X,Y]), FsF*n_chassis + RrF*tF) + cross2(pR-np.array([X,Y]), FsR*n_chassis + RrR*tR)

    C_pitch = getattr(P, 'C_pitch', 50.0)
    thdd = (Mz - C_pitch * thd) / P.Is

    zddf = (-FsF*(n_chassis @ nF) + Nf + P.muf*P.g*nF[1]) / P.muf
    zddr = (-FsR*(n_chassis @ nR) + Nr + P.mur*P.g*nR[1]) / P.mur

    dydt = np.array([Xd, Yd, thd, zdf, zdr, Xdd, Ydd, thdd, zddf, zddr], dtype=float)

    if return_output:
        output = {
            "uRx": uRx,
            "uRy": uRy,
            "etaF": etaF,
            "etaR": etaR,
            "FsF": FsF,
            "FsR": FsR,
            "deltaF": deltaF if "deltaF" in locals() else 0.0,
            "deltaR": deltaR if "deltaR" in locals() else 0.0,
            "Nf": Nf if "Nf" in locals() else 0.0,
            "Nr": Nr if "Nr" in locals() else 0.0,
        }
        return dydt, output
    else:
        return dydt
