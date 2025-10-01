import numpy as np

def static_sag(P, X0, th0, road):
    """
    Port of static_sag.m

    Returns
    -------
    zuf0, zur0, Y0, etaF, etaR, deltaF, deltaR
    """
    s0  = float(road.hpC(X0))
    d0  = np.sqrt(1.0 + s0**2)
    n0  = np.array([-s0, 1.0])/d0
    ny0 = n0[1]
    u0  = np.array([np.cos(th0), np.sin(th0)])

    # Gravity component normal to road
    g_normal = P.g * ny0

    # Load split for chassis (beam theory)
    L = P.lf + P.lr
    Fsf_ch = (P.lr / L) * P.ms * g_normal
    Fsr_ch = (P.lf / L) * P.ms * g_normal

    # Suspension spring deflections
    etaF = abs(Fsf_ch) / P.ksf
    etaR = abs(Fsr_ch) / P.ksr

    # Tyre compressions
    deltaF = abs(P.muf * g_normal - P.ksf * etaF) / P.ktf
    deltaR = abs(P.mur * g_normal - P.ksr * etaR) / P.ktr

    # Pick-up positions
    pFx = X0 + P.lf * u0[0]
    pRx = X0 - P.lr * u0[0]

    # Total effective lengths
    Lf = P.LsF + P.LtF - etaF - deltaF
    Lr = P.LsR + P.LtR - etaR - deltaR

    # Abscissa at road contact along body orientation (as in MATLAB code)
    rFx = pFx + Lf * u0[1]
    rRx = pRx + Lr * u0[1]

    # Absolute y of unsprung masses
    zuf0 = float(road.h(rFx)) + (P.LtF - deltaF) * n0[1]
    zur0 = float(road.h(rRx)) + (P.LtR - deltaR) * n0[1]

    # Chassis center Y0 (weighted average)
    Y0 = (P.lf/L) * (zuf0 + (P.LsF - etaF) * n0[1]) +              (P.lr/L) * (zur0 + (P.LsR - etaR) * n0[1])

    return zuf0, zur0, Y0, etaF, etaR, deltaF, deltaR
