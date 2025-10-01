# halfcar/road/smooth_fillet.py
import numpy as np
import warnings

def smooth_fillet(x1, y1, d1, d2, grad1, grad2, R_curve):
    """
    Port of smooth_fillet.m (your version).
    Returns X, Y polyline arrays.
    """
    if R_curve <= 0:
        raise ValueError("R_curve must be positive.")
    if d1 <= 0 or d2 <= 0:
        raise ValueError("d1 and d2 must be positive.")

    th1 = np.arctan(grad1)
    th2 = np.arctan(grad2)
    u1 = np.array([np.cos(th1), np.sin(th1)])
    u2 = np.array([np.cos(th2), np.sin(th2)])

    P0 = np.array([x1, y1], dtype=float)
    Pknee = P0 + d1 * u1

    dot12 = float(np.clip(np.dot(u1, u2), -1.0, 1.0))
    Delta = float(np.arccos(dot12))

    epsA = 1e-12
    if Delta < epsA:
        # straight through
        P_end = P0 + (d1 + d2) * u1
        n1 = max(2, int(np.ceil(d1/0.01)))
        n2 = max(2, int(np.ceil(d2/0.01)))
        X = np.concatenate([np.linspace(P0[0], Pknee[0], n1),
                            np.linspace(Pknee[0], P_end[0], n2)])
        Y = np.concatenate([np.linspace(P0[1], Pknee[1], n1),
                            np.linspace(Pknee[1], P_end[1], n2)])
        return _dedup_join(X, Y)

    if abs(Delta - np.pi) < 1e-9:
        # U-turn corner, no finite fillet
        warnings.warn("Directions are opposite; cannot place a finite-radius fillet. Returning sharp corner.")
        n1 = max(2, int(np.ceil(d1/0.01)))
        n2 = max(2, int(np.ceil(d2/0.01)))
        X = np.concatenate([np.linspace(P0[0], Pknee[0], n1),
                            np.linspace(Pknee[0], (Pknee + d2*u2)[0], n2)])
        Y = np.concatenate([np.linspace(P0[1], Pknee[1], n1),
                            np.linspace(Pknee[1], (Pknee + d2*u2)[1], n2)])
        return _dedup_join(X, Y)

    t_needed = R_curve * np.tan(Delta/2.0)
    t_max = min(d1, d2)
    if t_needed > t_max - 1e-12:
        R_old = R_curve
        t_fit = max(t_max - 1e-9, 0.0)
        if np.tan(Delta/2.0) < epsA:
            R_curve = 0.0
        else:
            R_curve = t_fit / np.tan(Delta/2.0)
        if R_curve <= 0.0:
            warnings.warn("Fillet radius too large for given d1/d2; returning sharp corner.")
            n1 = max(2, int(np.ceil(d1/0.01)))
            n2 = max(2, int(np.ceil(d2/0.01)))
            X = np.concatenate([np.linspace(P0[0], Pknee[0], n1),
                                np.linspace(Pknee[0], (Pknee + d2*u2)[0], n2)])
            Y = np.concatenate([np.linspace(P0[1], Pknee[1], n1),
                                np.linspace(Pknee[1], (Pknee + d2*u2)[1], n2)])
            return _dedup_join(X, Y)
        else:
            warnings.warn(f"Fillet radius reduced from {R_old:.6g} to {R_curve:.6g} to fit d1/d2.")
        t = t_fit
    else:
        t = t_needed

    T1 = Pknee - t * u1
    T2 = Pknee + t * u2

    z = u1[0]*u2[1] - u1[1]*u2[0]
    sgn = 1.0 if z >= 0 else -1.0

    n1v = sgn * np.array([-u1[1], u1[0]])
    C = T1 + R_curve * n1v

    a1 = np.arctan2(T1[1]-C[1], T1[0]-C[0])
    a2 = np.arctan2(T2[1]-C[1], T2[0]-C[0])

    if sgn > 0:  # CCW
        if a2 <= a1:
            a2 += 2*np.pi
        A = np.linspace(a1, a2, max(12, int(np.ceil((a2-a1)/(np.pi/90)))))
    else:        # CW
        if a2 >= a1:
            a2 -= 2*np.pi
        A = np.linspace(a1, a2, max(12, int(np.ceil((a1-a2)/(np.pi/90)))))

    Xarc = C[0] + R_curve*np.cos(A)
    Yarc = C[1] + R_curve*np.sin(A)

    n1s = max(2, int(np.ceil(max(10.0, np.linalg.norm(T1-P0)/0.01))))
    n2s = max(2, int(np.ceil(max(10.0, np.linalg.norm((Pknee + (d2 - t)*u2)-T2)/0.01))))
    X1 = np.linspace(P0[0], T1[0], n1s)
    Y1 = np.linspace(P0[1], T1[1], n1s)
    X2 = np.linspace(T2[0], (Pknee + (d2 - t)*u2)[0], n2s)
    Y2 = np.linspace(T2[1], (Pknee + (d2 - t)*u2)[1], n2s)

    X = np.concatenate([X1, Xarc, X2])
    Y = np.concatenate([Y1, Yarc, Y2])
    return _dedup_join(X, Y)


def _dedup_join(X, Y):
    keep = [True]
    for k in range(1, len(X)):
        if abs(X[k]-X[k-1]) < 1e-12 and abs(Y[k]-Y[k-1]) < 1e-12:
            keep.append(False)
        else:
            keep.append(True)
    keep = np.array(keep, dtype=bool)
    return X[keep], Y[keep]
