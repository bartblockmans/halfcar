# halfcar/physics/contact.py
import numpy as np

def circle_curve_contact(xc: float, yc: float, R: float,
                         X_curve, Y_curve):
    """
    Port of circleCurveContact.m

    Inputs
    ------
    xc, yc : circle center
    R      : circle radius (>0)
    X_curve, Y_curve : polyline vertices (1D arrays)

    Returns
    -------
    x_contact, y_contact : contact point (midpoint of chosen intersection pair)
    delta                : penetration depth (>=0)
    n_circle             : inward unit normal of circle at contact (2,)
    contact_state        : 1 if contact (>=1 intersection), else 0
    """
    x_contact = 0.0
    y_contact = 0.0
    delta = 0.0
    n_circle = np.array([0.0, 1.0], dtype=float)
    contact_state = 0

    X_curve = np.asarray(X_curve, dtype=float).ravel()
    Y_curve = np.asarray(Y_curve, dtype=float).ravel()
    N = X_curve.size
    if N < 2 or R <= 0:
        return x_contact, y_contact, delta, n_circle, contact_state

    # quick bounding box reject
    xmin, xmax = float(X_curve.min()), float(X_curve.max())
    ymin, ymax = float(Y_curve.min()), float(Y_curve.max())
    if (xmax < xc - R) or (xmin > xc + R) or (ymax < yc - R) or (ymin > yc + R):
        return x_contact, y_contact, delta, n_circle, contact_state

    # segment endpoints and directions
    Ax = X_curve[:-1].copy(); Ay = Y_curve[:-1].copy()
    Bx = X_curve[1:].copy();  By = Y_curve[1:].copy()
    dx = Bx - Ax; dy = By - Ay

    a = dx**2 + dy**2          # |d|^2
    valid = a > 1e-16
    if not np.any(valid):
        return x_contact, y_contact, delta, n_circle, contact_state

    Ax = Ax[valid]; Ay = Ay[valid]
    dx = dx[valid]; dy = dy[valid]
    Bx = Ax + dx;   By = Ay + dy
    a  = a[valid]

    fx = Ax - xc;  fy = Ay - yc
    b = 2.0*(dx*fx + dy*fy)
    c = (fx*fx + fy*fy) - R*R
    disc = b*b - 4.0*a*c

    has_real = disc >= 0.0
    if not np.any(has_real):
        return x_contact, y_contact, delta, n_circle, contact_state

    a = a[has_real]; b = b[has_real]; c = c[has_real]
    Ax = Ax[has_real]; Ay = Ay[has_real]
    dx = dx[has_real]; dy = dy[has_real]

    sqrtDisc = np.sqrt(np.maximum(disc[has_real], 0.0))
    inv2a = 0.5 / a
    t1 = (-b - sqrtDisc) * inv2a
    t2 = (-b + sqrtDisc) * inv2a

    tol = 1e-12
    P = []
    if np.any((t1 >= -tol) & (t1 <= 1.0 + tol)):
        tt = np.clip(t1[(t1 >= -tol) & (t1 <= 1.0 + tol)], 0.0, 1.0)
        Px = Ax[(t1 >= -tol) & (t1 <= 1.0 + tol)] + tt * dx[(t1 >= -tol) & (t1 <= 1.0 + tol)]
        Py = Ay[(t1 >= -tol) & (t1 <= 1.0 + tol)] + tt * dy[(t1 >= -tol) & (t1 <= 1.0 + tol)]
        P.append(np.stack([Px, Py], axis=1))
    if np.any((t2 >= -tol) & (t2 <= 1.0 + tol)):
        tt = np.clip(t2[(t2 >= -tol) & (t2 <= 1.0 + tol)], 0.0, 1.0)
        Px = Ax[(t2 >= -tol) & (t2 <= 1.0 + tol)] + tt * dx[(t2 >= -tol) & (t2 <= 1.0 + tol)]
        Py = Ay[(t2 >= -tol) & (t2 <= 1.0 + tol)] + tt * dy[(t2 >= -tol) & (t2 <= 1.0 + tol)]
        P.append(np.stack([Px, Py], axis=1))
    if not P:
        return x_contact, y_contact, delta, n_circle, contact_state

    P = np.vstack(P)

    # remove duplicates with tolerance
    if P.shape[0] > 1:
        P = _unique_rows_tol(P, tol=1e-10)

    # choose contact point
    if P.shape[0] == 1:
        M = P[0]
    else:
        # pick pair with max penetration (deepest midpoint)
        n = P.shape[0]
        bestDelta = -np.inf
        bestM = np.array([0.0, 0.0])
        for i in range(n-1):
            for j in range(i+1, n):
                Mij = 0.5*(P[i] + P[j])
                d = np.hypot(Mij[0]-xc, Mij[1]-yc)
                delt = max(0.0, R - d)
                if delt > bestDelta:
                    bestDelta = delt
                    bestM = Mij
        M = bestM

    x_contact = float(M[0])
    y_contact = float(M[1])
    dC = np.array([xc - x_contact, yc - y_contact], dtype=float)
    nrm = np.hypot(dC[0], dC[1])
    if nrm < 1e-15:
        n_circle = np.array([0.0, 1.0], dtype=float)
        delta = float(R)
    else:
        n_circle = dC / nrm
        delta = float(max(0.0, R - nrm))
    contact_state = 1
    return x_contact, y_contact, delta, n_circle, contact_state


def _unique_rows_tol(P: np.ndarray, tol=1e-10):
    """Remove near-duplicate rows (|diff|<tol) preserving order."""
    keep = [True]
    for k in range(1, P.shape[0]):
        if np.all(np.abs(P[k] - P[k-1]) < tol):
            keep.append(False)
        else:
            keep.append(True)
    return P[np.array(keep, dtype=bool)]
