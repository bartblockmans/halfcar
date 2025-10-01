# halfcar/physics/power_data.py
import numpy as np

def power_data(P, road):
    """
    Port of PowerData.m

    Parameters
    ----------
    P : params object with attributes:
        - lr : float
        - Tmax : float
        - power : (N x 3) array-like [[x_start, x_end, P_value], ...]
    road : object with attribute:
        - x_course : array

    Returns
    -------
    P_road : np.ndarray  (same length as road.x_course)
    T_max  : np.ndarray  (same length as road.x_course)
    """
    x_course = np.asarray(road.x_course, dtype=float)
    P_road = np.zeros_like(x_course)
    T_max = np.zeros_like(x_course)

    power_rows = getattr(P, "power", None)
    if power_rows is None or (isinstance(power_rows, (list, tuple)) and len(power_rows) == 0):
        return P_road, T_max

    power_rows = np.asarray(power_rows, dtype=float)
    if power_rows.ndim == 1 and power_rows.size == 3:
        power_rows = power_rows.reshape(1, 3)

    for i in range(power_rows.shape(0) if callable(getattr(power_rows, 'shape', None)) else power_rows.shape[0]):
        x_start = power_rows[i, 0]
        if x_start < P.lr:
            x_start = P.lr
        ind_start = int(np.argmin(np.abs(x_course - x_start)))

        x_end = power_rows[i, 1]
        ind_end = int(np.argmin(np.abs(x_course - x_end)))
        if ind_end < ind_start:
            ind_start, ind_end = ind_end, ind_start

        P_val = float(power_rows[i, 2])

        if ind_end == ind_start:
            P_road[ind_start] = max(P_road[ind_start], P_val)
            T_max[ind_start] = max(T_max[ind_start], P.Tmax)
            continue

        x_seg = x_course[ind_start:ind_end + 1]
        P_seg = _haversine_power(P_val, x_seg, x_trans=0.5)
        P_road[ind_start:ind_end + 1] = P_seg

        # Scale torque proportionally to the local power profile
        with np.errstate(divide='ignore', invalid='ignore'):
            ratio = np.where(P_val > 0.0, P_seg / P_val, 0.0)
        T_max[ind_start:ind_end + 1] = ratio * P.Tmax

    return P_road, T_max


def _haversine_power(P_val, x, x_trans=0.5):
    """Haversine upramp + downramp profile, then interpolate onto x."""
    x = np.asarray(x, dtype=float)

    # Upramp
    x_up = np.linspace(x[0], x[0] + x_trans, 101)
    P_up = _haversine(x_up[0], x_up[-1], 0.0, P_val, x_up)

    # Downramp
    x_down = np.linspace(x[-1] - x_trans, x[-1], 101)
    P_down = _haversine(x_down[0], x_down[-1], P_val, 0.0, x_down)

    # Assemble & interpolate
    x_ud = np.concatenate([x_up, x_down])
    P_ud = np.concatenate([P_up, P_down])
    order = np.argsort(x_ud)  # ensure monotone
    return np.interp(x, x_ud[order], P_ud[order])


def _haversine(t0, t1, F0, F1, t_all):
    """Vectorized port of Haversine() from MATLAB."""
    t_all = np.asarray(t_all, dtype=float)
    F = np.zeros_like(t_all)

    mask_lo = t_all <= t0
    mask_hi = t_all >= t1
    mask_mid = (~mask_lo) & (~mask_hi)

    F[mask_lo] = F0
    F[mask_hi] = F1

    if np.any(mask_mid):
        t = t_all[mask_mid]
        s = (np.pi * (t - t0) / (t1 - t0) + np.pi * (F0 > F1)) / 2.0
        F_mid = F0 + (F1 - F0) * ((F0 > F1) - (np.sin(s) ** 2) * ((F0 > F1) + (F0 < F1) * (-1)))
        F[mask_mid] = F_mid

    return F
