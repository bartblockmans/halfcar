# halfcar/post/compute_results.py
import numpy as np
from scipy.interpolate import interp1d
from physics.rhs import compute_accelerations  # must support return_output=True

def compute_results(t, y, P, road):
    """
    Port of ComputeResults.m

    Downsamples to dt=0.01s and recomputes helper outputs via RHS in
    'return_output' mode to avoid duplicating kinematics.
    """
    print("Post-processing results for plotting & animation...")
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    if y.ndim == 1:
        y = y[None, :]

    # Uniform time grid (like MATLAB)
    t_print = 1e-2
    nt = int(np.ceil(t[-1] / t_print))
    time = np.linspace(0.0, nt * t_print, nt + 1)

    # Interpolate states onto uniform grid
    y_uniform = np.empty((time.size, y.shape[1]), dtype=float)
    for j in range(y.shape[1]):
        f = interp1d(t, y[:, j], kind="linear", fill_value="extrapolate", assume_sorted=True)
        y_uniform[:, j] = f(time)

    # Preallocate outputs we need for plots/animation
    out = {
        "time": time,
        "uRx": np.zeros(time.size),
        "uRy": np.zeros(time.size),
        "etaF": np.zeros(time.size),
        "etaR": np.zeros(time.size),
        "FsF": np.zeros(time.size),
        "FsR": np.zeros(time.size),
        "deltaF": np.zeros(time.size),
        "deltaR": np.zeros(time.size),
        "Nf": np.zeros(time.size),
        "Nr": np.zeros(time.size),
    }

    # Evaluate per-sample using the existing kinematics from the RHS
    for i in range(time.size):
        _, output = compute_accelerations(
            time[i], y_uniform[i, :], P, road, debug=False, return_output=True
        )
        out["uRx"][i] = output.get("uRx", np.nan)
        out["uRy"][i] = output.get("uRy", np.nan)
        out["etaF"][i] = output.get("etaF", np.nan)
        out["etaR"][i] = output.get("etaR", np.nan)
        out["FsF"][i] = output.get("FsF", np.nan)
        out["FsR"][i] = output.get("FsR", np.nan)
        out["deltaF"][i] = output.get("deltaF", np.nan)
        out["deltaR"][i] = output.get("deltaR", np.nan)
        out["Nf"][i] = output.get("Nf", np.nan)
        out["Nr"][i] = output.get("Nr", np.nan)

    print("Post-processing completed.\n")
    return out
