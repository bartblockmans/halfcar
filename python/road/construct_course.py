import numpy as np
from road.smooth_fillet import smooth_fillet

def construct_course(segments, resolution=10):
    if not segments:
        raise ValueError("At least one segment must be defined")
    x_course = None
    z_course = None
    segment_boundaries = []
    current_x = 0.0
    current_z = 0.0
    current_slope = 0.0

    for i, seg in enumerate(segments, start=1):
        x, z, slope_end = _generate_segment(seg, current_slope, resolution)
        if i == 1:
            x = x + current_x
            z = z + current_z
        else:
            x = x - x[0] + current_x
            z = z - z[0] + current_z
        if x_course is None:
            x_course = x
            z_course = z
        else:
            x_course = np.concatenate([x_course[:-1], x])
            z_course = np.concatenate([z_course[:-1], z])
        current_x = float(x_course[-1])
        current_z = float(z_course[-1])
        current_slope = slope_end
        segment_boundaries.append(current_x)

    if np.any(np.diff(x_course) <= 0):
        raise RuntimeError("Course generation failed: non-monotonic x-coordinates")

    course_info = {
        "segments": segments,
        "segment_boundaries": np.array(segment_boundaries, dtype=float),
        "total_distance": float(current_x),
    }
    print(f"Course constructed: {course_info['total_distance']:.1f} m total, {len(segments)} segments, {len(x_course)} points")
    return x_course, z_course, course_info

# ----------------------- Segment Generators ---------------------------
def _generate_segment(seg, slope_start, resolution):
    typ = seg["type"].lower()
    if typ == "flat":
        return _flat(seg, slope_start, resolution)
    if typ == "bumpy":
        return _bumpy(seg, slope_start, resolution)
    if typ == "jump":
        return _jump(seg, slope_start, resolution)
    if typ == "drop":
        return _drop(seg, slope_start, resolution)
    if typ == "staircase":
        return _staircase(seg, slope_start, resolution)
    if typ == "trapezium":
        return _trapezium(seg, slope_start, resolution)
    if typ == "gap_jump":
        return _gap_jump(seg, slope_start, resolution)
    if typ == "rock_garden":
        return _rock_garden(seg, slope_start, resolution)
    if typ == "root_section":
        return _root_section(seg, slope_start, resolution)
    if typ == "curve":
        return _curve(seg, slope_start, resolution)
    raise ValueError(f"Unknown segment type: {seg['type']}")

def _make_x(seg, resolution):
    n = int(round(seg["distance"] * resolution)) + 1
    return np.linspace(0.0, seg["distance"], n)

def _flat(seg, slope_start, resolution):
    x = _make_x(seg, resolution)
    z_flat = np.zeros_like(x)
    z = z_flat + seg["gradient"] * x
    slope_end = slope_start + seg["gradient"]
    return x, z, slope_end

def _bumpy(seg, slope_start, resolution):
    if "height" not in seg or "frequency" not in seg:
        raise ValueError("Bumpy segment requires 'height' and 'frequency'")
    x = _make_x(seg, resolution)
    z_flat = seg["height"] * np.sin(2*np.pi*seg["frequency"]*x)
    z = z_flat + seg["gradient"] * x
    slope_end = slope_start + seg["gradient"]
    return x, z, slope_end

def _jump(seg, slope_start, resolution):
    if "height" not in seg:
        raise ValueError("Jump segment requires 'height'")
    x = _make_x(seg, resolution)
    if "jump_length" in seg:
        jump_length = seg["jump_length"]
    elif "jump_inclination" in seg:
        jump_inclination_rad = np.deg2rad(seg["jump_inclination"])
        jump_length = seg["height"] / np.tan(jump_inclination_rad)
    else:
        jump_inclination_rad = np.deg2rad(10.0)
        jump_length = seg["height"] / np.tan(jump_inclination_rad)
    jump_length = min(jump_length, seg["distance"]*0.8)
    jump_start = (seg["distance"] - jump_length)/2.0
    jump_end = jump_start + jump_length
    z_flat = np.zeros_like(x)
    mask = (x >= jump_start) & (x <= jump_end)
    if np.any(mask):
        t = (x[mask]-jump_start)/jump_length
        smooth_t = 3*t**2 - 2*t**3
        z_flat[mask] = seg["height"] * smooth_t
    z = z_flat + seg["gradient"] * x
    slope_end = slope_start + seg["gradient"]
    return x, z, slope_end

def _drop(seg, slope_start, resolution):
    if "height" not in seg:
        raise ValueError("Drop segment requires 'height'")
    x = _make_x(seg, resolution)
    z_flat = np.zeros_like(x)
    drop_pos = seg["distance"]*0.5
    z_flat[x>=drop_pos] = -seg["height"]
    z = z_flat + seg["gradient"] * x
    slope_end = slope_start + seg["gradient"]
    return x, z, slope_end

def _staircase(seg, slope_start, resolution):
    for key in ("height","step_length","num_steps"):
        if key not in seg:
            raise ValueError("Staircase requires height, step_length, num_steps")
    x = _make_x(seg, resolution)
    total_len = seg["step_length"] * seg["num_steps"]
    start = (seg["distance"] - total_len)/2.0
    end = start + total_len
    step_h = seg["height"]/seg["num_steps"]
    z_flat = np.zeros_like(x)
    for i in range(1, seg["num_steps"]+1):
        s = start + (i-1)*seg["step_length"]
        e = start + i*seg["step_length"]
        mask = (x>=s) & (x<=e)
        z_flat[mask] = -i*step_h
    z_flat[x> end] = -seg["height"]
    z = z_flat + seg["gradient"] * x
    slope_end = slope_start + seg["gradient"]
    return x, z, slope_end

def _trapezium(seg, slope_start, resolution):
    if "height" not in seg or "flat_length" not in seg:
        raise ValueError("Trapezium requires height and flat_length")
    x = _make_x(seg, resolution)
    if "ramp_length" in seg:
        ramp_length = seg["ramp_length"]
    elif "ramp_inclination" in seg:
        ramp_length = seg["height"] / np.tan(np.deg2rad(seg["ramp_inclination"]))
    else:
        ramp_length = seg["height"] / np.tan(np.deg2rad(20.0))
    total_len = 2*ramp_length + seg["flat_length"]
    start = (seg["distance"] - total_len)/2.0
    up_s, up_e = start, start + ramp_length
    flat_s, flat_e = up_e, up_e + seg["flat_length"]
    down_s, down_e = flat_e, flat_e + ramp_length
    z_flat = np.zeros_like(x)
    # up
    mask = (x>=up_s) & (x<=up_e)
    if np.any(mask):
        t = (x[mask]-up_s)/ramp_length
        smooth_t = 3*t**2 - 2*t**3
        z_flat[mask] = seg["height"] * smooth_t
    # flat
    mask = (x>=flat_s) & (x<=flat_e)
    z_flat[mask] = seg["height"]
    # down
    mask = (x>=down_s) & (x<=down_e)
    if np.any(mask):
        t = (x[mask]-down_s)/ramp_length
        smooth_t = 3*t**2 - 2*t**3
        z_flat[mask] = seg["height"] * (1 - smooth_t)
    z = z_flat + seg["gradient"] * x
    slope_end = slope_start + seg["gradient"]
    return x, z, slope_end

def _gap_jump(seg, slope_start, resolution):
    if "height" not in seg or "gap_width" not in seg:
        raise ValueError("Gap jump requires height and gap_width")
    x = _make_x(seg, resolution)
    if "ramp_length" in seg:
        ramp_length = seg["ramp_length"]
    elif "ramp_inclination" in seg:
        ramp_length = seg["height"] / np.tan(np.deg2rad(seg["ramp_inclination"]))
    else:
        ramp_length = seg["height"] / np.tan(np.deg2rad(10.0))
    total_len = 2*ramp_length + seg["gap_width"]
    start = (seg["distance"] - total_len)/2.0
    up_s, up_e = start, start + ramp_length
    gap_s, gap_e = up_e, up_e + seg["gap_width"]
    down_s, down_e = gap_e, gap_e + ramp_length
    z_flat = np.zeros_like(x)
    # up
    mask = (x>=up_s) & (x<=up_e)
    if np.any(mask):
        t = (x[mask]-up_s)/ramp_length
        smooth_t = 3*t**2 - 2*t**3
        z_flat[mask] = seg["height"] * smooth_t
    # gap -> leave at 0
    # down
    mask = (x>=down_s) & (x<=down_e)
    if np.any(mask):
        t = (x[mask]-down_s)/ramp_length
        smooth_t = 3*t**2 - 2*t**3
        z_flat[mask] = seg["height"] * (1 - smooth_t)
    z = z_flat + seg["gradient"] * x
    slope_end = slope_start + seg["gradient"]
    return x, z, slope_end

def _rock_garden(seg, slope_start, resolution):
    if "height" not in seg or "rock_density" not in seg:
        raise ValueError("Rock garden requires height and rock_density")
    x = _make_x(seg, resolution)
    z_flat = np.zeros_like(x)
    num_rocks = int(round(seg["distance"] * seg["rock_density"]))
    if num_rocks > 0:
        rng = np.random.default_rng(12345)  # deterministic
        rock_positions = np.sort(rng.random(num_rocks) * seg["distance"])
        for rx in rock_positions:
            rock_h = seg["height"] * (0.3 + 0.7 * rng.random())
            rock_w = 0.2 + 0.3 * rng.random()
            mask = (x >= rx - rock_w/2.0) & (x <= rx + rock_w/2.0)
            if np.any(mask):
                xloc = x[mask] - rx
                t = xloc/(rock_w/2.0)
                rock_profile = rock_h * np.maximum(0.0, np.cos(np.pi * t / 2.0))
                z_flat[mask] = np.maximum(z_flat[mask], rock_profile)
    z = z_flat + seg["gradient"] * x
    slope_end = slope_start + seg["gradient"]
    return x, z, slope_end

def _root_section(seg, slope_start, resolution):
    if "height" not in seg or "root_density" not in seg:
        raise ValueError("Root section requires height and root_density")
    x = _make_x(seg, resolution)
    z_flat = np.zeros_like(x)
    num_roots = int(round(seg["distance"] * seg["root_density"]))
    if num_roots > 0:
        rng = np.random.default_rng(23456)  # deterministic
        root_positions = np.sort(rng.random(num_roots) * seg["distance"])
        for rx in root_positions:
            root_h = seg["height"] * (0.2 + 0.8 * rng.random())
            root_w = 0.1 + 0.2 * rng.random()
            root_ang = (rng.random() - 0.5) * np.pi/6.0
            mask = (x >= rx - root_w/2.0) & (x <= rx + root_w/2.0)
            if np.any(mask):
                xloc = x[mask] - rx
                t = xloc/(root_w/2.0)
                root_profile = root_h * np.maximum(0.0, np.cos(np.pi * t / 2.0))
                root_profile = root_profile + xloc * np.tan(root_ang)
                z_flat[mask] = np.maximum(z_flat[mask], root_profile)
    z = z_flat + seg["gradient"] * x
    slope_end = slope_start + seg["gradient"]
    return x, z, slope_end

def _curve(seg, slope_start, resolution):
    if "final_gradient" not in seg or "R_curve" not in seg:
        raise ValueError("Curve segment requires final_gradient and R_curve")
    d1 = seg["distance"]/2.0
    d2 = seg["distance"]/2.0
    X, Y = smooth_fillet(0.0, 0.0, d1, d2, seg["gradient"], seg["final_gradient"], seg["R_curve"])
    # Re-sample to match resolution roughly
    ds = 1.0 / resolution
    # cumulative distance along X (since we use X as course abscissa)
    # ensure X monotone increasing:
    if (X[-1] - X[0]) <= 0:
        # degenerate, fallback to straight
        x = _make_x(seg, resolution)
        z = seg["gradient"] * x
    else:
        x_target = np.arange(X[0], X[-1]+1e-9, ds)
        # simple interp
        z = np.interp(x_target, X, Y)
        x = x_target
    slope_end = seg["final_gradient"]
    return x, z, slope_end

def _dedup_join(X, Y):
    keep = [True]
    for k in range(1, len(X)):
        if abs(X[k]-X[k-1]) < 1e-12 and abs(Y[k]-Y[k-1]) < 1e-12:
            keep.append(False)
        else:
            keep.append(True)
    keep = np.array(keep, dtype=bool)
    return X[keep], Y[keep]
