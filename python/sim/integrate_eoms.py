import numpy as np
from scipy.integrate import solve_ivp
from physics.rhs import compute_accelerations

def _course_end_event(course_length):
    def event(t, y):
        return y[0] - course_length
    event.terminal = True
    event.direction = 1.0
    return event

def integrate_eoms(y0, P, road, Tend=np.inf, method="Radau", t_eval=None, rhs=None, rtol=1e-3, atol=1e-6, max_step=0.005, debug=False):
    if rhs is None:
        def rhs(t, y):
            return compute_accelerations(t, y, P, road, debug=debug, return_output=False)

    # Time span & evaluation grid
    if np.isinf(Tend):
        # Use a generous max time; event will stop integration
        t_span = (0.0, 1e6)
        events = [_course_end_event(road.x_course[-1] - road.x_course[0])]
    else:
        t_span = (0.0, float(Tend))
        events = None

    if t_eval is None:
        if np.isinf(Tend):
            # provide a coarse uniform grid; solver still uses internal steps
            t_eval = np.linspace(0.0, 60.0, 3001)  # 60s default window
        else:
            t_eval = np.linspace(0.0, Tend, int(max(201, Tend/0.02)))

    sol = solve_ivp(rhs, t_span, y0, method=method, t_eval=t_eval,
                    rtol=rtol, atol=atol, max_step=max_step, events=events, vectorized=False)
    if not sol.success:
        raise RuntimeError(sol.message)
    return sol.t, sol.y.T
