import numpy as np
import matplotlib.pyplot as plt
from params import Params
from road.road import Road
from road.select_course import select_course
from physics.compute_damping import compute_damping
from physics.static_sag import static_sag
from road.power_data import power_data
from sim.integrate_eoms import integrate_eoms
from post.compute_results import compute_results
from viz.plots import plot_states
from viz.animate import animate_results

def main():
    P = Params()
    # Derived length
    P.L0 = P.lf + P.lr

    # Compute damping from zeta settings (stub)
    csf, csr, ctf, ctr = compute_damping(P)
    P.csf, P.csr, P.ctf, P.ctr = csf, csr, ctf, ctr

    # Initial position and body geometry (match MATLAB intent)
    q = np.zeros(5)
    q[0] = P.lf
    q[1] = 0.5*(P.LtF + P.LsF + P.LtR + P.LsR)

    # ---- ROAD ----
    x_course, z_course, _, course_info = select_course(course="extreme", resolution=10, preview=False)
    road = Road.from_samples(x_course, z_course)

    # Plot road quick-look
    fig = plt.figure(); ax = fig.add_subplot(111)
    ax.plot(x_course, road.h(x_course))
    ax.fill_between(x_course, road.h(x_course), road.h(x_course).min()-5, alpha=0.3, color='gray')
    ax.set_aspect('equal', adjustable='box')
    ax.set_title('Road profile')
    ax.grid(True)

    # ---- PACING PLAN ----
    road.P, road.T_max = power_data(P, road)

    # ---- INITIAL SPEED ALONG ROAD ----
    U0 = 1.0
    X0 = P.lr
    s0 = road.hpC(X0)
    t0 = np.array([1.0, s0]) / np.sqrt(1.0 + s0**2)
    v0_world = U0 * t0
    th0 = np.arctan(s0)

    # ---- STATIC SAG ----
    zuf0, zur0 = 0.0, 0.0
    zuf0, zur0, Y0, etaF, etaR, deltaF, deltaR = static_sag(P, X0, th0, road)
    P.theta_sag = -np.arctan2((zur0 - zuf0), (P.lf + P.lr))

    q0 = np.array([X0, Y0, th0, zuf0, zur0])
    v0 = np.array([v0_world[0], v0_world[1], 0.0, 0.0, 0.0])
    y0 = np.concatenate([q0, v0])

    print('Initial conditions:')
    print('q0 =', q0)
    print('v0 =', v0)
    print('Contact mode:', P.contact)
    print(f'Front suspension compression: {etaF*1000:.2f} mm')
    print(f'Rear suspension compression: {etaR*1000:.2f} mm')

    # ---- INTEGRATE EOMs (stub RHS) ----
    # NOTE: integrate_eoms currently uses a zero RHS.
    # Once we port your RHS, the dynamics will be non-trivial.
    t_eval = np.linspace(0.0, 10.0, 1201)  # 10 s
    t, y = integrate_eoms(y0, P, road, Tend=np.inf, method="Radau", t_eval=t_eval)

    # ---- RESULTS ----
    results = compute_results(t, y, P, road)

    # ---- PLOTS ----
    plot_states(t, y, title_prefix=f"5-DOF model (contact={P.contact})")

    # ---- ANIMATION ----
    animate_results(t, y, P, road, nf=500, speed=2)

if __name__ == "__main__":
    main()