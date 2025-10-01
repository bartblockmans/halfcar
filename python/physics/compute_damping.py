import numpy as np
from scipy.linalg import eigh

def compute_damping(P):
    """
    Port of ComputeDamping.m

    Returns
    -------
    csf, csr, ctf, ctr : float
        Suspension and tyre damping (front/rear) in N*s/m.
    """
    # Extract parameters
    ksf = P.ksf; ksr = P.ksr
    ktf = P.ktf; ktr = P.ktr
    mc  = P.ms
    muf = P.muf; mur = P.mur
    front_split = P.m_split_f

    zeta_body_F  = P.zeta_body_F
    zeta_wheel_F = P.zeta_wheel_F
    zeta_body_R  = P.zeta_body_R
    zeta_wheel_R = P.zeta_wheel_R

    # Effective sprung masses
    msf = mc * front_split
    msr = mc * (1.0 - front_split)

    csf, ctf = _quarter_car_c_from_zetas(msf, muf, ksf, ktf, zeta_body_F, zeta_wheel_F)
    csr, ctr = _quarter_car_c_from_zetas(msr, mur, ksr, ktr, zeta_body_R, zeta_wheel_R)

    # Optional: quick summary
    zF, wF = _quarter_car_zeta_from_c(msf, muf, ksf, ktf, csf, ctf)
    zR, wR = _quarter_car_zeta_from_c(msr, mur, ksr, ktr, csr, ctr)
    print(f"[ComputeDamping] Front:  cs={csf:.1f} Ns/m, ct={ctf:.1f} Ns/m | zeta=[{zF[0]:.3f}, {zF[1]:.3f}], f=[{wF[0]/(2*np.pi):.2f}, {wF[1]/(2*np.pi):.2f}] Hz")
    print(f"[ComputeDamping] Rear:   cs={csr:.1f} Ns/m, ct={ctr:.1f} Ns/m | zeta=[{zR[0]:.3f}, {zR[1]:.3f}], f=[{wR[0]/(2*np.pi):.2f}, {wR[1]/(2*np.pi):.2f}] Hz")
    return csf, csr, ctf, ctr

def _mk_mk(ms, mu, ks, kt):
    M = np.diag([ms, mu])
    K = np.array([[ ks,   -ks],
                  [-ks, ks+kt]], dtype=float)
    return M, K

def _quarter_car_c_from_zetas(ms, mu, ks, kt, zeta_body, zeta_wheel):
    M, K = _mk_mk(ms, mu, ks, kt)
    # Generalized symmetric EVP: K*phi = w^2*M*phi (same as MATLAB eig(K,M))
    w2, Phi = eigh(K, M)  # ascending by default
    omega = np.sqrt(np.maximum(w2, 0.0))
    # Mode shapes (columns) already correspond to ascending omega
    # Modal masses
    m1 = float(Phi[:,0].T @ M @ Phi[:,0])
    m2 = float(Phi[:,1].T @ M @ Phi[:,1])
    # a_i, b_i per mode
    a1 = float((Phi[0,0] - Phi[1,0])**2); b1 = float(Phi[1,0]**2)
    a2 = float((Phi[0,1] - Phi[1,1])**2); b2 = float(Phi[1,1]**2)
    d1 = 2*zeta_body*omega[0]*m1
    d2 = 2*zeta_wheel*omega[1]*m2
    A = np.array([[a1, b1],
                  [a2, b2]], dtype=float)
    rhs = np.array([d1, d2], dtype=float)
    x = np.linalg.solve(A, rhs)
    cs, ct = float(x[0]), float(x[1])
    # clip tiny negatives from round-off
    if cs < 0 and cs > -1e-9: cs = 0.0
    if ct < 0 and ct > -1e-9: ct = 0.0
    if cs < 0 or ct < 0:
        print(f"[ComputeDamping] WARNING: negative damping computed (cs={cs:.2f}, ct={ct:.2f}) — check targets/params.")
    return cs, ct

def _quarter_car_zeta_from_c(ms, mu, ks, kt, cs, ct):
    M, K = _mk_mk(ms, mu, ks, kt)
    C = np.array([[ cs,  -cs],
                  [-cs, cs+ct]], dtype=float)
    w2, Phi = eigh(K, M)
    omega = np.sqrt(np.maximum(w2, 0.0))
    m1 = float(Phi[:,0].T @ M @ Phi[:,0])
    m2 = float(Phi[:,1].T @ M @ Phi[:,1])
    d1 = float(Phi[:,0].T @ C @ Phi[:,0])
    d2 = float(Phi[:,1].T @ C @ Phi[:,1])
    z1 = d1/(2*omega[0]*m1) if omega[0] > 0 else 0.0
    z2 = d2/(2*omega[1]*m2) if omega[1] > 0 else 0.0
    return np.array([z1, z2], dtype=float), omega
