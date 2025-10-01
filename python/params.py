from dataclasses import dataclass, field

@dataclass
class Params:
    # Mass & inertia
    ms: float = 80.0
    m_split_f: float = 0.4
    Is: float = 24.0
    muf: float = 3.5
    mur: float = 4.0

    # Aero & rolling
    CdA: float = 0.350
    rho: float = 1.225
    Crr: float = 0.010

    # Drivetrain
    Tmax: float = 80.0

    # Geometry
    lf: float = 0.60
    lr: float = 0.60

    # Free lengths (visual)
    LtF: float = 0.35
    LtR: float = 0.35
    LsF: float = 0.45
    LsR: float = 0.45

    # Plot geometry (heights)
    hc: float = 0.15
    hu: float = 0.15

    # Stiffness
    ksf: float = 18e3
    ksr: float = 10e3
    ktf: float = 120e3
    ktr: float = 140e3

    # Damping ratios (used to compute c* below)
    zeta_body_F: float = 0.25
    zeta_wheel_F: float = 0.50
    zeta_body_R: float = 0.25
    zeta_wheel_R: float = 0.50

    # Damping (to be computed by ComputeDamping)
    csf: float = 0.0
    csr: float = 0.0
    ctf: float = 0.0
    ctr: float = 0.0

    # Gravity (MATLAB code uses negative for down)
    g: float = -9.81

    # Options
    use_n_dot: bool = False
    contact: str = "unilateral"   # 'unilateral' or 'bilateral'
    contact_method: str = "circle" # 'point' or 'circle'

    # Derived/extra
    theta_sag: float = 0.0
