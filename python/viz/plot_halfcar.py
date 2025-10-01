import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from viz.plot_circle import plot_circle
from viz.plot_rectangle import plot_rectangle
from viz.plot_full_spring import plot_full_spring

def plot_halfcar(q, P, ax=None):
    """
    Plot detailed halfcar model matching MATLAB PlotHalfCar.m
    
    Parameters
    ----------
    q : array_like
        State vector [X, Y, theta, zF, zR]
    P : Params
        Parameter object
    ax : matplotlib.axes.Axes, optional
        Axes to plot on, by default None (creates new figure)
    """
    
    if ax is None:
        fig, ax = plt.subplots()
    
    # Unpack state
    X = q[0]  # Absolute X position in world axis frame of the chassis
    Y = q[1]  # Absolute Y position in world axis frame of the chassis
    theta = q[2]  # Pitch angle of the chassis
    zF = q[3]  # Absolute Y position in world axis frame of unsprung mass
    zR = q[4]  # Absolute Y position in world axis frame of unsprung mass
    
    # Unit vector
    e = np.array([np.cos(theta), np.sin(theta)])
    
    # Compute coordinates
    # -------------------------------------------------------------------------
    
    # Chassis pickup points 
    pRx = X - P.lr * e[0]
    pFx = X + P.lf * e[0]
    pRy = Y - P.lr * e[1]
    pFy = Y + P.lf * e[1]
    
    # Unsprung mass coordinates
    uFy = zF
    uFx = pFx + (pFy - zF) * np.tan(theta)
    uRy = zR
    uRx = pRx + (pRy - zR) * np.tan(theta)
    
    # Ground coordinates
    rRx = uRx + P.LtR * e[1]
    rFx = uFx + P.LtF * e[1]
    rRy = uRy - P.LtR * e[0]
    rFy = uFy - P.LtF * e[0]
    
    # Plot
    # -------------------------------------------------------------------------
    
    # Plot chassis
    plot_rectangle(pRx, pRy, pFx, pFy, P.hc, [0, 0, 0], [0.5, 0.5, 0.5], 1, ax)
    
    # Plot circles at connection points
    plot_circle(pRx, pRy, 0.025, 0, 2*np.pi, [0, 0, 0], 1, 0, 0, 50, ax)
    plot_circle(pFx, pFy, 0.025, 0, 2*np.pi, [0, 0, 0], 1, 0, 0, 50, ax)
    
    # Plot unsprung masses
    plot_circle(uRx, uRy, P.hu/2, 0, 2*np.pi, [0.5, 0.5, 0.5], 1, 0, 0, 50, ax)
    plot_circle(uFx, uFy, P.hu/2, 0, 2*np.pi, [0.5, 0.5, 0.5], 1, 0, 0, 50, ax)
    
    # Plot center unsprung mass
    plot_circle(uRx, uRy, 0.025, 0, 2*np.pi, [0, 0, 0], 1, 0, 0, 50, ax)
    plot_circle(uFx, uFy, 0.025, 0, 2*np.pi, [0, 0, 0], 1, 0, 0, 50, ax)
    
    # Plot ground points
    plot_circle(rRx, rRy, 0.025, 0, 2*np.pi, [0, 0, 0], 1, 0, 0, 50, ax)
    plot_circle(rFx, rFy, 0.025, 0, 2*np.pi, [0, 0, 0], 1, 0, 0, 50, ax)
    
    # Plot springs (reduced coil count for better performance)
    plot_full_spring(pRx, pRy, uRx, uRy, 0.1, 1/2, 3, [0, 0, 0], 1, ax)
    plot_full_spring(pFx, pFy, uFx, uFy, 0.1, 1/2, 3, [0, 0, 0], 1, ax)
    plot_full_spring(uRx, uRy, rRx, rRy, 0.1, 1/2, 2, [0, 0, 0], 1, ax)
    plot_full_spring(uFx, uFy, rFx, rFy, 0.1, 1/2, 2, [0, 0, 0], 1, ax)
    
    # Plot wheels
    plot_circle(uRx, uRy, P.LtR, 0, 2*np.pi, None, 1, 0, 0, 50, ax)
    plot_circle(uFx, uFy, P.LtF, 0, 2*np.pi, None, 1, 0, 0, 50, ax)