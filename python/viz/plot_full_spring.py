import numpy as np
import matplotlib.pyplot as plt
from viz.spring import Spring

def plot_full_spring(x1, y1, x2, y2, R, RATIO, nc, color='k', LW=1, ax=None):
    """
    Plot full spring matching MATLAB PlotFullSpring.m
    
    Parameters
    ----------
    x1, y1 : float
        Start point coordinates
    x2, y2 : float
        End point coordinates
    R : float
        Spring radius
    RATIO : float
        Spring length ratio
    nc : int
        Number of coils
    color : str or array_like, optional
        Color specification, by default 'k'
    LW : float, optional
        Line width, by default 1
    ax : matplotlib.axes.Axes, optional
        Axes to plot on, by default None (creates new figure)
    """
    
    if ax is None:
        fig, ax = plt.subplots()
    
    # Create an instance of spring
    spr = Spring(R, nc)
    
    # Direction vector
    n = np.array([x2, y2]) - np.array([x1, y1])
    
    # Coordinates
    p1 = np.array([x1, y1])
    p2 = np.array([x1 + 0.5 * (1-RATIO) * n[0], y1 + 0.5 * (1-RATIO) * n[1]])
    p3 = np.array([x2 - 0.5 * (1-RATIO) * n[0], y2 - 0.5 * (1-RATIO) * n[1]])
    p4 = np.array([x2, y2])
    
    # Get coordinates of spring
    x_s, y_s = spr.get_spr(p2, p3)
    
    # Plot spring
    ax.plot(x_s, y_s, color=color, linewidth=LW)
    
    # Plot legs
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=color, linewidth=LW)
    ax.plot([p3[0], p4[0]], [p3[1], p4[1]], color=color, linewidth=LW)