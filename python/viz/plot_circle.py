import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def plot_circle(x, y, R, ang0=0, ang1=2*np.pi, color='k', LW=1.5, 
                plot_center=0, elim0=0, n_points=1000, ax=None):
    """
    Plot circle matching MATLAB PlotCircle.m
    
    Parameters
    ----------
    x, y : float
        Circle center coordinates
    R : float
        Circle radius
    ang0, ang1 : float, optional
        Start and end angles in radians, by default 0 and 2*pi
    color : str or array_like, optional
        Color specification, by default 'k'
    LW : float, optional
        Line width, by default 1.5
    plot_center : int, optional
        Whether to plot center point, by default 0
    elim0 : int, optional
        Whether to eliminate points with y < 0, by default 0
    n_points : int, optional
        Number of points for circle, by default 1000
    ax : matplotlib.axes.Axes, optional
        Axes to plot on, by default None (creates new figure)
    """
    
    if ax is None:
        fig, ax = plt.subplots()
    
    # Angle parameter
    ang = np.linspace(ang0, ang1, n_points)
    
    # Circle coordinates
    x_circle = R * np.cos(ang)
    y_circle = R * np.sin(ang)
    
    # Add origin
    x_c = x + x_circle
    y_c = y + y_circle
    
    # Eliminate all points that have y-values smaller than 0
    if elim0:
        x_c[y_c < 0] = np.nan
        y_c[y_c < 0] = np.nan
    
    # Plot
    if color is None:
        ax.plot(x_c, y_c, color=[0, 0, 0], linewidth=LW)
    else:
        ax.fill(x_c, y_c, color=color, linewidth=LW)
    
    if plot_center:
        ax.scatter(x, y, s=100, c=color, marker='o')