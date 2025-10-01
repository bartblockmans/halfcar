import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

def plot_rectangle(x1, y1, x2, y2, w, line_color='k', face_color='w', line_width=1.5, ax=None):
    """
    Plot rectangle matching MATLAB PlotRectangle.m
    
    Parameters
    ----------
    x1, y1 : float
        Center of the bottom edge
    x2, y2 : float
        Center of the top edge
    w : float
        Width (perpendicular to vector from (x1,y1) to (x2,y2))
    line_color : str or array_like, optional
        Edge color, by default 'k'
    face_color : str or array_like, optional
        Face color, by default 'w'
    line_width : float, optional
        Edge thickness, by default 1.5
    ax : matplotlib.axes.Axes, optional
        Axes to plot on, by default None (creates new figure)
        
    Returns
    -------
    matplotlib.patches.Polygon
        The rectangle patch object
    """
    
    if ax is None:
        fig, ax = plt.subplots()
    
    # Endpoints
    p1 = np.array([x1, y1])
    p2 = np.array([x2, y2])
    
    # Direction and right-hand normal
    v = p2 - p1
    L = np.linalg.norm(v)
    if L <= np.finfo(float).eps:
        raise ValueError('Points (x1,y1) and (x2,y2) must be distinct.')
    d = v / L
    n = np.array([d[1], -d[0]])  # right-hand normal
    
    # Corners (counter-clockwise: bottom-left, bottom-right, top-right, top-left)
    bl = p1 - (w/2) * n
    br = p1 + (w/2) * n
    tr = p2 + (w/2) * n
    tl = p2 - (w/2) * n
    
    # Create polygon
    corners = np.array([bl, br, tr, tl])
    
    # Draw
    rect = Polygon(corners, closed=True, facecolor=face_color, 
                   edgecolor=line_color, linewidth=line_width)
    ax.add_patch(rect)
    
    return rect