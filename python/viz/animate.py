import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle
from viz.plot_halfcar import plot_halfcar
from viz.spring import Spring

def animate_results(t, y, P, road, nf=500, size_factor=2, speed=1):
    """
    Create detailed animation of the halfcar simulation matching MATLAB style.
    
    Parameters
    ----------
    t : array_like
        Time vector
    y : array_like
        State vector [X, Y, theta, zuf, zur, Xd, Yd, thd, zdf, zdr]
    P : Params
        Parameter object
    road : Road
        Road object
    nf : int, optional
        Number of frames, by default 500
    size_factor : int, optional
        Size factor for road display, by default 2
    speed : int, optional
        Animation speed multiplier, by default 1
    """
    
    # Create full screen figure
    fig = plt.figure(figsize=(16, 9))
    ax = fig.add_subplot(111)
    
    # Animation time
    t_anim = np.linspace(0, t[-1], nf)
    
    # Interpolate state vector for animation
    y_interp = np.zeros((nf, y.shape[1]))
    for i in range(y.shape[1]):
        y_interp[:, i] = np.interp(t_anim, t, y[:, i])
    
    # Initialize plot elements
    ax.set_aspect('equal')
    ax.set_xlim(0, 10)  # Will be updated dynamically
    ax.set_ylim(0, 10)  # Will be updated dynamically
    
    # Remove axis ticks and labels
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    
    # Add title
    ax.set_title('Animation of the downhill run', fontsize=14, fontweight='bold')
    
    # Initialize text for time, distance, and velocity
    info_text = ax.text(0.95, 0.95, '', transform=ax.transAxes, 
                       horizontalalignment='right', verticalalignment='top',
                       fontsize=12, fontweight='bold',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='black'))
    
    # Simple square view centered on chassis position
    view_size = 2.0  # Half-width of the square view
    
    def animate_frame(frame):
        """Update animation for each frame"""
        # Clear the plot
        ax.clear()
        
        # Get current state
        y_i = y_interp[frame]
        X = y_i[0]  # Chassis X position
        Y = y_i[1]  # Chassis Y position
        
        # Calculate road coordinates around current position
        x_road = np.linspace(X - view_size, X + view_size, 200)
        y_road = road.h(x_road)
        
        # Plot road
        ax.plot(x_road, y_road, 'k', linewidth=3)
        
        # Plot halfcar
        plot_halfcar(y_i[:5], P, ax)
        
        # Set simple square axis limits centered on chassis
        ax.set_xlim(X - view_size, X + view_size)
        ax.set_ylim(Y - view_size, Y + view_size)
        
        # Remove axis ticks and labels
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        
        # Add title
        ax.set_title('Animation of the downhill run', fontsize=14, fontweight='bold')
        
        # Calculate and display info
        time_str = f't = {t_anim[frame]:.2f} s'
        distance_str = f'd = {y_i[0]:.0f} m'
        velocity = np.sqrt(y_i[5]**2 + y_i[6]**2) * 3.6  # Convert m/s to km/h
        velocity_str = f'V = {velocity:.1f} km/h'
        
        # Update info text
        info_text = ax.text(0.95, 0.95, f'{time_str}\n{distance_str}\n{velocity_str}', 
                           transform=ax.transAxes,
                           horizontalalignment='right', verticalalignment='top',
                           fontsize=12, fontweight='bold',
                           bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='black'))
        
        return ax.collections + ax.patches + [info_text]
    
    # Create animation with optimized settings
    anim = animation.FuncAnimation(fig, animate_frame, frames=nf, 
                                 interval=max(10, 30//speed), blit=False, repeat=True)
    
    plt.tight_layout()
    plt.show()
    
    return anim