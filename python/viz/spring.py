import numpy as np

class Spring:
    """
    Spring class matching MATLAB Spring.m for generating 2D spring coordinates.
    """
    
    def __init__(self, R, num_coils):
        """
        Initialize spring with radius and number of coils.
        
        Parameters
        ----------
        R : float
            Spring radius
        num_coils : int
            Number of coils
        """
        self.spr0_x, self.spr0_y, self.spr0_len = self._spr_init(R, num_coils)
    
    def get_spr(self, pt1, pt2):
        """
        Get coordinates of transformed spring connecting two points.
        
        Parameters
        ----------
        pt1 : array_like
            Start point [x, y]
        pt2 : array_like
            End point [x, y]
            
        Returns
        -------
        x, y : array_like
            Spring coordinates
        """
        # Scale
        scale = np.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2) / self.spr0_len
        x = self.spr0_x.copy()
        y = self.spr0_y * scale
        
        # Convert to polar coordinates
        theta, r = self._cart2pol(x, y)
        
        # Rotate
        alpha = np.pi / 2 - np.arctan((pt2[1] - pt1[1]) / (pt2[0] - pt1[0]))
        if pt1[0] > pt2[0]:
            alpha = alpha + np.pi
        theta = theta - alpha
        x, y = self._pol2cart(theta, r)
        
        # Move
        x = x + pt1[0]
        y = y + pt1[1]
        
        return x, y
    
    def _spr_init(self, R, n):
        """
        Create initial spring geometry.
        
        Parameters
        ----------
        R : float
            Spring radius
        n : int
            Number of coils
            
        Returns
        -------
        x, y : array_like
            Spring coordinates
        len : float
            Spring length
        """
        r = 0.7 * R
        step = 2 * (R - r)
        
        ang_R = np.linspace(3 * np.pi / 2, np.pi / 2, 20)
        ang_r = np.linspace(np.pi / 2, -np.pi / 2, 20)
        
        x = []
        y = []
        
        c_R = 0
        
        for i in range(n):
            c_R = c_R + step
            R_x = R * np.cos(ang_R)
            R_y = c_R + R * np.sin(ang_R)
            
            c_r = c_R + R - r
            r_x = R * np.cos(ang_r)
            r_y = c_r + r * np.sin(ang_r)
            
            x.extend(R_x.tolist())
            x.extend(r_x.tolist())
            y.extend(R_y.tolist())
            y.extend(r_y.tolist())
            
            if i == n - 1:
                c_R = c_R + step
                R_x = R * np.cos(ang_R)
                R_y = c_R + R * np.sin(ang_R)
                
                # Add straight lines at the spring's two ends
                x = [0] + x + R_x.tolist() + [0]
                y = [y[0] - R] + y + R_y.tolist() + [y[-1] + 3 * R]
                
                # Move the spring's start point to origin
                y = np.array(y) - np.min(y)
                break
        
        x = np.array(x)
        y = np.array(y)
        length = abs(y[-1] - y[0])
        
        return x, y, length
    
    def _cart2pol(self, x, y):
        """Convert Cartesian to polar coordinates."""
        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x)
        return theta, r
    
    def _pol2cart(self, theta, r):
        """Convert polar to Cartesian coordinates."""
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return x, y