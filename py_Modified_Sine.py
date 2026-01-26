import numpy as np
import pandas as pd


""" 
The Problem : 

- Design the motion curves of a double-dwell cam to move a roller follower 50 mm.
- Choose suitable parameters to minimize velocities for both rise and fall.
- The total cycle must take 4 seconds.

Follower motion:
- Rise: 60 degrees
- Dwell: 120 degrees
- Fall: 30 degrees
- Dwell: 150 degrees 
"""

# Design parameters

h = 50.0        # lift (mm)
Rp = 50.0       # not sure what was that (mm)
Rf = 10.0       # Follower Radius (mm)

# Durations (Degree)

beta1 = 60.0    # rise 
beta2 = 120.0   # dwell 
beta3 = 30.0    # fall 
beta4 = 150.0   # dwell

# Selction for minmum velocity : Modified Sine

b = 0.25
d = 0.75
c = 0  

# Zone Limits in the modified sine 

x1 = b / 2
x2 = (1 - d) / 2
x3 = (1 + d) / 2
x4 = 1 - (b / 2)

# Peak acceleration factor

Ca = (4 * np.pi**2) / (
    (np.pi**2 - 8) * (b**2 - d**2)
    - 2 * np.pi * (np.pi - 2) * b
    + np.pi**2
)



# Normalized 5-Zone Motion Law

def norm_motion(x):

    if 0 <= x < x1:     # Zone 1
                
        y  = Ca * ((b/np.pi)*x - (b/np.pi)**2 * np.sin((np.pi/b)*x))
        yp = Ca * ((b/np.pi) - (b/np.pi)*np.cos((np.pi/b)*x))
        ypp = Ca * np.sin((np.pi/b)*x)
        yppp = Ca * (np.pi/b) * np.cos((np.pi/b)*x)

    elif x1 <= x < x2:      # Zone 2
            
        y  = Ca * (0.5*x**2 + b*(1/np.pi - 0.5)*x + b**2*(1/8 - 1/np.pi**2))
        yp = Ca * (x + b*(1/np.pi - 0.5))
        ypp = Ca
        yppp = 0.0

    elif x2 <= x < x3:      # Zone 3
        
        y  = Ca * (
            (b/np.pi + c/2)*x
            + (d/np.pi)**2
            + b**2*(1/8 - 1/np.pi**2)
            - ((1 - d)**2) / 8
            - (d/np.pi)**2 * np.cos((np.pi/d)*(x - (1 - d)/2))
        )
        yp = Ca * (
            b/np.pi + c/2
            + (d/np.pi) * np.sin((np.pi/d)*(x - (1 - d)/2))
        )
        ypp = Ca * np.cos((np.pi/d)*(x - (1 - d)/2))
        yppp = -Ca * (np.pi/d) * np.sin((np.pi/d)*(x - (1 - d)/2))

    elif x3 <= x < x4:      # Zone 4
        
        y  = Ca * (
            -0.5*x**2
            + (b/np.pi + 1 - b/2)*x
            + (2*d**2 - b**2)*(1/np.pi**2 - 1/8)
            - 0.5
        )
        yp = Ca * (-x + (b/np.pi + 1 - b/2))
        ypp = -Ca
        yppp = 0.0
    
    elif x4 <= x <= 1:      # Zone 5

        xr = x - 1.0
        y  = 1.0 + Ca * ((b/np.pi)*xr - (b/np.pi)**2 * np.sin((np.pi/b)*xr))
        yp = Ca * ((b/np.pi) - (b/np.pi)*np.cos((np.pi/b)*xr))
        ypp = Ca * np.sin((np.pi/b)*xr)
        yppp = Ca * (np.pi/b) * np.cos((np.pi/b)*xr)

    else:

        y = yp = ypp = yppp = 0.0

    return y, yp, ypp, yppp

def cam_motion(theta):

    theta_rad = np.radians(theta)
    b1_rad = np.radians(beta1)
    b2_rad = np.radians(beta2)
    b3_rad = np.radians(beta3)

    # rise
    if theta_rad < b1_rad:
        x = theta_rad / b1_rad
        y, yp, ypp, yppp = norm_motion(x)
        return h*y, (h/b1_rad)*yp, (h/b1_rad**2)*ypp, (h/b1_rad**3)*yppp, x

    # Dwell
    elif theta_rad < (b1_rad + b2_rad):
        
        return h, 0.0, 0.0, 0.0, 1.0

    # Fall
    elif theta_rad < (b1_rad + b2_rad + b3_rad):
        # Normalize x from 0 to 1 for the fall duration
        x = (theta_rad - b1_rad - b2_rad) / b3_rad
        y, yp, ypp, yppp = norm_motion(x)
        # Note: 1-y makes it go from h to 0
        return h*(1-y), -(h/b3_rad)*yp, -(h/b3_rad**2)*ypp, -(h/b3_rad**3)*yppp, x

    # Dwell
    else:
        return 0.0, 0.0, 0.0, 0.0, 0.0


def check_continuity():

    test_points = [x1 - 1e-6, x1, x1 + 1e-6, 
                   x2 - 1e-6, x2, x2 + 1e-6,
                   x3 - 1e-6, x3, x3 + 1e-6,
                   x4 - 1e-6, x4, x4 + 1e-6]
    
    print("Continuity check at zone boundaries:")
    for x in test_points:
        if 0 <= x <= 1:
            y, yp, ypp, yppp = norm_motion(x)
            print(f"x={x:.6f}: y={y:.6f}, yp={yp:.6f}, ypp={ypp:.6f}, yppp={yppp:.6f}")






# Data generation

rows = []

for theta in np.arange(0, 361, 1):
    s, v, a, j, x = cam_motion(theta)
    r = Rp + s
    th = np.radians(theta)

    rows.append({
        "Angle_deg": theta,
        "x_norm": round(x, 5),
        "s_mm": round(s, 5),
        "v_mm_per_deg": round(v, 6),
        "a_mm_per_deg2": round(a, 6),
        "j_mm_per_deg3": round(j, 6),   
        "Cam_X_mm": round((r - Rf)*np.cos(th), 5),
        "Cam_Y_mm": round((r - Rf)*np.sin(th), 5)
    })


df = pd.DataFrame(rows)
df.to_excel("cam_modified_sine.xlsx", index=False)

print("File:cam_modified_sine.xlsx")