import numpy as np
import math

from robot import *

def forward_geometry(q1, q2, q3):
    # Constants
    tan30 = math.tan(math.radians(30))
    sin30 = math.sin(math.radians(30))
    tan60 = math.tan(math.radians(60))
    pi = math.pi

    # Intermediate calculations
    t = 0.5 * (EE_RADIUS - BASE_RADIUS) * tan30
    
    # Calculate positions
    y1 = -(t + BICEPS_LEN * math.cos(q1))
    z1 = -BICEPS_LEN * math.sin(q1)
    
    y2 = (t + BICEPS_LEN * math.cos(q2)) * sin30
    x2 = y2 * tan60
    z2 = -BICEPS_LEN * math.sin(q2)
    
    y3 = (t + BICEPS_LEN * math.cos(q3)) * sin30
    x3 = -y3 * tan60
    z3 = -BICEPS_LEN * math.sin(q3)
    
    # Calculate determinant
    dnm = (y2 - y1) * x3 - (y3 - y1) * x2
    
    # Coefficients for x, y
    w1 = y1 * y1 + z1 * z1
    w2 = x2 * x2 + y2 * y2 + z2 * z2
    w3 = x3 * x3 + y3 * y3 + z3 * z3
    
    a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
    b1 = - 0.5 *((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1))
    
    a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
    b2 = 0.5 * ((w2 - w1) * x3 - (w3 - w1) * x2)
    
    # Coefficients for quadratic equation a*z^2 + b*z + c = 0
    aV = a1 * a1 + a2 * a2 + dnm * dnm
    bV = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
    cV = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - FOREARM_LEN * FOREARM_LEN)
    
    # Discriminant
    dV = bV * bV - 4.0 * aV * cV
    
    if dV < 0.0:
        return None, None, None  # Non-existing solution, return error
    
    # Calculate z
    z = -0.5 * (bV + math.sqrt(dV)) / aV
    
    # Calculate x and y
    x = (a1 * z + b1) / dnm
    y = (a2 * z + b2) / dnm
    
    return x, y, z


