import math as np

def invkine(px, py):
    l2 = 107
    l3 = 107
    l4 = 107

    r = np.sqrt(px**2 + py**2)

    q1 = np.atan2(py, px)

    cp2 = ((r - l4)**2 + l2**2 - l3**2) / (2 * (r - l4) * l2)
    sp2 = np.sqrt(1 - cp2**2)
    p2 = np.atan2(sp2, cp2)
    q2 = np.pi/2 - p2

    cp3 = (l2**2 + l3**2 - (r - l4)**2) / (2 * l2 * l3)
    sp3 = np.sqrt(1 - cp3**2)
    p3 = np.atan2(sp3, cp3)
    q3 = np.pi - p3

    c4 = ((r - l4)**2 + l3**2 - l2**2) / (2 * (r - l4) * l3)
    s4 = np.sqrt(1 - c4**2)
    q4 = np.atan2(s4, c4)

    # Adjust for articular limits and zeroes irl
    q1 = -np.pi/2 + q1 
    q2 = -q2
    q3 = -q3 + np.radians(-15)
    q4 = q4 + np.radians(15)

    return q1, q2, q3, q4
