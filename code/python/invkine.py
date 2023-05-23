import math

def invkine(px, py):
    l2 = 107
    l3 = 107
    l4 = 107

    r = math.sqrt(px**2 + py**2)

    q1 = math.atan2(py, px)

    cp2 = ((r - l4)**2 + l2**2 - l3**2) / (2 * (r - l4) * l3)
    sp2 = math.sqrt(1 - cp2**2)
    p2 = math.atan2(sp2, cp2)
    q2 = math.pi/2 - p2

    cp3 = (l2**2 + l3**2 - (r - l4)**2) / (2 * l2 * l3)
    sp3 = math.sqrt(1 - cp3**2)
    p3 = math.atan2(sp3, cp3)
    q3 = math.pi - p3

    c4 = ((r - l4)**2 + l3**2 - l2**2) / (2 * (r - l4) * l3)
    s4 = math.sqrt(1 - c4**2)
    q4 = math.atan2(s4, c4)

    return q1, q2, q3, q4
