import math
import numpy as np

G = 386.09  # ShotSolver.GRAVITY_IN_PER_S2
W_OPEN = 20.0
SHOULDER = 7.61
H_OPEN = 14.0
BOTTOM_Z = 53.5
TILT = math.radians(30)
AIM_DEPTH = 3.0
RIM = 1.0
EXIT_Z = 14.0
POLLEN_D = 2.8


def centroid_w():
    rect = W_OPEN * SHOULDER
    tri = W_OPEN * (H_OPEN - SHOULDER) / 2
    tric = SHOULDER + (H_OPEN - SHOULDER) / 3
    return (rect * SHOULDER / 2 + tri * tric) / (rect + tri)


# Local frame: x across the opening (u), y horizontal INTO the cell, z up. Opening bottom centre at (0,0,BOTTOM_Z).
U = np.array([1.0, 0.0, 0.0])
Wv = np.array([0.0, math.sin(TILT), math.cos(TILT)])   # up the face, leaning back into the cell
N = np.array([0.0, -math.cos(TILT), math.sin(TILT)])  # out of the opening, 30 deg up
O = np.array([0.0, 0.0, BOTTOM_Z])
CW = centroid_w()
AIM = O + CW * Wv - AIM_DEPTH * N


def inside(u, w, margin):
    half = W_OPEN / 2
    if w < margin or abs(u) > half - margin:
        return False
    rise = H_OPEN - SHOULDER
    slant = math.hypot(half, rise)
    below = ((H_OPEN - w) * half - abs(u) * rise) / slant
    return below >= margin


def edge_slack(u, w, margin):
    """min signed distance to an edge minus margin (positive = inside with slack)."""
    half = W_OPEN / 2
    rise = H_OPEN - SHOULDER
    slant = math.hypot(half, rise)
    below = ((H_OPEN - w) * half - abs(u) * rise) / slant
    return min(w - margin, half - abs(u) - margin, below - margin)
