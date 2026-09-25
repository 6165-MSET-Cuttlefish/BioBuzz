"""Contour-gated ball-detection SnapScript for the Limelight 3A — finds Pollen and red/blue Nectar.

ball_detection_snapscript.py's HSV masks with its Hough stage replaced by contour work. Every colour
blob is first split with a distance transform: its deepest point is the centre of the largest circle
that fits, which is taken, blanked out, and repeated until what is left is too small for a ball. A
blob that splits into two or more circles is a group of touching balls. Otherwise it is one ball if
its area, bounding-box aspect (w / h) and fill (contour area / box area) are all in range, drawn as
its enclosing circle. There is no tracking and no overlap suppression; the types' hues don't overlap.

Upload this file from the Limelight web UI as a Python pipeline (Input tab, pipeline type Python) to
pipeline 4. Pipeline 0 is DECODE's AprilTags, pipeline 3 the Hough detector.

Areas are in pixels of a CALIBRATION_SIZE frame, so the gates hold at any streaming resolution.
H_ARRAY is the webcam's and must be recalibrated for the Limelight's lens with eocvsim/homography
before the inches mean anything.

llpython, 32 doubles, the same layout as ball_detection_snapscript.py but SCRIPT_ID 6166:
    0        SCRIPT_ID, so the hub can reject the wrong pipeline
    1        ball count N, 0 to MAX_BALLS
    2 + 3i   type code: 1 Pollen, 2 red Nectar, 3 blue Nectar
    3 + 3i   x, camera-frame ground inches
    4 + 3i   y, camera-frame ground inches
Balls are packed nearest first. The returned contour (tx/ty/ta) is the lowest-cost ball's circle,
where cost = |area / EXPECT_AREA_PX - 1| + FILL_COST_WEIGHT * (1 - fill); a split circle's fill is pi/4.
"""

import math
import time

import cv2
import numpy as np

SCRIPT_ID = 6166.0
MAX_BALLS = 10  # (32 llpython slots - 2 header) // 3 per ball

DISPLAY_MODE = "OVERLAY"  # "OVERLAY" for detections on the camera image, "MASK" to tune HSV gates
DRAW_OVERLAY = True

# The resolution H_ARRAY was calibrated at, not necessarily the one the Limelight streams.
CALIBRATION_SIZE = (640, 480)
H_ARRAY = (
    (-6.8658673540e-02, -1.4582606197e-02, 2.5633211718e+01),
    (-8.0700352292e-04, -2.1452449123e-01, 6.3092382406e+01),
    (-1.8726593163e-04, -7.7392061899e-03, 1.0000000000e+00),
)

MIN_AREA_PX = 350.0
MAX_AREA_PX = 16000.0
MIN_ASPECT = 0.71
MAX_ASPECT = 1.4
MIN_FILL = 0.5
MAX_FILL = 1.3

EXPECT_AREA_PX = 3000.0
FILL_COST_WEIGHT = 4.0

MAX_SPLIT_BALLS = 8  # per blob
SPLIT_SUPPRESS_SCALE = 0.7  # blanked radius / found radius; lower finds overlapping balls, higher fewer ghosts
SPLIT_MIN_OPEN_EDGE = 0.5  # fraction of a split circle's rim that must border background
EDGE_RING_GAP_PX = 3
EDGE_SAMPLES = 36

FPS_SMOOTHING = 0.1
FONT = cv2.FONT_HERSHEY_SIMPLEX
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
MAGENTA = (255, 0, 255)
CONTACT = (0, 0, 255)


class BallType(object):
    """Draw colours are BGR."""

    def __init__(self, code, label, hues, s, v, glare, draw):
        self.code = code
        self.label = label
        glare_s_high, glare_v_low = glare
        self.ranges = []
        for h_low, h_high in hues:
            self.ranges.append((np.array((h_low, s[0], v[0]), np.uint8),
                                np.array((h_high, s[1], v[1]), np.uint8)))
            self.ranges.append((np.array((h_low, 0, glare_v_low), np.uint8),
                                np.array((h_high, glare_s_high, 255), np.uint8)))
        self.draw = draw

    def mask(self, hsv):
        out = cv2.inRange(hsv, *self.ranges[0])
        for low, high in self.ranges[1:]:
            out = cv2.bitwise_or(out, cv2.inRange(hsv, low, high))
        return out


# Arguments: hue ranges, (sLow, sHigh), (vLow, vHigh), (glareSHigh, glareVLow). Each hue range also
# gets a glare band, since highlights wash out saturation and raise value but keep hue. Red wraps past 179.
BALL_TYPES = (
    BallType(1, "Pollen", ((15, 32),), (120, 255), (120, 255), (70, 200), (0, 255, 255)),
    BallType(2, "Red Nectar", ((0, 12), (165, 179)), (130, 255), (90, 255), (80, 190), (40, 40, 255)),
    BallType(3, "Blue Nectar", ((100, 125),), (130, 255), (75, 255), (100, 180), (255, 120, 40)),
)

_H = np.array(H_ARRAY, np.float64)
_H_INV = np.linalg.inv(_H)
NO_CONTOUR = np.array([[]])

# Module globals persist between frames; the FPS estimate lives here.
_fps = 0.0
_last_frame = None


def _candidate(ball, cx, cy, radius, area, fill):
    cost = abs(area / EXPECT_AREA_PX - 1.0) + FILL_COST_WEIGHT * (1.0 - fill)
    return ball, cx, cy, radius, area, fill, cost


def _single(ball, contour, raw_area, area_scale):
    _, _, w, h = cv2.boundingRect(contour)
    area = raw_area * area_scale
    aspect = float(w) / h
    fill = raw_area / float(w * h)
    if not (MIN_AREA_PX <= area <= MAX_AREA_PX and MIN_ASPECT <= aspect <= MAX_ASPECT
            and MIN_FILL <= fill <= MAX_FILL):
        return None
    (cx, cy), radius = cv2.minEnclosingCircle(contour)
    return _candidate(ball, cx, cy, radius, area, fill)


def _open_edge(blob, cx, cy, radius):
    """Most of a ball's rim borders background even in a row of touching balls, while a circle fitted
    inside a large same-coloured panel is hemmed in by the panel."""
    ring = radius + EDGE_RING_GAP_PX
    outside = 0
    for i in range(EDGE_SAMPLES):
        angle = 2.0 * math.pi * i / EDGE_SAMPLES
        px = int(round(cx + ring * math.cos(angle)))
        py = int(round(cy + ring * math.sin(angle)))
        if not (0 <= px < blob.shape[1] and 0 <= py < blob.shape[0]) or blob[py, px] == 0:
            outside += 1
    return float(outside) / EDGE_SAMPLES


def _split(ball, contour, area_scale):
    x, y, w, h = cv2.boundingRect(contour)
    # A zero border so blob pixels on the box edge still measure their distance to the edge.
    blob = np.zeros((h + 2, w + 2), np.uint8)
    cv2.drawContours(blob, [contour], -1, 255, -1, offset=(1 - x, 1 - y))
    dist = cv2.distanceTransform(blob, cv2.DIST_L2, 5)
    min_r = math.sqrt(MIN_AREA_PX / (math.pi * area_scale))

    found = []
    for _ in range(MAX_SPLIT_BALLS):
        _, radius, _, (px, py) = cv2.minMaxLoc(dist)
        if radius < min_r:
            break
        cv2.circle(dist, (px, py), int(math.ceil(radius * SPLIT_SUPPRESS_SCALE)), 0, -1)
        area = math.pi * radius * radius * area_scale
        if area > MAX_AREA_PX or _open_edge(blob, px, py, radius) < SPLIT_MIN_OPEN_EDGE:
            continue
        found.append(_candidate(ball, px + x - 1, py + y - 1, radius, area, math.pi / 4.0))
    return found


def _to_ground(balls, scale_x, scale_y):
    contacts = np.array([[[b[1] * scale_x, (b[2] + b[3]) * scale_y]] for b in balls], np.float32)
    return cv2.perspectiveTransform(contacts, _H).reshape(-1, 2)


def _label(image, text, origin, color, scale=0.5, thickness=1):
    (w, h), base = cv2.getTextSize(text, FONT, scale, thickness)
    x, y = origin
    cv2.rectangle(image, (x - 2, y - h - 2), (x + w + 2, y + base), BLACK, -1)
    cv2.putText(image, text, (x, y), FONT, scale, color, thickness, cv2.LINE_AA)


def _draw_overlay(image, balls, ground, order, best, scale_x, scale_y):
    for rank, i in enumerate(order):
        ball, x, y, radius, area, fill, _ = balls[i]
        center = (int(round(x)), int(round(y)))
        r = int(round(radius))
        cv2.circle(image, center, r, WHITE, 3 if i == best else 2)
        cv2.circle(image, center, 4, WHITE, -1)
        cv2.circle(image, (center[0], center[1] + r), 5, CONTACT, -1)
        _label(image, "%s%s%s (%.1f, %.1f)in a=%.0f f=%.2f"
               % ("* " if i == best else "", "" if rank < MAX_BALLS else "dropped ", ball.label,
                  ground[i][0], ground[i][1], area, fill),
               (center[0] + r + 6, center[1] + r), ball.draw)

    origin = cv2.perspectiveTransform(np.array([[[0.0, 0.0]]], np.float32), _H_INV).reshape(2)
    ox, oy = int(origin[0] / scale_x), int(origin[1] / scale_y)
    cv2.line(image, (ox - 10, oy), (ox + 10, oy), MAGENTA, 2)
    cv2.line(image, (ox, oy - 10), (ox, oy + 10), MAGENTA, 2)
    _label(image, "(0,0)", (ox + 14, oy), MAGENTA)

    _label(image, "%d balls, %d reported  %.0f fps"
           % (len(balls), min(len(balls), MAX_BALLS), _fps), (8, 18), WHITE)


def runPipeline(image, llrobot):
    global _fps, _last_frame

    now = time.monotonic()
    if _last_frame is not None and now > _last_frame:
        _fps += FPS_SMOOTHING * (1.0 / (now - _last_frame) - _fps)
    _last_frame = now

    height, width = image.shape[:2]
    scale_x = float(CALIBRATION_SIZE[0]) / width
    scale_y = float(CALIBRATION_SIZE[1]) / height
    area_scale = scale_x * scale_y

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_canvas = np.zeros(image.shape, np.uint8) if DISPLAY_MODE == "MASK" else None

    balls = []
    for ball in BALL_TYPES:
        color_mask = ball.mask(hsv)
        if mask_canvas is not None:
            mask_canvas[color_mask > 0] = ball.draw
        contours = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        for contour in contours:
            raw_area = cv2.contourArea(contour)
            if raw_area * area_scale < MIN_AREA_PX:
                continue
            circles = _split(ball, contour, area_scale)
            if len(circles) < 2:
                single = _single(ball, contour, raw_area, area_scale)
                if single is not None:
                    circles = [single]
            balls.extend(circles)

    ground = _to_ground(balls, scale_x, scale_y) if balls else []
    # Nearest first, so the ones the llpython budget drops are the far ones.
    order = sorted(range(len(balls)), key=lambda i: math.hypot(ground[i][0], ground[i][1]))
    best = min(range(len(balls)), key=lambda i: balls[i][6]) if balls else None

    llpython = [SCRIPT_ID, float(min(len(balls), MAX_BALLS))]
    for i in order[:MAX_BALLS]:
        llpython.extend((float(balls[i][0].code), float(ground[i][0]), float(ground[i][1])))

    if mask_canvas is not None:
        image = mask_canvas
    if DRAW_OVERLAY:
        _draw_overlay(image, balls, ground, order, best, scale_x, scale_y)

    if best is None:
        return NO_CONTOUR, image, llpython
    _, x, y, radius, _, _, _ = balls[best]
    circle = cv2.ellipse2Poly((int(round(x)), int(round(y))), (int(round(radius)), int(round(radius))),
                              0, 0, 360, 10)
    return circle.reshape(-1, 1, 2).astype(np.int32), image, llpython
