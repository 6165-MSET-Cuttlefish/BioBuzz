"""Ball-detection SnapScript for the Limelight 3A — finds Pollen and red/blue Nectar.

A port of modules/vision/BallDetectionPipeline. Each circle's ground contact point (cx, cy + r) goes
through a homography to camera-frame ground inches, so the numbers mean what BallDetection's
cameraX/cameraY mean and BallFieldTransform can make them field-relative. There is no tracking here:
every frame's detections stand alone, with no ids and no velocities.

Upload this file from the Limelight web UI as a Python pipeline (Input tab, pipeline type Python) to
pipeline 3. Pipeline 0 is DECODE's AprilTags, and only one pipeline runs at a time.

H_ARRAY is the webcam's, copied from BallVisionConstants, and must be recalibrated for the
Limelight's lens with eocvsim/homography before the inches mean anything. Contact points are scaled
to CALIBRATION_SIZE first, so the Limelight's streaming resolution need not match the calibration's.

llpython, 32 doubles:
    0        SCRIPT_ID, so the hub can reject the wrong pipeline
    1        ball count N, 0 to MAX_BALLS
    2 + 3i   type code: 1 Pollen, 2 red Nectar, 3 blue Nectar
    3 + 3i   x, camera-frame ground inches
    4 + 3i   y, camera-frame ground inches
Balls are packed nearest first, so a full frame drops the farthest and keeps the reachable ones.
"""

import math
import time

import cv2
import numpy as np

SCRIPT_ID = 6165.0
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

# BallVisionConstants.Detection
HOUGH_DP = 1.2
HOUGH_CANNY = 80.0
HOUGH_ACCUMULATOR = 22.0
MIN_RADIUS_FRAME_FRACTION = 0.02
MAX_RADIUS_FRAME_FRACTION = 0.10
MIN_COLOR_FILL = 0.30
MIN_CENTER_SEPARATION = 0.7
MASK_CIRCLE_MIN_FILL = 0.60

# BallVisionConstants, structural
REFERENCE_BALL_DIAMETER_IN = 2.8
DETECTION_SCALE = 0.5
ROI_CLOSE_KERNEL_SIZE = (9, 9)
ROI_PAD_PX = 14
ROI_MERGE_DIST_PX = 20
HOUGH_BLUR_KERNEL = (5, 5)
HOUGH_CANNY_MIN_THRESHOLD = 30.0
HOUGH_MAX_RADIUS_FRACTION = 0.60
HOUGH_MIN_DIST_FRACTION = 0.5
HOUGH_WORKING_MIN_RADIUS_PX = 6.0
ROI_MAX_UPSCALE = 4.0

FPS_SMOOTHING = 0.1
FONT = cv2.FONT_HERSHEY_SIMPLEX
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
MAGENTA = (255, 0, 255)
CONTACT = (0, 0, 255)


class BallType(object):
    """Draw colours are BGR here; the Java Scalars are RGB because EasyOpenCV hands out RGB."""

    def __init__(self, code, label, diameter_in, hues, s, v, glare, draw):
        self.code = code
        self.label = label
        self.radius_scale = diameter_in / REFERENCE_BALL_DIAMETER_IN
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
    BallType(1, "Pollen", 2.8, ((15, 32),), (120, 255), (120, 255), (70, 200), (0, 255, 255)),
    BallType(2, "Red Nectar", 3.6, ((0, 12), (165, 179)), (130, 255), (90, 255), (80, 190), (40, 40, 255)),
    BallType(3, "Blue Nectar", 3.6, ((100, 125),), (130, 255), (75, 255), (100, 180), (255, 120, 40)),
)

_CLOSE_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, ROI_CLOSE_KERNEL_SIZE)
_H = np.array(H_ARRAY, np.float64)
_H_INV = np.linalg.inv(_H)
NO_CONTOUR = np.array([[]])

# Module globals persist between frames; the FPS estimate lives here.
_fps = 0.0
_last_frame = None


def _fill_fraction(mask, x, y, radius):
    """Mask fill over the frame-clipped bounding box, which a circle covers pi/4 of.

    Rejects the phantom centers HOUGH_GRADIENT votes one radius outside real edges, and assigns type.
    """
    x0 = max(0, int(round(x - radius)))
    y0 = max(0, int(round(y - radius)))
    x1 = min(mask.shape[1], int(round(x + radius)))
    y1 = min(mask.shape[0], int(round(y + radius)))
    if x1 - x0 < 1 or y1 - y0 < 1:
        return 0.0
    box = mask[y0:y1, x0:x1]
    return float(np.count_nonzero(box)) / ((math.pi / 4.0) * box.shape[0] * box.shape[1])


def _is_near(a, b):
    d = ROI_MERGE_DIST_PX
    return (a[0] - d < b[0] + b[2] and a[0] + a[2] + d > b[0]
            and a[1] - d < b[1] + b[3] and a[1] + a[3] + d > b[1])


def _union(a, b):
    x = min(a[0], b[0])
    y = min(a[1], b[1])
    return x, y, max(a[0] + a[2], b[0] + b[2]) - x, max(a[1] + a[3], b[1] + b[3]) - y


def _merge_nearby(rects):
    merged = []
    consumed = [False] * len(rects)
    for i in range(len(rects)):
        if consumed[i]:
            continue
        current = rects[i]
        consumed[i] = True
        growing = True
        while growing:
            growing = False
            for j in range(len(rects)):
                if consumed[j] or not _is_near(current, rects[j]):
                    continue
                current = _union(current, rects[j])
                consumed[j] = True
                growing = True
        merged.append(current)
    return merged


def _add_mask_circle(ball, contour, color_mask, min_r, max_r, out):
    """Fallback for frames Hough misses: its accumulator is a hard threshold, so a borderline ball
    flickers in and out even when its mask is steady."""
    (cx, cy), radius = cv2.minEnclosingCircle(contour)
    if radius < min_r or radius > max_r:
        return
    if cv2.contourArea(contour) < MASK_CIRCLE_MIN_FILL * math.pi * radius * radius:
        return
    fill = _fill_fraction(color_mask, cx, cy, radius)
    if fill >= MIN_COLOR_FILL:
        out.append((ball, cx, cy, radius, fill))


def _search_regions(ball, roi_mask, color_mask, min_r, max_r, mask_circles):
    """Also collects round-enough mask blobs into mask_circles as Hough fallbacks."""
    contours = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    padded = []
    for contour in contours:
        _add_mask_circle(ball, contour, color_mask, min_r, max_r, mask_circles)
        x, y, w, h = cv2.boundingRect(contour)
        x0 = max(0, x - ROI_PAD_PX)
        y0 = max(0, y - ROI_PAD_PX)
        w = min(roi_mask.shape[1] - x0, w + ROI_PAD_PX * 2)
        h = min(roi_mask.shape[0] - y0, h + ROI_PAD_PX * 2)
        if w > 0 and h > 0:
            padded.append((x0, y0, w, h))
    return _merge_nearby(padded)


def _hough_circles(ball, gray, color_mask, regions, min_r, max_r):
    # Never derived from a region's size: a bounding box wandering a pixel would change upscale and
    # Canny frame to frame and make detections blink.
    min_dist = max(4.0, min_r * HOUGH_MIN_DIST_FRACTION * 2.0)
    upscale = min(ROI_MAX_UPSCALE, max(1.0, HOUGH_WORKING_MIN_RADIUS_PX / min_r))
    # Upscaling spreads each edge over `upscale` pixels, cutting gradient magnitude by that factor.
    canny = max(HOUGH_CANNY_MIN_THRESHOLD, HOUGH_CANNY / upscale)

    found = []
    for x, y, w, h in regions:
        short_side = min(w, h)
        if short_side < 4:
            continue
        # Only this ceiling may follow the region; the parameters above must not.
        region_max_r = min(short_side * HOUGH_MAX_RADIUS_FRACTION, max_r)
        if region_max_r <= min_r:
            continue

        work = gray[y:y + h, x:x + w]
        if upscale > 1.0:
            work = cv2.resize(work, (int(round(w * upscale)), int(round(h * upscale))),
                              interpolation=cv2.INTER_LINEAR)
        work = cv2.GaussianBlur(work, HOUGH_BLUR_KERNEL, 0)

        circles = cv2.HoughCircles(work, cv2.HOUGH_GRADIENT, HOUGH_DP, min_dist * upscale,
                                   param1=canny, param2=HOUGH_ACCUMULATOR,
                                   minRadius=int(min_r * upscale),
                                   maxRadius=int(region_max_r * upscale))
        if circles is None:
            continue
        for cx, cy, radius in circles[0]:
            cx = cx / upscale + x
            cy = cy / upscale + y
            radius = radius / upscale
            fill = _fill_fraction(color_mask, cx, cy, radius)
            if fill >= MIN_COLOR_FILL:
                found.append((ball, cx, cy, radius, fill))
    return found


def _unclaimed(mask_circles, hough):
    if not mask_circles or not hough:
        return mask_circles
    return [m for m in mask_circles
            if not any(math.hypot(m[1] - f[1], m[2] - f[2]) <= m[3] for f in hough)]


def _suppress_overlaps(candidates):
    """Largest-first NMS across all types: holes and glare fit smaller circles inside a ball, and the
    types' overlapping glare bands can find one ball under two colours."""
    # Whole-pixel radii so sub-pixel Hough jitter ties and the steadier colour fill decides.
    candidates.sort(key=lambda c: (-int(round(c[3])), -c[4]))

    kept = []
    for candidate in candidates:
        overlaps = False
        for k in kept:
            distance = math.hypot(candidate[1] - k[1], candidate[2] - k[2])
            # Second test catches rim artifacts; touching balls sit at 1.0, so
            # MIN_CENTER_SEPARATION must stay well under that.
            if distance <= k[3] or distance < MIN_CENTER_SEPARATION * (k[3] + candidate[3]):
                overlaps = True
                break
        if not overlaps:
            kept.append(candidate)
    return kept


def _to_ground(candidates, scale_x, scale_y):
    contacts = np.array([[[candidate[1] / DETECTION_SCALE * scale_x,
                           (candidate[2] + candidate[3]) / DETECTION_SCALE * scale_y]]
                         for candidate in candidates], np.float32)
    return cv2.perspectiveTransform(contacts, _H).reshape(-1, 2)


def _label(image, text, origin, color, scale=0.5, thickness=1):
    (w, h), base = cv2.getTextSize(text, FONT, scale, thickness)
    x, y = origin
    cv2.rectangle(image, (x - 2, y - h - 2), (x + w + 2, y + base), BLACK, -1)
    cv2.putText(image, text, (x, y), FONT, scale, color, thickness, cv2.LINE_AA)


def _draw_overlay(image, balls, ground, order, region_count, scale_x, scale_y):
    up = 1.0 / DETECTION_SCALE
    for rank, i in enumerate(order):
        ball, x, y, radius, _ = balls[i]
        center = (int(x * up), int(y * up))
        r = int(radius * up)
        cv2.circle(image, center, r, WHITE, 2)
        cv2.circle(image, center, 4, WHITE, -1)
        cv2.circle(image, (center[0], center[1] + r), 5, CONTACT, -1)
        _label(image, "%s%s (%.1f, %.1f)in"
               % ("" if rank < MAX_BALLS else "dropped ", ball.label, ground[i][0], ground[i][1]),
               (center[0] + r + 6, center[1] + r), ball.draw)

    origin = cv2.perspectiveTransform(np.array([[[0.0, 0.0]]], np.float32), _H_INV).reshape(2)
    ox, oy = int(origin[0] / scale_x), int(origin[1] / scale_y)
    cv2.line(image, (ox - 10, oy), (ox + 10, oy), MAGENTA, 2)
    cv2.line(image, (ox, oy - 10), (ox, oy + 10), MAGENTA, 2)
    _label(image, "(0,0)", (ox + 14, oy), MAGENTA)

    _label(image, "%d balls, %d reported  %d ROIs  %.0f fps"
           % (len(balls), min(len(balls), MAX_BALLS), region_count, _fps), (8, 18), WHITE)


def runPipeline(image, llrobot):
    global _fps, _last_frame

    now = time.monotonic()
    if _last_frame is not None and now > _last_frame:
        _fps += FPS_SMOOTHING * (1.0 / (now - _last_frame) - _fps)
    _last_frame = now

    height, width = image.shape[:2]
    scale_x = float(CALIBRATION_SIZE[0]) / width
    scale_y = float(CALIBRATION_SIZE[1]) / height

    small = cv2.resize(image, (max(1, int(round(width * DETECTION_SCALE))),
                               max(1, int(round(height * DETECTION_SCALE)))),
                       interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

    frame_short_side = min(small.shape[0], small.shape[1])
    mask_canvas = np.zeros(small.shape, np.uint8) if DISPLAY_MODE == "MASK" else None

    candidates = []
    region_count = 0
    for ball in BALL_TYPES:
        color_mask = ball.mask(hsv)
        roi_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, _CLOSE_KERNEL)
        if mask_canvas is not None:
            mask_canvas[roi_mask > 0] = ball.draw

        min_r = frame_short_side * MIN_RADIUS_FRAME_FRACTION * ball.radius_scale
        max_r = frame_short_side * MAX_RADIUS_FRAME_FRACTION * ball.radius_scale

        mask_circles = []
        regions = _search_regions(ball, roi_mask, color_mask, min_r, max_r, mask_circles)
        region_count += len(regions)

        hough = _hough_circles(ball, gray, color_mask, regions, min_r, max_r)
        candidates.extend(hough)
        candidates.extend(_unclaimed(mask_circles, hough))

    balls = _suppress_overlaps(candidates)
    ground = _to_ground(balls, scale_x, scale_y) if balls else []
    # Nearest first, so the ones the llpython budget drops are the far ones.
    order = sorted(range(len(balls)), key=lambda i: math.hypot(ground[i][0], ground[i][1]))

    llpython = [SCRIPT_ID, float(min(len(balls), MAX_BALLS))]
    for i in order[:MAX_BALLS]:
        llpython.extend((float(balls[i][0].code), float(ground[i][0]), float(ground[i][1])))

    if mask_canvas is not None:
        image = cv2.resize(mask_canvas, (width, height), interpolation=cv2.INTER_NEAREST)
    if DRAW_OVERLAY:
        _draw_overlay(image, balls, ground, order, region_count, scale_x, scale_y)

    if not order:
        return NO_CONTOUR, image, llpython
    # The contour only feeds tx/ty/ta; the positions ride in llpython. Point it at the nearest ball.
    _, x, y, radius, _ = balls[order[0]]
    up = 1.0 / DETECTION_SCALE
    box = np.array([[x - radius, y - radius], [x + radius, y - radius],
                    [x + radius, y + radius], [x - radius, y + radius]]) * up
    return box.reshape(-1, 1, 2).astype(np.int32), image, llpython
