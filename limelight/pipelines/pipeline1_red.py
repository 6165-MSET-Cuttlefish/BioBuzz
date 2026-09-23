"""HIVE-cell SnapScript for the Limelight 3A — pipeline 1, RED.

GENERATED from limelight/cell_tip_snapscript.py by limelight/generate_pipelines.py; do not edit.

Reports whether this alliance's HIVE cell has tipped. A cluster in frame and reading upside-down
(|roll| >= 90) is scorable; right-side up or absent is tipped. That is the inverse of the roll
heuristic in FIRST's "AprilTag Clusters" Tech Tip, which assumes the camera looks the way the launcher
launches. Roll is each tag's in-image rotation, not a pose solve, so the Limelight must be mounted
with its image upright. The verdict only flips once the new reading has held for HOLD_SECONDS, in
either direction, and starts as tipped.

Reads nothing from the hub. llpython is one value; keep in step with LimelightCamera:
    0  tipped, 1 or 0
"""

import math
import time

import cv2
import numpy as np

ALLIANCE = "RED"
PIPELINE_INDEX = 1
CLUSTERS = (("SCORING", (30, 31, 32, 33)), ("AUDIENCE", (34, 35, 36, 37)))

HOLD_SECONDS = 0.25
MIN_TAG_AREA_PX = 120.0
MIN_VISIBLE_TAGS = 1
SCORABLE_MIN_ROLL_DEG = 90.0
DRAW_OVERLAY = True

NO_CONTOUR = np.array([[]])

FONT = cv2.FONT_HERSHEY_SIMPLEX
GREEN = (0, 255, 0)
RED = (0, 0, 255)
AMBER = (0, 190, 255)
GREY = (150, 150, 150)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)


def _make_detector():
    """Works with both the old and the new cv2.aruco API."""
    aruco = cv2.aruco
    if hasattr(aruco, "getPredefinedDictionary"):
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    else:
        dictionary = aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)

    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        return lambda gray: detector.detectMarkers(gray)[:2]

    params = aruco.DetectorParameters_create()
    return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)[:2]


_detect = _make_detector()

# Module globals persist between frames; the debounce lives here.
_tipped = True
_pending_since = None


def _tag_roll_deg(pts):
    """c0->c1 is the tag's top edge: ~0 upright, ~180 upside-down."""
    return math.degrees(math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0]))


def _circular_mean_deg(angles):
    """So 179 and -179 average to 180, not 0."""
    x = sum(math.cos(math.radians(a)) for a in angles)
    y = sum(math.sin(math.radians(a)) for a in angles)
    if x == 0.0 and y == 0.0:
        return float("nan")
    return math.degrees(math.atan2(y, x))


def _scorable_roll(roll):
    return not math.isnan(roll) and abs(roll) >= SCORABLE_MIN_ROLL_DEG


def _label(image, text, origin, color, scale=0.5, thickness=1):
    (w, h), base = cv2.getTextSize(text, FONT, scale, thickness)
    x, y = origin
    cv2.rectangle(image, (x - 2, y - h - 2), (x + w + 2, y + base), BLACK, -1)
    cv2.putText(image, text, (x, y), FONT, scale, color, thickness, cv2.LINE_AA)


def _draw_overlay(image, found_by_cluster, rolls, now_scorable):
    for c, tags in enumerate(found_by_cluster):
        color = GREEN if _scorable_roll(rolls[c]) else RED
        for pts, tag_id in tags:
            cv2.polylines(image, [pts.astype(np.int32)], True, color, 2)
            cx, cy = pts.mean(axis=0)
            _label(image, "%d" % tag_id, (int(cx) - 10, int(cy)), color)

    y = 18
    _label(image, "%s  pipeline %d" % (ALLIANCE, PIPELINE_INDEX), (8, y), WHITE)
    for c, (label, ids) in enumerate(CLUSTERS):
        y += 20
        tags = found_by_cluster[c]
        if not tags:
            _label(image, "%-8s %d-%d  0/4  --" % (label, ids[0], ids[-1]), (8, y), GREY)
            continue
        good = _scorable_roll(rolls[c])
        _label(image, "%-8s %d-%d  %d/4  roll %+.1f  %s"
               % (label, ids[0], ids[-1], len(tags), rolls[c], "DOWN" if good else "UP"),
               (8, y), GREEN if good else RED)

    text, fill, ink = ("TIPPED", RED, WHITE) if _tipped else ("SCORABLE", GREEN, BLACK)
    (w, h), _ = cv2.getTextSize(text, FONT, 1.1, 3)
    y += 8
    cv2.rectangle(image, (4, y + 8), (12 + w, y + 20 + h), fill, -1)
    cv2.putText(image, text, (8, y + 14 + h), FONT, 1.1, ink, 3, cv2.LINE_AA)
    if _pending_since is not None:
        _label(image, "reads %s for %.2fs / %.2fs"
               % ("SCORABLE" if now_scorable else "TIPPED", time.monotonic() - _pending_since,
                  HOLD_SECONDS),
               (8, y + 40 + h), AMBER, 0.5, 1)


def runPipeline(image, llrobot):
    global _tipped, _pending_since

    now = time.monotonic()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    corners, found = _detect(gray)

    found_by_cluster = [[], []]
    if found is not None:
        for quad, tag_id in zip(corners, found.flatten()):
            tag_id = int(tag_id)
            for c, (_, ids) in enumerate(CLUSTERS):
                if tag_id in ids:
                    pts = quad.reshape(-1, 2).astype(np.float32)
                    if abs(cv2.contourArea(pts)) >= MIN_TAG_AREA_PX:
                        found_by_cluster[c].append((pts, tag_id))
                    break

    rolls = [_circular_mean_deg([_tag_roll_deg(pts) for pts, _ in tags]) if tags else float("nan")
             for tags in found_by_cluster]
    now_scorable = any(len(tags) >= MIN_VISIBLE_TAGS and _scorable_roll(rolls[c])
                       for c, tags in enumerate(found_by_cluster))

    if now_scorable != _tipped:
        _pending_since = None
    elif _pending_since is None:
        _pending_since = now
    if _pending_since is not None and now - _pending_since >= HOLD_SECONDS:
        _tipped = not now_scorable
        _pending_since = None

    if DRAW_OVERLAY:
        _draw_overlay(image, found_by_cluster, rolls, now_scorable)

    # The contour only feeds tx/ty/ta; the verdict rides in llpython.
    tags = found_by_cluster[0] + found_by_cluster[1]
    if not tags:
        return NO_CONTOUR, image, [1.0 if _tipped else 0.0]
    biggest = max(tags, key=lambda t: abs(cv2.contourArea(t[0])))[0]
    return biggest.reshape(-1, 1, 2).astype(np.int32), image, [1.0 if _tipped else 0.0]
