"""HIVE-cell SnapScript for the Limelight 3A — pipeline 2, BLUE.

GENERATED from limelight/cell_tip_snapscript.py by limelight/generate_pipelines.py; do not edit.

Reports whether this alliance's HIVE cell is scorable: its AprilTag cluster in frame and reading
upside-down (|roll| >= 90). That is the inverse of the roll heuristic in FIRST's "AprilTag Clusters"
Tech Tip, which assumes the camera looks the way the launcher launches. Roll is each tag's in-image
rotation, not a pose solve, so the Limelight must be mounted with its image upright. Right-side up
(|roll| < 90) or absent for HOLD_SECONDS is tipped.

llpython, 8 doubles; keep in step with LimelightCamera's OUT_* constants:
    0  tipped, 1 or 0
    1  scorable, 1 or 0
    2  roll of the targeted cluster in degrees, ROLL_NONE when none is in view
    3  visible tags in the targeted cluster
    4  targeted cell: 0 scoring side, 1 audience side, -1 none
    5  visible tags in the other cluster
    6  sum of this alliance's ids, so the hub can reject the wrong script
    7  frame counter, wrapping at 10000
"""

import math
import time

import cv2
import numpy as np

ALLIANCE = "BLUE"
PIPELINE_INDEX = 2
# Scoring side first for both alliances, so llpython[4] means the same thing either way.
CLUSTERS = (("SCORING", (42, 43, 44, 45)), ("AUDIENCE", (38, 39, 40, 41)))

HOLD_SECONDS = 0.25
MIN_TAG_AREA_PX = 120.0
MIN_VISIBLE_TAGS = 1
SCORABLE_MIN_ROLL_DEG = 90.0
# False: an empty frame reads as tipped after HOLD_SECONDS, even if the cluster was never seen.
REQUIRE_SEEN = False
# llpython travels in the results JSON, where a NaN can cost the hub the whole payload.
ROLL_NONE = 999.0
DRAW_OVERLAY = True

ALL_IDS = CLUSTERS[0][1] + CLUSTERS[1][1]
CHECKSUM = float(sum(ALL_IDS))
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

# Module globals persist between frames; the hold timer lives here.
_last_good = None
_seen_since_start = False
_frames = 0


def _tag_roll_deg(pts):
    """c0->c1 is the tag's top edge: ~0 upright, ~180 upside-down."""
    dx = pts[1][0] - pts[0][0]
    dy = pts[1][1] - pts[0][1]
    return math.degrees(math.atan2(dy, dx))


def _scorable_roll(roll):
    return not math.isnan(roll) and abs(roll) >= SCORABLE_MIN_ROLL_DEG


def _circular_mean_deg(angles):
    """So 179 and -179 average to 180, not 0."""
    x = sum(math.cos(math.radians(a)) for a in angles)
    y = sum(math.sin(math.radians(a)) for a in angles)
    if x == 0.0 and y == 0.0:
        return float("nan")
    return math.degrees(math.atan2(y, x))


def _label(image, text, origin, color, scale=0.5, thickness=1):
    (w, h), base = cv2.getTextSize(text, FONT, scale, thickness)
    x, y = origin
    cv2.rectangle(image, (x - 2, y - h - 2), (x + w + 2, y + base), BLACK, -1)
    cv2.putText(image, text, (x, y), FONT, scale, color, thickness, cv2.LINE_AA)


def _draw_overlay(image, found_by_cluster, strangers, rolls, target, tipped, scorable):
    for pts, tag_id in strangers:
        cv2.polylines(image, [pts.astype(np.int32)], True, GREY, 1)
        cx, cy = pts.mean(axis=0)
        _label(image, "%d" % tag_id, (int(cx) - 10, int(cy)), GREY)

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
        roll = rolls[c]
        if not tags:
            _label(image, "%-8s %d-%d  0/4  --" % (label, ids[0], ids[-1]), (8, y), GREY)
            continue
        good = _scorable_roll(roll)
        _label(image, "%-8s %d-%d  %d/4  roll %+.1f  %s%s"
               % (label, ids[0], ids[-1], len(tags), roll,
                  "DOWN" if good else "UP", "  <-- target" if c == target else ""),
               (8, y), GREEN if good else RED)

    y += 8
    if tipped:
        (w, h), _ = cv2.getTextSize("TIPPED", FONT, 1.1, 3)
        cv2.rectangle(image, (4, y + 8), (12 + w, y + 20 + h), RED, -1)
        cv2.putText(image, "TIPPED", (8, y + 14 + h), FONT, 1.1, WHITE, 3, cv2.LINE_AA)
        y += 28 + h
    elif scorable:
        (w, h), _ = cv2.getTextSize("SCORABLE", FONT, 1.1, 3)
        cv2.rectangle(image, (4, y + 8), (12 + w, y + 20 + h), GREEN, -1)
        cv2.putText(image, "SCORABLE", (8, y + 14 + h), FONT, 1.1, BLACK, 3, cv2.LINE_AA)
        y += 28 + h
    else:
        y += 20
        _label(image, "NOT SCORABLE, waiting out the hold", (8, y), AMBER, 0.6, 2)

    y += 20
    away = 0.0 if _last_good is None else time.monotonic() - _last_good
    _label(image, "not scorable for %.2fs / %.2fs   need %d tag%s + |roll| >= %.0f   seen %s   frame %d"
           % (away, HOLD_SECONDS, MIN_VISIBLE_TAGS, "" if MIN_VISIBLE_TAGS == 1 else "s",
              SCORABLE_MIN_ROLL_DEG, "yes" if _seen_since_start else "no", _frames),
           (8, y), WHITE, 0.45)


def runPipeline(image, llrobot):
    global _last_good, _seen_since_start, _frames

    now = time.monotonic()
    _frames = (_frames + 1) % 10000

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    corners, found = _detect(gray)

    found_by_cluster = [[], []]
    strangers = []

    if found is not None and len(found) > 0:
        for quad, tag_id in zip(corners, found.flatten()):
            pts = quad.reshape(-1, 2).astype(np.float32)
            if abs(cv2.contourArea(pts)) < MIN_TAG_AREA_PX:
                continue
            tag_id = int(tag_id)
            for c, (_, ids) in enumerate(CLUSTERS):
                if tag_id in ids:
                    found_by_cluster[c].append((pts, tag_id))
                    break
            else:
                strangers.append((pts, tag_id))

    rolls = [_circular_mean_deg([_tag_roll_deg(pts) for pts, _ in tags]) if tags else float("nan")
             for tags in found_by_cluster]

    # Any scorable-roll cluster wins; tag count breaks ties.
    target = -1
    best = (-1, -1)  # (scorable roll, visible count)
    for c, tags in enumerate(found_by_cluster):
        if not tags:
            continue
        good = 1 if _scorable_roll(rolls[c]) else 0
        if (good, len(tags)) > best:
            best = (good, len(tags))
            target = c

    if target < 0:
        roll = float("nan")
        visible = 0
        other_visible = 0
    else:
        roll = rolls[target]
        visible = len(found_by_cluster[target])
        other_visible = len(found_by_cluster[1 - target])

    scorable = target >= 0 and visible >= MIN_VISIBLE_TAGS and _scorable_roll(roll)
    if scorable:
        _last_good = now
        _seen_since_start = True
    elif _last_good is None:
        _last_good = now

    away_s = now - _last_good
    tipped = not scorable and away_s >= HOLD_SECONDS and (_seen_since_start or not REQUIRE_SEEN)

    if DRAW_OVERLAY:
        _draw_overlay(image, found_by_cluster, strangers, rolls, target, tipped, scorable)

    llpython = [
        1.0 if tipped else 0.0,
        1.0 if scorable else 0.0,
        ROLL_NONE if math.isnan(roll) else float(roll),
        float(visible),
        float(target),
        float(other_visible),
        CHECKSUM,
        float(_frames),
    ]

    # The contour only feeds tx/ty/ta; the verdict rides in llpython.
    if target < 0:
        return NO_CONTOUR, image, llpython
    biggest = max(found_by_cluster[target], key=lambda t: abs(cv2.contourArea(t[0])))[0]
    return biggest.reshape(-1, 1, 2).astype(np.int32), image, llpython
