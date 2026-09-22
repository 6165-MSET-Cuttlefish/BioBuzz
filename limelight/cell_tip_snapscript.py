"""BIOBUZZ cell-tip SnapScript for the Limelight 3A — template.

Watches the four AprilTags under one cell and decides, on the camera, whether that cell has tipped
over onto them: the verdict is false while any of the four is in view and true once all four have
been out of view continuously for HOLD_SECONDS. Everything is baked in here, so the Control Hub
sends nothing at all — it selects this pipeline once at init and then only reads llpython.

This file is the source of truth. It is NOT what gets uploaded: `scripts/generate-limelight-
pipelines.py` stamps it out once per cell into limelight/pipelines/, each copy differing only in
TAG_IDS. Edit this file, re-run the generator, re-upload the four.

Upload: Limelight web UI (http://limelight.local:5801) -> pick the pipeline index -> Input tab ->
pipeline type "Python" -> paste the matching limelight/pipelines/ file -> Save. The index each cell
expects is Context.Cell's `pipeline` field, and the two have to agree; a mismatch is caught at run
time by the checksum in llpython[5], which the hub checks against the cell it thinks it selected.

llpython (here -> hub), 8 doubles, read by LimelightCamera.parse():
    0     tipped, 1 or 0
    1     how many of the four are visible this frame
    2     visible bitmask, bit i = TAG_IDS[i]
    3     seconds since any of the four was last seen
    4     whether any of them has been seen since the pipeline started
    5     checksum (sum) of TAG_IDS — identifies which cell this pipeline answers about
    6     AprilTags of any id in this frame
    7     frame counter, wrapping at 10000

llrobot is ignored.
"""

import time

import cv2
import numpy as np

TAG_IDS = (30, 31, 32, 33)  # generated per pipeline
CELL_NAME = "RED_1"  # generated per pipeline
PIPELINE_INDEX = 0  # generated per pipeline

HOLD_SECONDS = 0.25
MIN_TAG_AREA_PX = 120.0
REQUIRE_SEEN = False
DRAW_OVERLAY = True

CHECKSUM = float(sum(TAG_IDS))
NO_CONTOUR = np.array([[]])


def _make_detector():
    """Returns detect(gray) -> (corners, ids), across both the old and new cv2.aruco APIs."""
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

# Module scope, so it survives between frames — this is where the hold window lives.
_last_seen = None
_seen_since_start = False
_frames = 0


FONT = cv2.FONT_HERSHEY_SIMPLEX
GREEN = (0, 255, 0)
RED = (0, 0, 255)
GREY = (150, 150, 150)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)


def _label(image, text, origin, color, scale=0.5, thickness=1):
    """Text on a filled black plate, so it stays readable over a bright field."""
    (w, h), base = cv2.getTextSize(text, FONT, scale, thickness)
    x, y = origin
    cv2.rectangle(image, (x - 2, y - h - 2), (x + w + 2, y + base), BLACK, -1)
    cv2.putText(image, text, (x, y), FONT, scale, color, thickness, cv2.LINE_AA)


def _draw_overlay(image, seen, mask, visible, detected, hidden_s, tipped):
    """Everything the hub knows, drawn on the frame the Limelight web UI streams."""
    for pts, tag_id, slot in seen:
        mine = slot >= 0
        cv2.polylines(image, [pts.astype(np.int32)], True, GREEN if mine else GREY, 2 if mine else 1)
        cx, cy = pts.mean(axis=0)
        _label(image, "%d" % tag_id, (int(cx) - 10, int(cy)), GREEN if mine else GREY)

    y = 18
    _label(image, "%s  pipeline %d" % (CELL_NAME, PIPELINE_INDEX), (8, y), WHITE)

    # One column per watched tag, so a single missing tag is obvious at a glance.
    y += 22
    x = 8
    for i, tag_id in enumerate(TAG_IDS):
        lit = (mask >> i) & 1
        _label(image, "%d%s" % (tag_id, "" if lit else " x"), (x, y), GREEN if lit else RED)
        x += 62

    if tipped:
        (w, h), _ = cv2.getTextSize("TIPPED", FONT, 1.1, 3)
        cv2.rectangle(image, (4, y + 8), (12 + w, y + 20 + h), RED, -1)
        cv2.putText(image, "TIPPED", (8, y + 14 + h), FONT, 1.1, WHITE, 3, cv2.LINE_AA)
        y += 28 + h
    else:
        y += 26
        _label(image, "UPRIGHT  %d/4 visible" % visible, (8, y), GREEN, 0.6, 2)

    y += 22
    _label(image, "hidden %.2fs / %.2fs" % (hidden_s, HOLD_SECONDS),
           (8, y), RED if hidden_s >= HOLD_SECONDS else WHITE)
    y += 18
    _label(image, "tags in frame %d   seen since start %s   frame %d"
           % (detected, "yes" if _seen_since_start else "no", _frames), (8, y), WHITE, 0.45)


def runPipeline(image, llrobot):
    global _last_seen, _seen_since_start, _frames

    now = time.monotonic()
    _frames = (_frames + 1) % 10000
    if _last_seen is None:
        _last_seen = now

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    corners, found = _detect(gray)

    mask = 0
    visible = 0
    detected = 0
    best_quad = None
    best_area = 0.0
    seen = []  # (pts, tag_id, slot) for the overlay; slot is the TAG_IDS index, or -1 for a stranger

    if found is not None and len(found) > 0:
        for quad, tag_id in zip(corners, found.flatten()):
            pts = quad.reshape(-1, 2).astype(np.float32)
            area = abs(cv2.contourArea(pts))
            if area < MIN_TAG_AREA_PX:
                continue
            detected += 1
            tag_id = int(tag_id)
            slot = TAG_IDS.index(tag_id) if tag_id in TAG_IDS else -1
            seen.append((pts, tag_id, slot))
            if slot < 0 or (mask >> slot) & 1:
                continue
            mask |= 1 << slot
            visible += 1
            if area > best_area:
                best_area = area
                best_quad = pts

    if visible > 0:
        _last_seen = now
        _seen_since_start = True

    hidden_s = now - _last_seen
    tipped = visible == 0 and hidden_s >= HOLD_SECONDS and (_seen_since_start or not REQUIRE_SEEN)

    if DRAW_OVERLAY:
        _draw_overlay(image, seen, mask, visible, detected, hidden_s, tipped)

    llpython = [
        1.0 if tipped else 0.0,
        float(visible),
        float(mask),
        float(hidden_s),
        1.0 if _seen_since_start else 0.0,
        CHECKSUM,
        float(detected),
        float(_frames),
    ]

    # A contour is returned only so tx/ty/ta and result.isValid() track a watched tag when one is in
    # view; the tip verdict itself rides entirely in llpython.
    contour = NO_CONTOUR if best_quad is None else best_quad.reshape(-1, 1, 2).astype(np.int32)
    return contour, image, llpython
