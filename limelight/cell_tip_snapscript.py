"""BIOBUZZ cell-tip SnapScript for the Limelight 3A.

Watches the four AprilTags under one cell and decides, on the camera, whether that cell has tipped
over onto them: the verdict is false while any of the four is in view and true once all four have
been out of view continuously for the hold time. The Control Hub only says which four tags to watch
and reads the answer back, so the hub spends no frame time on detection and the hold is timed
against the camera's own frames rather than the OpMode loop.

Upload: Limelight web UI (http://limelight.local:5801) -> Input tab -> pipeline type "Python" ->
paste this file into the editor -> Save, then note the pipeline index and set
LimelightCamera.snapScriptPipeline to it. Keep the exposure low enough that the tags are not blown
out; nothing else in the pipeline's settings affects this script.

llrobot (hub -> here), 8 doubles, from LimelightCamera.pushInputs():
    0..3  the four tag ids to watch; all zero = use DEFAULT_IDS (bench testing from the web UI)
    4     hold seconds; <= 0 = DEFAULT_HOLD_S
    5     enabled; 0 = idle, skip detection entirely (ignored while 0..3 are all zero)
    6     minimum tag quad area in px^2; <= 0 = DEFAULT_MIN_AREA_PX
    7     require-seen; nonzero = never report tipped until the cell has been seen at least once

llpython (here -> hub), 8 doubles, read by LimelightCamera.parse():
    0     tipped, 1 or 0
    1     how many of the four are visible this frame
    2     visible bitmask, bit i = llrobot[i]
    3     seconds since any of the four was last seen
    4     whether any of them has been seen since this set was armed
    5     checksum (sum) of the tag set this verdict is about
    6     AprilTags of any id in this frame
    7     frame counter, wrapping at 10000
"""

import time

import cv2
import numpy as np

DEFAULT_IDS = (30, 31, 32, 33)
DEFAULT_HOLD_S = 0.25
DEFAULT_MIN_AREA_PX = 120.0
DRAW_OVERLAY = True

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
_armed_ids = ()
_last_seen = None
_seen_since_arm = False
_frames = 0


def _inputs(llrobot):
    def at(i, default=0.0):
        try:
            return float(llrobot[i])
        except (IndexError, TypeError, ValueError):
            return default

    ids = tuple(int(round(at(i))) for i in range(4))
    # No tag set at all means nobody is driving this from an OpMode — fall back to DEFAULT_IDS and
    # run anyway, so the web UI's own preview is useful with llrobot sitting at all zeros.
    hub_driven = any(ids)
    if not hub_driven:
        ids = DEFAULT_IDS
    hold = at(4)
    area = at(6)
    return (
        ids,
        hold if hold > 0 else DEFAULT_HOLD_S,
        at(5) != 0 if hub_driven else True,
        area if area > 0 else DEFAULT_MIN_AREA_PX,
        at(7) != 0,
    )


def runPipeline(image, llrobot):
    global _armed_ids, _last_seen, _seen_since_arm, _frames

    now = time.monotonic()
    _frames = (_frames + 1) % 10000
    ids, hold_s, enabled, min_area, require_seen = _inputs(llrobot)
    checksum = float(sum(ids))

    # A new tag set starts its own hold window: the previous cell's hidden time says nothing here.
    if ids != _armed_ids:
        _armed_ids = ids
        _last_seen = now
        _seen_since_arm = False

    if not enabled:
        _last_seen = now
        _seen_since_arm = False
        return NO_CONTOUR, image, [0.0, 0.0, 0.0, 0.0, 0.0, checksum, 0.0, float(_frames)]

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    corners, found = _detect(gray)

    mask = 0
    visible = 0
    detected = 0
    best_quad = None
    best_area = 0.0

    if found is not None and len(found) > 0:
        for quad, tag_id in zip(corners, found.flatten()):
            pts = quad.reshape(-1, 2).astype(np.float32)
            area = abs(cv2.contourArea(pts))
            if area < min_area:
                continue
            detected += 1
            tag_id = int(tag_id)
            for i, wanted in enumerate(ids):
                if tag_id != wanted or (mask >> i) & 1:
                    continue
                mask |= 1 << i
                visible += 1
                if area > best_area:
                    best_area = area
                    best_quad = pts
                break

    if visible > 0:
        _last_seen = now
        _seen_since_arm = True

    hidden_s = now - (_last_seen if _last_seen is not None else now)
    tipped = visible == 0 and hidden_s >= hold_s and (_seen_since_arm or not require_seen)

    if DRAW_OVERLAY:
        if best_quad is not None:
            cv2.polylines(image, [best_quad.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.putText(
            image,
            "TIPPED" if tipped else "%d/4 visible  %.2fs" % (visible, hidden_s),
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255) if tipped else (0, 255, 0),
            2,
        )

    llpython = [
        1.0 if tipped else 0.0,
        float(visible),
        float(mask),
        float(hidden_s),
        1.0 if _seen_since_arm else 0.0,
        checksum,
        float(detected),
        float(_frames),
    ]

    # A contour is returned only so tx/ty/ta and result.isValid() track a watched tag when one is in
    # view; the tip verdict itself rides entirely in llpython.
    contour = NO_CONTOUR if best_quad is None else best_quad.reshape(-1, 1, 2).astype(np.int32)
    return contour, image, llpython
