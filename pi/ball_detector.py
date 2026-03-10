"""
Ball (Fuel) Detection Co-processor  —  Team 539
================================================
Runs as a module inside main.py.  Captures frames from a USB camera,
detects fuel balls by HSV color filtering + contour analysis, and
publishes detection results to NetworkTables.

An annotated MJPEG stream (green circle on nearest ball, orange on
the rest) is served to the driver station via CameraServer so the
driver can verify the camera aim and detection quality.

Tuning
------
1. Find the right HSV range for your balls:
   - Point the camera at a ball, grab a frame, and run:
       python3 -c "
       import cv2, numpy as np
       img = cv2.imread('ball.jpg')
       hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
       print(hsv[y, x])   # replace y,x with pixel coords of the ball
       "
   - Set HSV_LOW / HSV_HIGH to bracket that hue with ±10–15 hue,
     and ±40 saturation/value margin.

2. Adjust CAMERA_HFOV_DEG to match your specific camera (check the
   datasheet or calibrate with a ruler and known-distance target).

3. Lower MIN_BALL_AREA if small/far balls are missed;
   raise it if you get false-positive detections on field elements.

NetworkTables (table: "BallDetection")
---------------------------------------
  Count            integer  — number of balls currently visible
  NearestAngle     double   — horiz. angle to nearest ball (deg, +right)
  NearestDistance  double   — estimated distance to nearest ball (inches)
  Heartbeat        integer  — increments every frame; roboRIO staleness check
"""

import math
import cv2
import numpy as np
import cscore as cs
import ntcore

# ── Camera settings ────────────────────────────────────────────────────────────

CAMERA_INDEX  = 0      # /dev/video0 — change if using a second USB camera
FRAME_WIDTH   = 320
FRAME_HEIGHT  = 240
FPS           = 30

# ── Ball color (HSV) — TUNE THESE for your ball color ─────────────────────────
# OpenCV hue range: 0–180  (red ≈ 0/180, orange ≈ 10, yellow ≈ 25, green ≈ 60)

HSV_LOW  = np.array([ 20,  80,  80])   # (hue_min, sat_min, val_min)
HSV_HIGH = np.array([ 40, 255, 255])   # (hue_max, sat_max, val_max)

# ── Detection thresholds ───────────────────────────────────────────────────────

MIN_BALL_AREA    = 200    # px²  — smaller blobs are ignored as noise
MIN_CIRCULARITY  = 0.50   # 1.0 = perfect circle; rejects elongated shapes

# ── Physical / optical constants ──────────────────────────────────────────────

BALL_DIAMETER_INCHES = 9.5   # TODO: confirm from game manual
CAMERA_HFOV_DEG      = 68.5  # horizontal field-of-view; check your camera spec


# ── Helpers ────────────────────────────────────────────────────────────────────

def _focal_length_px(frame_width: int) -> float:
    return (frame_width / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG / 2.0))

def estimate_distance_inches(apparent_radius_px: float, frame_width: int) -> float:
    """Pinhole-model distance from apparent ball radius."""
    if apparent_radius_px <= 0:
        return 0.0
    return (BALL_DIAMETER_INCHES * _focal_length_px(frame_width)) / (2.0 * apparent_radius_px)

def angle_from_center_deg(cx: float, frame_width: int) -> float:
    """
    Horizontal angle (degrees) from the camera's optical axis.
    Negative = left of center, positive = right.
    """
    offset = cx - (frame_width / 2.0)
    return math.degrees(math.atan2(offset, _focal_length_px(frame_width)))


# ── Main entry point ──────────────────────────────────────────────────────────

def run(table: ntcore.NetworkTable) -> None:
    """
    Blocking loop.  Call from main.py in a daemon thread.

    :param table: Pre-initialized NT table (e.g. inst.getTable("BallDetection")).
                  main.py owns the NT connection so this module never calls
                  startClient4() itself.
    """

    count_pub = table.getIntegerTopic("Count").publish()
    angle_pub = table.getDoubleTopic("NearestAngle").publish()
    dist_pub  = table.getDoubleTopic("NearestDistance").publish()
    hb_pub    = table.getIntegerTopic("Heartbeat").publish()

    # ── Camera and stream ──────────────────────────────────────────────────────
    camera = cs.UsbCamera("ball_cam", CAMERA_INDEX)
    camera.setResolution(FRAME_WIDTH, FRAME_HEIGHT)
    camera.setFPS(FPS)
    camera.setExposureManual(50)   # reduce motion blur; tune per environment

    annotated_src = cs.CvSource(
        "Ball Detection", cs.VideoMode.PixelFormat.kMJPEG,
        FRAME_WIDTH, FRAME_HEIGHT, FPS)
    server = cs.MjpegServer("ball_stream", 1182)
    server.setSource(annotated_src)

    sink = cs.CvSink("ball_cv_sink")
    sink.setSource(camera)

    frame     = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    kernel    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    heartbeat = 0

    print("Ball detection: camera open, streaming on :1182")

    while True:
        ts, frame = sink.grabFrame(frame)
        if ts == 0:
            continue   # frame error — don't update NT

        # ── HSV color filter ───────────────────────────────────────────────────
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)   # remove speckle
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)   # fill gaps

        # ── Contour detection ─────────────────────────────────────────────────
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        balls: list[tuple[float, float, float, float]] = []  # (cx, cy, radius, area)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_BALL_AREA:
                continue
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            if (4 * math.pi * area / (perimeter ** 2)) < MIN_CIRCULARITY:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            balls.append((cx, cy, radius, area))

        # Sort largest (closest) first
        balls.sort(key=lambda b: b[3], reverse=True)

        count = len(balls)
        if count > 0:
            cx, cy, radius, _ = balls[0]
            nearest_angle    = angle_from_center_deg(cx, FRAME_WIDTH)
            nearest_distance = estimate_distance_inches(radius, FRAME_WIDTH)
        else:
            nearest_angle    = 0.0
            nearest_distance = 0.0

        count_pub.set(count)
        angle_pub.set(nearest_angle)
        dist_pub .set(nearest_distance)

        # ── Annotate and stream ────────────────────────────────────────────────
        annotated = frame.copy()
        for i, (bx, by, br, _) in enumerate(balls):
            color = (0, 255, 0) if i == 0 else (0, 165, 255)
            cv2.circle(annotated, (int(bx), int(by)), int(br), color, 2)
            dist_label = f"{int(estimate_distance_inches(br, FRAME_WIDTH))}in"
            cv2.putText(annotated, dist_label,
                        (int(bx) - 18, int(by) - int(br) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        cv2.putText(annotated, f"Balls: {count}",
                    (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        annotated_src.putFrame(annotated)

        heartbeat += 1
        hb_pub.set(heartbeat)
