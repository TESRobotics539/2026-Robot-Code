"""
AprilTag Backup Vision Co-processor  —  Team 539
=================================================
Runs as a module inside main.py.  Detects AprilTags with a USB camera,
estimates robot field pose, and publishes results to NetworkTables.

This is the FALLBACK for the Limelight.  The Java side only calls
addVisionMeasurement() from this source when the Limelight returns no
confident data.  Standard deviations are set higher than the Limelight
values to reflect lower precision.

Camera
------
This module uses CAMERA_INDEX = 1 (/dev/video1) so it does not conflict
with ball_detector.py which uses /dev/video0.  Wire a second USB camera
forward-facing on the robot.

Calibration
-----------
The default CAMERA_FX / FY / CX / CY are approximate for a typical
640 × 480 USB camera.  For best accuracy, calibrate your camera:

  1. Print a 9 × 6 OpenCV chessboard pattern.
  2. Capture 20+ images of it from varying angles.
  3. Run:
       python3 calibrate_camera.py   (not included — see OpenCV docs)
  4. Paste the resulting fx, fy, cx, cy here.

Without calibration the pose X/Y accuracy is typically ±15–30 cm — still
useful as a fallback but not as good as a calibrated Limelight.

Field layout
------------
Place the 2026 AprilTag field layout JSON at /home/pi/field.json on the Pi.

To get the file:
  - Copy from your PC:  C:\\Users\\Public\\wpilib\\2026\\tools\\apriltagfields\\
  - Or download from:   https://github.com/wpilibsuite/allwpilib (resources/)
  - Or copy from the roboRIO deploy directory via FTP (10.5.39.2, port 22).

NetworkTables (table: "PiVision")
----------------------------------
  RobotX            double   — estimated robot X in field coords (m)
  RobotY            double   — estimated robot Y in field coords (m)
  RobotYaw          double   — estimated robot heading (degrees, CCW positive)
  TagCount          integer  — number of tags used in this estimate
  AvgDecisionMargin double   — average detection confidence (0–100+)
  Heartbeat         integer  — increments every frame; roboRIO staleness check
"""

import math
import time
import numpy as np
import cscore as cs
import ntcore

from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d

# ── Camera settings ────────────────────────────────────────────────────────────

CAMERA_INDEX  = 1      # /dev/video1 — separate from ball_detector's camera 0
FRAME_WIDTH   = 640
FRAME_HEIGHT  = 480
FPS           = 20     # lower than ball detector to leave CPU headroom

# ── Camera intrinsics — calibrate for accurate pose ───────────────────────────
# Approximate for a typical 640 × 480 consumer USB camera.
# Replace with values from a proper chessboard calibration.

CAMERA_FX = 500.0              # horizontal focal length (pixels)
CAMERA_FY = 500.0              # vertical focal length (pixels)
CAMERA_CX = FRAME_WIDTH  / 2.0 # principal point X (pixels)
CAMERA_CY = FRAME_HEIGHT / 2.0 # principal point Y (pixels)

# ── AprilTag physical size ─────────────────────────────────────────────────────
# Measure the black border of a field tag (outer edge to outer edge).
# For most FRC fields this is 6.5 in = 0.1651 m.  Confirm from game manual.

TAG_SIZE_METERS = 0.1651  # TODO: confirm from 2026 game manual

# ── Minimum detection quality to use a tag for pose estimation ────────────────
# Higher values reject ambiguous / partially occluded tags.
# Range roughly 0–100; most confident detections score 50+.

MIN_DECISION_MARGIN = 35.0

# ── Camera-to-robot transform ─────────────────────────────────────────────────
# Where the Pi camera is mounted relative to the robot center origin.
# Positive X = forward, Positive Y = left, Positive Z = up.
# All in meters and radians.  Measure carefully after mounting.

CAM_X     = 0.20   # TODO: measure — 20 cm forward is a placeholder
CAM_Y     = 0.00   # TODO: measure — centered left-right
CAM_Z     = 0.50   # TODO: measure — 50 cm above floor
CAM_ROLL  = 0.0    # radians
CAM_PITCH = 0.0    # radians — tilt up if camera is angled upward
CAM_YAW   = 0.0    # radians — 0 = facing straight forward

# ── Field layout path ─────────────────────────────────────────────────────────

FIELD_JSON_PATH = "/home/pi/field.json"


# ── Helpers ────────────────────────────────────────────────────────────────────

def _load_field_layout():
    """
    Attempts to load the AprilTag field layout from /home/pi/field.json.
    Falls back to the most recent WPILib built-in layout if the file is missing.
    Returns None if no layout can be found (detection still works, pose skipped).
    """
    from wpimath.apriltag import AprilTagFieldLayout, AprilTagField
    import os

    if os.path.exists(FIELD_JSON_PATH):
        try:
            layout = AprilTagFieldLayout(FIELD_JSON_PATH)
            print(f"AprilTag vision: loaded field layout from {FIELD_JSON_PATH}")
            return layout
        except Exception as e:
            print(f"AprilTag vision: could not load {FIELD_JSON_PATH}: {e}")

    # Try known WPILib built-in field names newest-first.
    for field in ["k2026Reefscape", "k2025Reefscape"]:
        try:
            layout = AprilTagFieldLayout.loadField(getattr(AprilTagField, field))
            print(f"AprilTag vision: using built-in layout '{field}'")
            return layout
        except Exception:
            pass

    print("AprilTag vision: WARNING — no field layout found; publishing tag count only.")
    return None


def _weighted_pose_average(
    poses_and_weights: list[tuple[Pose3d, float]]
) -> Pose3d | None:
    """
    Computes a weighted average of 3-D poses.
    Rotation is averaged via quaternion SLERP (approx. as unit-vector average).
    """
    if not poses_and_weights:
        return None

    total_weight = sum(w for _, w in poses_and_weights)
    if total_weight == 0:
        return None

    # Weighted average of translation
    avg_x = sum(p.X() * w for p, w in poses_and_weights) / total_weight
    avg_y = sum(p.Y() * w for p, w in poses_and_weights) / total_weight
    avg_z = sum(p.Z() * w for p, w in poses_and_weights) / total_weight

    # Circular-mean of yaw (only meaningful for 2-D projection)
    sin_sum = sum(math.sin(p.rotation().Z()) * w for p, w in poses_and_weights)
    cos_sum = sum(math.cos(p.rotation().Z()) * w for p, w in poses_and_weights)
    avg_yaw = math.atan2(sin_sum, cos_sum)

    # Keep pitch/roll from the highest-weight estimate (they are less important)
    best_pose = max(poses_and_weights, key=lambda pw: pw[1])[0]

    return Pose3d(
        Translation3d(avg_x, avg_y, avg_z),
        Rotation3d(
            best_pose.rotation().X(),
            best_pose.rotation().Y(),
            avg_yaw,
        ),
    )


# ── Main entry point ──────────────────────────────────────────────────────────

def run(table: ntcore.NetworkTable) -> None:
    """
    Blocking loop.  Call from main.py in a daemon thread.

    :param table: Pre-initialized NT table (e.g. inst.getTable("PiVision")).
    """
    from robotpy_apriltag import AprilTagDetector, AprilTagPoseEstimator

    # ── Publishers ──────────────────────────────────────────────────────────────
    x_pub      = table.getDoubleTopic("RobotX").publish()
    y_pub      = table.getDoubleTopic("RobotY").publish()
    yaw_pub    = table.getDoubleTopic("RobotYaw").publish()
    count_pub  = table.getIntegerTopic("TagCount").publish()
    margin_pub = table.getDoubleTopic("AvgDecisionMargin").publish()
    hb_pub     = table.getIntegerTopic("Heartbeat").publish()

    # ── Field layout ───────────────────────────────────────────────────────────
    layout = _load_field_layout()

    # ── Camera-to-robot transform ──────────────────────────────────────────────
    robot_to_camera = Transform3d(
        Translation3d(CAM_X, CAM_Y, CAM_Z),
        Rotation3d(CAM_ROLL, CAM_PITCH, CAM_YAW),
    )

    # ── AprilTag detector ──────────────────────────────────────────────────────
    detector = AprilTagDetector()
    det_cfg = detector.Config()
    det_cfg.families = "tag36h11"   # FRC standard tag family
    detector.setConfig(det_cfg)

    # ── Pose estimator ─────────────────────────────────────────────────────────
    est_cfg = AprilTagPoseEstimator.Config(
        tagSize=TAG_SIZE_METERS,
        fx=CAMERA_FX, fy=CAMERA_FY,
        cx=CAMERA_CX, cy=CAMERA_CY,
    )
    estimator = AprilTagPoseEstimator(est_cfg)

    # ── Camera and stream ──────────────────────────────────────────────────────
    camera = cs.UsbCamera("apriltag_cam", CAMERA_INDEX)
    camera.setResolution(FRAME_WIDTH, FRAME_HEIGHT)
    camera.setFPS(FPS)
    camera.setExposureAuto()   # auto exposure for varying lighting conditions

    annotated_src = cs.CvSource(
        "AprilTag Detection", cs.VideoMode.PixelFormat.kMJPEG,
        FRAME_WIDTH, FRAME_HEIGHT, FPS)
    server = cs.MjpegServer("apriltag_stream", 1183)
    server.setSource(annotated_src)

    sink = cs.CvSink("apriltag_cv_sink")
    sink.setSource(camera)

    frame     = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    heartbeat = 0

    print("AprilTag vision: camera open, streaming on :1183")

    while True:
        ts, frame = sink.grabFrame(frame)
        if ts == 0:
            time.sleep(0.02)
            continue

        gray = __import__("cv2").cvtColor(frame, __import__("cv2").COLOR_BGR2GRAY)
        detections = detector.detect(gray)

        valid_detections = [
            d for d in detections
            if d.getDecisionMargin() >= MIN_DECISION_MARGIN
        ]

        robot_poses_and_weights: list[tuple[Pose3d, float]] = []
        annotated = frame.copy()
        cv2 = __import__("cv2")

        for det in valid_detections:
            tag_id = det.getId()

            # ── Draw tag outline ─────────────────────────────────────────────
            corners = det.getCorners([0.0] * 8)
            pts = [(int(corners[i * 2]), int(corners[i * 2 + 1])) for i in range(4)]
            for i in range(4):
                cv2.line(annotated, pts[i], pts[(i + 1) % 4], (0, 255, 255), 2)
            cx_px = int(sum(p[0] for p in pts) / 4)
            cy_px = int(sum(p[1] for p in pts) / 4)
            cv2.putText(annotated, f"ID {tag_id}", (cx_px - 15, cy_px),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # ── Pose estimate ────────────────────────────────────────────────
            if layout is None:
                continue

            tag_pose_optional = layout.getTagPose(tag_id)
            if tag_pose_optional is None:
                continue    # tag ID not in this year's layout

            # camera_to_tag: Transform3d from camera origin to tag center
            camera_to_tag: Transform3d = estimator.estimate(det)

            # Camera field pose: start at tag, undo camera_to_tag
            tag_field_pose: Pose3d = Pose3d(
                tag_pose_optional.translation(),
                tag_pose_optional.rotation(),
            )
            camera_field_pose: Pose3d = tag_field_pose.transformBy(
                camera_to_tag.inverse()
            )

            # Robot field pose: undo robot_to_camera from camera pose
            robot_field_pose: Pose3d = camera_field_pose.transformBy(
                robot_to_camera.inverse()
            )

            robot_poses_and_weights.append(
                (robot_field_pose, det.getDecisionMargin())
            )

        # ── Average all valid pose estimates ──────────────────────────────────
        avg_pose = _weighted_pose_average(robot_poses_and_weights)
        tag_count = len(robot_poses_and_weights)
        avg_margin = (
            sum(w for _, w in robot_poses_and_weights) / tag_count
            if tag_count > 0 else 0.0
        )

        if avg_pose is not None:
            robot_2d = avg_pose.toPose2d()
            x_pub.set(robot_2d.X())
            y_pub.set(robot_2d.Y())
            yaw_pub.set(robot_2d.rotation().degrees())

        count_pub.set(tag_count)
        margin_pub.set(avg_margin)

        # ── Status overlay ────────────────────────────────────────────────────
        cv2 = __import__("cv2")
        cv2.putText(annotated, f"Tags: {tag_count}  Conf: {avg_margin:.0f}",
                    (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        annotated_src.putFrame(annotated)

        heartbeat += 1
        hb_pub.set(heartbeat)
