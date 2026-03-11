"""
Shot Trajectory Visualizer  —  Team 539
========================================
Draws a 2D side-view of the current physics-calculated shot and streams it as
an MJPEG feed.  Add the following URL as a camera widget in Elastic:

    http://10.5.39.XX:1184/stream.mjpg

Canvas layout (side view, x = downrange distance, y = height above floor)
--------------------------------------------------------------------------
  Left:    Robot box (grey outline) with green shooter-exit dot + launch arrow
  Right:   Hub / target box (white outline) with scoring opening
  Curve:   Simulated parabolic trajectory line (white)
  Ball:    Yellow dot at the hub scoring opening
  Overlay: Distance, flywheel speed, angle, efficiency, drag coefficient

Runs at 10 Hz — the trajectory is static (no animation), so high FPS is
unnecessary and this conserves Pi CPU for the other co-processors.

NetworkTables read
------------------
  UltraShooter/Avg Distance to Hub (m)            — distance to hub
  UltraShooter/Target ft/s                        — current flywheel setpoint
  UltraShooter/Physics Velocity ft/s              — physics-calculated speed
  UltraShooter/Launch Angle (deg)                 — fixed launch angle
  UltraShooter/Hood Height From Floor (in)        — hood exit height
  UltraShooter/Hub Center Height From Floor (in)  — hub center height
  UltraShooter/Pi Active                          — whether Pi physics is live
  ShooterTuner/Params/FlywheelEfficiency          — efficiency (0–1)
  ShooterTuner/Params/DragCoefficient             — drag constant (kg/m)
  ShooterTuner/Params/BallMassLbs                 — ball mass (lbs)
"""

import math
import time
import threading

import cv2
import numpy as np
import ntcore
from http.server import HTTPServer, BaseHTTPRequestHandler

# ── Stream settings ────────────────────────────────────────────────────────────

STREAM_PORT = 1184
CANVAS_W    = 800
CANVAS_H    = 420
FPS         = 10      # static diagram — low FPS sufficient

# ── Colours (BGR) ─────────────────────────────────────────────────────────────

_BG          = (28,  28,  28)
_WHITE       = (255, 255, 255)
_GREY        = (140, 140, 140)
_DARK_GREY   = (70,  70,  70)
_GREEN       = (80,  200,  80)
_YELLOW      = (0,   215, 255)   # bright yellow in BGR
_TRAJ_COLOR  = (210, 210, 210)
_TEXT_COLOR  = (225, 225, 225)
_LABEL_COLOR = (180, 200, 255)
_DIM_RED     = (80,   80, 200)

# ── Shared frame buffer (producer: render loop / consumer: HTTP handlers) ─────

_frame_lock  = threading.Lock()
_latest_jpeg = b""


# ── MJPEG HTTP server ──────────────────────────────────────────────────────────

class _MjpegHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _frame_lock:
                    jpeg = _latest_jpeg
                if jpeg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                time.sleep(1.0 / FPS)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *_):
        pass   # suppress access-log noise


def _start_server():
    HTTPServer(("", STREAM_PORT), _MjpegHandler).serve_forever()


# ── Physics helpers ────────────────────────────────────────────────────────────

def _simulate_trajectory(v0_mps: float, angle_rad: float,
                          drag_per_mass: float, hood_height_m: float,
                          dt: float = 0.005) -> list[tuple[float, float]]:
    """
    Euler-integrates the trajectory.  Returns a list of (x, y) world-space
    coordinates in metres, with y measured from the floor (not the hood exit).
    Stops when the ball hits the floor or travels > 12 m.
    """
    vx = v0_mps * math.cos(angle_rad)
    vy = v0_mps * math.sin(angle_rad)
    x  = 0.0
    y  = hood_height_m   # start at hood exit above the floor
    pts: list[tuple[float, float]] = [(x, y)]
    for _ in range(4000):
        speed = math.sqrt(vx * vx + vy * vy)
        vx += (-drag_per_mass * speed * vx) * dt
        vy += (-9.81 - drag_per_mass * speed * vy) * dt
        x  += vx * dt
        y  += vy * dt
        pts.append((x, y))
        if y < -0.1 or x > 12.0:
            break
    return pts


# ── Canvas rendering ──────────────────────────────────────────────────────────

def _draw_frame(distance_m: float, v_flywheel_fps: float, angle_deg: float,
                hood_h_m: float, hub_h_m: float,
                efficiency: float, drag_coeff: float, ball_mass_lbs: float,
                pi_active: bool) -> bytes:
    """
    Renders one trajectory frame and returns JPEG bytes.
    """
    angle_rad     = math.radians(angle_deg)
    ball_mass_kg  = ball_mass_lbs * 0.453592
    drag_per_mass = drag_coeff / max(ball_mass_kg, 0.001) if drag_coeff > 0 else 0.0
    v0_ball_mps   = (v_flywheel_fps / 3.28084) * efficiency

    # ── Compute trajectory ────────────────────────────────────────────────────
    traj: list[tuple[float, float]] = []
    if distance_m > 0.1 and v0_ball_mps > 0.5:
        traj = _simulate_trajectory(v0_ball_mps, angle_rad, drag_per_mass, hood_h_m)

    # ── World bounding box for the canvas ────────────────────────────────────
    x_max = max(distance_m * 1.12, 1.5)
    y_max = hub_h_m * 1.45
    if traj:
        y_max = max(y_max, max(p[1] for p in traj) * 1.18)

    # ── Layout margins ────────────────────────────────────────────────────────
    ML = 58   # left  margin (space for y-axis labels)
    MR = 18   # right margin
    MT = 32   # top   margin (title)
    MB = 38   # bottom margin (x-axis labels)
    pw = CANVAS_W - ML - MR   # plot width  (pixels)
    ph = CANVAS_H - MT - MB   # plot height (pixels)

    def w2c(x_m: float, y_m: float) -> tuple[int, int]:
        """World metres → canvas pixel (clamped)."""
        px = int(ML + x_m / x_max * pw)
        py = int(CANVAS_H - MB - y_m / y_max * ph)
        return (max(0, min(CANVAS_W - 1, px)),
                max(0, min(CANVAS_H - 1, py)))

    # ── Canvas ────────────────────────────────────────────────────────────────
    img = np.full((CANVAS_H, CANVAS_W, 3), _BG, dtype=np.uint8)

    # Floor line
    cv2.line(img, w2c(0, 0), w2c(x_max, 0), _DARK_GREY, 1)

    # ── Y-axis tick lines & labels ────────────────────────────────────────────
    for y_tick in [0.0, hood_h_m, hub_h_m]:
        cx, cy = w2c(0, y_tick)
        cv2.line(img, (ML - 5, cy), (ML, cy), _GREY, 1)
        cv2.putText(img, f"{y_tick:.1f}m", (2, cy + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, _GREY, 1)

    # ── X-axis tick labels ────────────────────────────────────────────────────
    for x_tick in [0.0, distance_m * 0.5, distance_m]:
        cx, cy = w2c(x_tick, 0)
        cv2.line(img, (cx, cy), (cx, cy + 4), _GREY, 1)
        cv2.putText(img, f"{x_tick:.1f}m", (cx - 14, cy + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, _GREY, 1)

    # ── Robot / shooter box (left) ────────────────────────────────────────────
    ROBOT_W_M = 0.65
    ROBOT_H_M = hood_h_m + 0.03
    cv2.rectangle(img, w2c(0, ROBOT_H_M), w2c(ROBOT_W_M, 0), _GREY, 2)

    # Shooter exit point (green dot)
    shoot_px = w2c(ROBOT_W_M * 0.88, hood_h_m)
    cv2.circle(img, shoot_px, 5, _GREEN, -1)

    # Launch-direction arrow
    arr_end = w2c(ROBOT_W_M * 0.88 + 0.22 * math.cos(angle_rad),
                  hood_h_m        + 0.22 * math.sin(angle_rad))
    cv2.arrowedLine(img, shoot_px, arr_end, _GREEN, 2, tipLength=0.35)

    # ── Hub / target box (right) ──────────────────────────────────────────────
    HUB_W_M    = 0.28
    HUB_OPEN_M = 0.28   # opening height
    hub_tl = w2c(distance_m - HUB_W_M, hub_h_m + HUB_OPEN_M / 2)
    hub_br = w2c(distance_m,            hub_h_m - HUB_OPEN_M / 2)
    cv2.rectangle(img, hub_tl, hub_br, _WHITE, 2)

    # Stand from hub base to floor
    stand_top    = w2c(distance_m - HUB_W_M / 2, hub_h_m - HUB_OPEN_M / 2)
    stand_bottom = w2c(distance_m - HUB_W_M / 2, 0)
    cv2.line(img, stand_top, stand_bottom, _DARK_GREY, 1)

    # ── Trajectory curve ──────────────────────────────────────────────────────
    if len(traj) >= 2:
        pts_canvas = [w2c(p[0], p[1]) for p in traj]
        for i in range(len(pts_canvas) - 1):
            cv2.line(img, pts_canvas[i], pts_canvas[i + 1], _TRAJ_COLOR, 2)

    # ── Fuel ball — yellow dot at hub opening ─────────────────────────────────
    ball_px = w2c(distance_m - HUB_W_M * 0.15, hub_h_m)
    cv2.circle(img, ball_px, 9,  _YELLOW, -1)
    cv2.circle(img, ball_px, 9,  _WHITE,   1)   # thin outline

    # ── Info overlay (top-left of plot area) ─────────────────────────────────
    src = "Pi" if pi_active else "RIO"
    info_lines = [
        (f"Dist:   {distance_m * 3.28084:.1f} ft  ({distance_m / 0.0254:.0f}\")", _TEXT_COLOR),
        (f"Speed:  {v_flywheel_fps:.1f} ft/s  (flywheel)",              _TEXT_COLOR),
        (f"Angle:  {angle_deg:.1f}\u00b0  Hood: {hood_h_m * 39.37:.0f}\" "
         f" Hub: {hub_h_m * 39.37:.0f}\"",                              _TEXT_COLOR),
        (f"Effic:  {efficiency * 100:.0f}%   "
         f"Drag: {drag_coeff:.4f} kg/m",                                _LABEL_COLOR),
        (f"Source: {src}",
         _GREEN if pi_active else _LABEL_COLOR),
    ]
    for idx, (text, color) in enumerate(info_lines):
        cv2.putText(img, text, (ML + 6, MT + 10 + idx * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    # ── Title ─────────────────────────────────────────────────────────────────
    cv2.putText(img, "Team 539  \u2014  Shot Trajectory",
                (CANVAS_W // 2 - 120, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, _LABEL_COLOR, 1, cv2.LINE_AA)

    # ── Encode to JPEG ────────────────────────────────────────────────────────
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return bytes(buf) if ok else b""


# ── Main entry point ──────────────────────────────────────────────────────────

def run(table: ntcore.NetworkTable) -> None:
    """
    Blocking loop.  Called from main.py in a daemon thread.

    :param table: Pre-initialized UltraShooter NT table.
                  The ShooterTuner table is accessed via the same NT instance.
    """
    global _latest_jpeg

    inst         = table.getInstance()
    tuner_params = inst.getTable("ShooterTuner").getSubTable("Params")

    dist_sub     = table.getDoubleTopic("Avg Distance to Hub (ft)").subscribe(0.0)
    target_sub   = table.getDoubleTopic("Target ft/s").subscribe(0.0)
    physics_sub  = table.getDoubleTopic("Physics Velocity ft/s").subscribe(0.0)
    angle_sub    = table.getDoubleTopic("Launch Angle (deg)").subscribe(75.0)
    hood_sub     = table.getDoubleTopic("Hood Height From Floor (in)").subscribe(27.0)
    hub_sub      = table.getDoubleTopic("Hub Center Height From Floor (in)").subscribe(96.0)
    pi_act_sub   = table.getBooleanTopic("Pi Active").subscribe(False)

    effic_sub    = tuner_params.getDoubleTopic("FlywheelEfficiency").subscribe(0.85)
    drag_sub     = tuner_params.getDoubleTopic("DragCoefficient").subscribe(0.0132)
    mass_sub     = tuner_params.getDoubleTopic("BallMassLbs").subscribe(0.595)

    # Start HTTP server in a background thread
    threading.Thread(target=_start_server, daemon=True,
                     name="traj-http").start()
    print(f"[TrajectoryViz] MJPEG stream on port {STREAM_PORT}  "
          f"→ http://10.5.39.XX:{STREAM_PORT}/stream.mjpg")

    while True:
        d       = dist_sub.get() / 3.28084   # ft → m for physics/canvas
        v_fps   = target_sub.get()
        angle   = angle_sub.get()
        hood_h  = hood_sub.get() * 0.0254    # inches → metres
        hub_h   = hub_sub.get()  * 0.0254
        pi_act  = pi_act_sub.get()
        effic   = effic_sub.get()
        drag    = drag_sub.get()
        mass    = mass_sub.get()

        # Use physics velocity if no active setpoint
        if v_fps < 1.0:
            v_fps = physics_sub.get()

        frame = _draw_frame(d, v_fps, angle, hood_h, hub_h,
                            effic, drag, mass, pi_act)
        with _frame_lock:
            _latest_jpeg = frame

        time.sleep(1.0 / FPS)
