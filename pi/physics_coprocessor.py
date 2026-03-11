"""
UltraShooter Physics Co-processor  —  Team 539
===============================================
Runs as a module inside main.py.  Reads the 1-second averaged hub distance
published by UltraShooter.java, computes the required flywheel surface velocity
(with optional aerodynamic drag and flywheel efficiency), and publishes the
result back so the roboRIO can use it in setPhysicsTarget().

The roboRIO falls back to its own local calculation automatically if this
script stops responding (heartbeat goes stale for > 500 ms).

Physics model
-------------
Without drag (DragCoefficient = 0):
    Analytical range equation solved for v₀:
        v₀_ball = d · √( g / (2·cos²θ·(d·tanθ − h)) )

With drag (DragCoefficient > 0):
    Quadratic aerodynamic drag applied during Euler integration:
        ax = -(B/m) · |v| · vx
        ay = -g − (B/m) · |v| · vy
    Binary search finds the ball exit speed that lands at (d, h).

Flywheel surface speed = v₀_ball / FlywheelEfficiency.

Constants
---------
Keep HOOD_HEIGHT_INCHES, HUB_HEIGHT_INCHES, SHOOTER_OFFSET_INCHES, and
LAUNCH_ANGLE_DEG in sync with Constants.java → UltraShooterConstants.
Physics tuning params (efficiency, drag, mass) are read live from
ShooterTuner/Params so they stay in sync with dashboard edits.

NetworkTables (table: "UltraShooter")
---------------------------------------
  Pi Physics Velocity ft/s  double  — raw physics-calculated flywheel velocity
  Pi Heartbeat              integer — increments every cycle; roboRIO staleness check
"""

import math
import time
import ntcore

# ── Geometry — mirror Constants.java UltraShooterConstants ────────────────────

HOOD_HEIGHT_INCHES    = 27.0   # kHoodHeightFromFloorInches
HUB_HEIGHT_INCHES     = 72.0   # kHubCenterHeightFromFloorInches
SHOOTER_OFFSET_INCHES = 8.0    # kShooterCenterlineOffsetInches
LAUNCH_ANGLE_DEG      = 75.0   # kLaunchAngleDegrees

# ── Physics defaults (overridden by ShooterTuner NT values each cycle) ────────

DEFAULT_EFFICIENCY     = 0.43    # FlywheelEfficiency — calibrated vs ShooterOrca at 2 m
DEFAULT_DRAG_COEFF     = 0.0132  # DragCoefficient (kg/m)
DEFAULT_BALL_MASS_LBS  = 0.595   # BallMassLbs

# ── Unit helpers ───────────────────────────────────────────────────────────────

def _in_to_m(x: float) -> float:
    return x * 0.0254

def _mps_to_fps(x: float) -> float:
    return x * 3.28084


# ── Numerical trajectory helpers ────────────────────────────────────────────────

def _simulate_y_at_x(
        v0: float, target_x: float, angle_rad: float,
        drag_per_mass: float, dt: float = 0.005) -> float:
    """
    Euler-integrates the trajectory and returns the ball's height (m, relative
    to hood exit) when it crosses *target_x* meters downrange.
    """
    vx = v0 * math.cos(angle_rad)
    vy = v0 * math.sin(angle_rad)
    x = 0.0
    y = 0.0
    prev_x = 0.0
    prev_y = 0.0
    for _ in range(5000):
        speed = math.sqrt(vx * vx + vy * vy)
        vx += (-drag_per_mass * speed * vx) * dt
        vy += (-9.81 - drag_per_mass * speed * vy) * dt
        prev_x, prev_y = x, y
        x += vx * dt
        y += vy * dt
        if x >= target_x:
            t = (target_x - prev_x) / (x - prev_x) if (x - prev_x) > 1e-9 else 0.0
            return prev_y + t * (y - prev_y)
        if y < -2.0:
            break
    return y


def _binary_search_v0(
        d: float, h: float, angle_rad: float, drag_per_mass: float) -> float:
    """
    Binary-searches for ball exit speed v₀ (m/s) that causes the trajectory to
    land at height *h* meters above the hood exit when it has traveled *d* meters
    horizontally.  Returns 0.0 if the shot is physically impossible.
    """
    HI = 40.0
    # Feasibility: if even max speed can't reach hub height, shot is impossible.
    if _simulate_y_at_x(HI, d, angle_rad, drag_per_mass) < h:
        return 0.0

    lo, hi = 0.5, HI
    for _ in range(60):
        mid = (lo + hi) * 0.5
        if _simulate_y_at_x(mid, d, angle_rad, drag_per_mass) < h:
            lo = mid
        else:
            hi = mid
    return (lo + hi) * 0.5


# ── Time-of-flight calculator ───────────────────────────────────────────────────

def _simulate_tof(v0: float, target_x: float, angle_rad: float, drag_per_mass: float,
                  dt: float = 0.005) -> float:
    """
    Returns the time (s) for the ball to travel *target_x* metres downrange
    under quadratic drag.  Uses the same Euler integration as _simulate_y_at_x
    with linear interpolation to sub-step accuracy.
    """
    vx = v0 * math.cos(angle_rad)
    vy = v0 * math.sin(angle_rad)
    x, prev_x = 0.0, 0.0
    for i in range(5000):
        speed = math.sqrt(vx * vx + vy * vy)
        vx += (-drag_per_mass * speed * vx) * dt
        vy += (-9.81 - drag_per_mass * speed * vy) * dt
        prev_x = x
        x += vx * dt
        if x >= target_x:
            frac = (target_x - prev_x) / (x - prev_x) if (x - prev_x) > 1e-9 else 0.0
            return (i + frac) * dt
    return 0.0


def calculate_tof_seconds(
        distance_to_hub_m: float,
        drag_coeff: float = DEFAULT_DRAG_COEFF,
        ball_mass_lbs: float = DEFAULT_BALL_MASS_LBS) -> float:
    """
    Returns time of flight (s) for the ball to reach the hub.
    Analytic when drag_coeff == 0; numerical otherwise.
    """
    angle_rad = math.radians(LAUNCH_ANGLE_DEG)
    d = distance_to_hub_m - _in_to_m(SHOOTER_OFFSET_INCHES)
    h = _in_to_m(HUB_HEIGHT_INCHES) - _in_to_m(HOOD_HEIGHT_INCHES)

    if d <= 0:
        return 0.0

    if drag_coeff <= 0:
        cos_t = math.cos(angle_rad)
        tan_t = math.tan(angle_rad)
        denom = 2.0 * cos_t * cos_t * (d * tan_t - h)
        if denom <= 0:
            return 0.0
        v0_mps = d * math.sqrt(9.81 / denom)
        return d / (v0_mps * cos_t)
    else:
        ball_mass_kg  = ball_mass_lbs * 0.453592
        drag_per_mass = drag_coeff / max(ball_mass_kg, 0.001)
        v0_mps = _binary_search_v0(d, h, angle_rad, drag_per_mass)
        if v0_mps <= 0:
            return 0.0
        return _simulate_tof(v0_mps, d, angle_rad, drag_per_mass)


# ── Core velocity calculator ────────────────────────────────────────────────────

def calculate_velocity_fps(
        distance_to_hub_m: float,
        flywheel_efficiency: float = DEFAULT_EFFICIENCY,
        drag_coeff: float = DEFAULT_DRAG_COEFF,
        ball_mass_lbs: float = DEFAULT_BALL_MASS_LBS) -> float:
    """
    Returns required flywheel surface velocity (ft/s) for the given
    robot-center-to-hub distance, or 0.0 if the shot is physically impossible.

    Parameters
    ----------
    distance_to_hub_m   : odometry robot-center → hub-center distance (m)
    flywheel_efficiency : ball exit speed / flywheel surface speed  (0–1)
    drag_coeff          : aerodynamic B = 0.5·Cd·ρ·A  (kg/m); 0 = no drag
    ball_mass_lbs       : ball mass (lbs); converted to kg internally
    """
    angle_rad = math.radians(LAUNCH_ANGLE_DEG)
    d = distance_to_hub_m - _in_to_m(SHOOTER_OFFSET_INCHES)
    h = _in_to_m(HUB_HEIGHT_INCHES) - _in_to_m(HOOD_HEIGHT_INCHES)

    if d <= 0 or flywheel_efficiency <= 0:
        return 0.0

    if drag_coeff <= 0:
        # ── Analytic vacuum solution ───────────────────────────────────────────
        cos_t = math.cos(angle_rad)
        tan_t = math.tan(angle_rad)
        denom = 2.0 * cos_t * cos_t * (d * tan_t - h)
        if denom <= 0:
            return 0.0
        v0_mps = d * math.sqrt(9.81 / denom)
    else:
        # ── Numerical solution with quadratic drag ────────────────────────────
        ball_mass_kg  = ball_mass_lbs * 0.453592
        drag_per_mass = drag_coeff / max(ball_mass_kg, 0.001)
        v0_mps = _binary_search_v0(d, h, angle_rad, drag_per_mass)
        if v0_mps <= 0:
            return 0.0

    # flywheel surface speed = ball exit speed / efficiency, then m/s → ft/s
    return _mps_to_fps(v0_mps / flywheel_efficiency)


# ── Main entry point ──────────────────────────────────────────────────────────

def run(table: ntcore.NetworkTable) -> None:
    """
    Blocking loop.  Call from main.py in a daemon thread.

    :param table: Pre-initialized NT table (e.g. inst.getTable("UltraShooter")).
                  main.py owns the NT connection so this module never calls
                  startClient4() itself.
    """
    inst         = table.getInstance()
    tuner_params = inst.getTable("ShooterTuner").getSubTable("Params")

    dist_sub     = table.getDoubleTopic("Avg Distance to Hub (ft)").subscribe(0.0)
    vel_pub      = table.getDoubleTopic("Pi Physics Velocity ft/s").publish()
    tof_pub      = table.getDoubleTopic("Pi Time of Flight (s)").publish()
    hb_pub       = table.getIntegerTopic("Pi Heartbeat").publish()

    effic_sub    = tuner_params.getDoubleTopic("FlywheelEfficiency").subscribe(DEFAULT_EFFICIENCY)
    drag_sub     = tuner_params.getDoubleTopic("DragCoefficient").subscribe(DEFAULT_DRAG_COEFF)
    mass_sub     = tuner_params.getDoubleTopic("BallMassLbs").subscribe(DEFAULT_BALL_MASS_LBS)

    heartbeat: int = 0
    print("Physics co-processor: running at 50 Hz (drag-aware).")

    while True:
        effic = effic_sub.get()
        drag  = drag_sub.get()
        mass  = mass_sub.get()

        dist_m = dist_sub.get() / 3.28084
        vel_pub.set(calculate_velocity_fps(dist_m, effic, drag, mass))  # mass in lbs
        tof_pub.set(calculate_tof_seconds(dist_m, drag, mass))
        heartbeat += 1
        hb_pub.set(heartbeat)
        time.sleep(0.02)   # 50 Hz — matches the roboRIO scheduler period
