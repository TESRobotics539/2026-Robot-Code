"""
UltraShooter Physics Co-processor  —  Team 539
===============================================
Runs as a module inside main.py.  Reads the 1-second averaged hub
distance published by UltraShooter.java, computes the required flywheel
surface velocity using the same projectile-motion formula as the roboRIO,
and publishes the result back so the roboRIO can use it in setPhysicsTarget().

The roboRIO falls back to its own local calculation automatically if
this script stops responding (heartbeat goes stale for > 500 ms).

Constants
---------
Keep HOOD_HEIGHT_INCHES, HUB_HEIGHT_INCHES, SHOOTER_OFFSET_INCHES, and
LAUNCH_ANGLE_DEG in sync with Constants.java → UltraShooterConstants.

The roboRIO is the single source of truth for the fine-tune offsets
(kNearShotOffsetPercent / kFarShotOffsetPercent); this script only
publishes raw physics velocity so the roboRIO can apply offsets itself.

NetworkTables (table: "UltraShooter")
---------------------------------------
  Pi Physics Velocity ft/s  double   — raw physics-calculated velocity
  Pi Heartbeat              integer  — increments every cycle; roboRIO staleness check
"""

import math
import time
import ntcore

# ── Geometry — mirror Constants.java UltraShooterConstants ────────────────────

HOOD_HEIGHT_INCHES    = 27.0   # kHoodHeightFromFloorInches
HUB_HEIGHT_INCHES     = 96.0   # kHubCenterHeightFromFloorInches
SHOOTER_OFFSET_INCHES = 8.0    # kShooterCenterlineOffsetInches
LAUNCH_ANGLE_DEG      = 75.0   # kLaunchAngleDegrees


# ── Unit helpers ───────────────────────────────────────────────────────────────

def inches_to_meters(x: float) -> float:
    return x * 0.0254

def mps_to_fps(x: float) -> float:
    return x * 3.28084


# ── Physics ────────────────────────────────────────────────────────────────────

def calculate_velocity_fps(distance_to_hub_m: float) -> float:
    """
    Returns required flywheel surface velocity (ft/s) for the given
    robot-center-to-hub distance, or 0 if the shot is physically impossible.

    Range equation solved for v₀:
        v₀ = d · √( g / (2·cos²θ·(d·tanθ − h)) )
    where d is horizontal distance (shooter exit → hub),
          h is net vertical rise (hub center above hood exit),
          θ is the fixed launch angle.
    """
    angle_rad = math.radians(LAUNCH_ANGLE_DEG)
    d = distance_to_hub_m - inches_to_meters(SHOOTER_OFFSET_INCHES)
    h = inches_to_meters(HUB_HEIGHT_INCHES) - inches_to_meters(HOOD_HEIGHT_INCHES)

    if d <= 0:
        return 0.0

    cos_t = math.cos(angle_rad)
    tan_t = math.tan(angle_rad)
    denom = 2.0 * cos_t * cos_t * (d * tan_t - h)

    if denom <= 0:
        return 0.0

    return mps_to_fps(d * math.sqrt(9.81 / denom))


# ── Main entry point ──────────────────────────────────────────────────────────

def run(table: ntcore.NetworkTable) -> None:
    """
    Blocking loop.  Call from main.py in a daemon thread.

    :param table: Pre-initialized NT table (e.g. inst.getTable("UltraShooter")).
                  main.py owns the NT connection so this module never calls
                  startClient4() itself.
    """

    dist_sub = table.getDoubleTopic("Avg Distance to Hub (m)").subscribe(0.0)
    vel_pub  = table.getDoubleTopic("Pi Physics Velocity ft/s").publish()
    hb_pub   = table.getIntegerTopic("Pi Heartbeat").publish()

    heartbeat: int = 0
    print("Physics co-processor: running at 50 Hz.")

    while True:
        vel_pub.set(calculate_velocity_fps(dist_sub.get()))
        heartbeat += 1
        hb_pub.set(heartbeat)
        time.sleep(0.02)   # 50 Hz — matches the roboRIO scheduler period
