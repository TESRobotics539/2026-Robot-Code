"""
Match Data Logger  —  Team 539
================================
Runs as a module inside main.py.  Subscribes to key NetworkTables entries
published by the roboRIO (via AdvantageKit's NT4Publisher) and writes a
timestamped CSV to /home/pi/logs/ at 10 Hz for every session.  A new file
is created each time main.py starts, so every match has its own log.

Log files survive roboRIO crashes and are accessible after the match by
connecting a laptop to the Pi's Wi-Fi or pulling the log via ssh/scp.

NetworkTables (reads from /AdvantageKit/*)
------------------------------------------
  UltraShooter/Velocity_fps       Double  live flywheel surface speed (ft/s)
  UltraShooter/Target_fps         Double  speed setpoint (ft/s)
  UltraShooter/PrimaryCurrent_A   Double  primary flywheel motor current (A)
  UltraShooter/SecondaryCurrent_A Double  secondary flywheel motor current (A)
  UltraShooter/TertiaryCurrent_A  Double  tertiary flywheel motor current (A)
  UltraShooter/PiActive           Boolean Pi physics engine heartbeat fresh
  Intake/rollerCurrentAmps        Double  intake roller motor current (A)
  Robot/BatteryVoltage_V          Double  RoboRIO bus voltage
  Robot/MatchTime_s               Double  remaining match time (s); -1 = unknown
  Robot/Enabled                   Boolean robot enabled state
  LoopTime/ms                     Double  CommandScheduler loop time (ms)

Log directory: /home/pi/logs/
Log file name: match_YYYYMMDD_HHMMSS.csv
"""

import csv
import os
import time
import ntcore

from datetime import datetime

LOG_DIR = "/home/pi/logs"
LOG_HZ  = 10   # rows per second


def run(table: ntcore.NetworkTable) -> None:
    """
    Blocking loop.  Call from main.py in a daemon thread.

    :param table: Any pre-initialized NT table — used only to obtain the
                  shared NetworkTableInstance so this module never calls
                  startClient4() itself.
    """
    inst = table.getInstance()

    # ── AdvantageKit subtables ─────────────────────────────────────────────────
    ak       = inst.getTable("AdvantageKit")
    us       = ak.getSubTable("UltraShooter")
    intake_t = ak.getSubTable("Intake")
    robot_t  = ak.getSubTable("Robot")
    loop_t   = ak.getSubTable("LoopTime")

    # ── Subscribe ──────────────────────────────────────────────────────────────
    vel_sub    = us.getDoubleTopic("Velocity_fps").subscribe(0.0)
    tgt_sub    = us.getDoubleTopic("Target_fps").subscribe(0.0)
    cur1_sub   = us.getDoubleTopic("PrimaryCurrent_A").subscribe(0.0)
    cur2_sub   = us.getDoubleTopic("SecondaryCurrent_A").subscribe(0.0)
    cur3_sub   = us.getDoubleTopic("TertiaryCurrent_A").subscribe(0.0)
    pi_sub     = us.getBooleanTopic("PiActive").subscribe(False)

    roller_sub = intake_t.getDoubleTopic("rollerCurrentAmps").subscribe(0.0)

    batt_sub   = robot_t.getDoubleTopic("BatteryVoltage_V").subscribe(0.0)
    mtime_sub  = robot_t.getDoubleTopic("MatchTime_s").subscribe(-1.0)
    enab_sub   = robot_t.getBooleanTopic("Enabled").subscribe(False)

    loop_sub   = loop_t.getDoubleTopic("ms").subscribe(0.0)

    # ── Create log directory and file ─────────────────────────────────────────
    os.makedirs(LOG_DIR, exist_ok=True)
    fname = os.path.join(LOG_DIR, datetime.now().strftime("match_%Y%m%d_%H%M%S.csv"))

    COLS = [
        "wall_time_s", "match_time_s", "enabled",
        "battery_V", "loop_ms",
        "shooter_vel_fps", "shooter_target_fps",
        "shooter_cur1_A", "shooter_cur2_A", "shooter_cur3_A",
        "pi_active", "roller_cur_A",
    ]

    with open(fname, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(COLS)
        print(f"Match logger: writing to {fname}")

        t0 = time.monotonic()
        while True:
            now = time.monotonic() - t0
            row = [
                f"{now:.2f}",
                f"{mtime_sub.get():.1f}",
                int(enab_sub.get()),
                f"{batt_sub.get():.3f}",
                f"{loop_sub.get():.2f}",
                f"{vel_sub.get():.2f}",
                f"{tgt_sub.get():.2f}",
                f"{cur1_sub.get():.2f}",
                f"{cur2_sub.get():.2f}",
                f"{cur3_sub.get():.2f}",
                int(pi_sub.get()),
                f"{roller_sub.get():.2f}",
            ]
            writer.writerow(row)
            f.flush()
            time.sleep(1.0 / LOG_HZ)
