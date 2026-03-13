"""
Robot Health Watchdog  —  Team 539
=====================================
Runs as a module inside main.py.  Monitors critical robot health metrics
published by the roboRIO via AdvantageKit and publishes a live summary to
NetworkTables for the Elastic dashboard.  Also writes a pre-match health
report to /home/pi/health/ the first time a valid battery reading arrives.

Monitors
--------
  Battery voltage   — LOW < 12.4 V,  CRITICAL < 11.5 V

  Loop time         — warn if roboRIO CommandScheduler loop exceeds 25 ms
  Motor currents    — warn if any flywheel motor exceeds 60 A
  Pi module heartbeats — staleness check for each daemon thread

NetworkTables reads (from /AdvantageKit/*)
-------------------------------------------
  Robot/BatteryVoltage_V          Double  bus voltage
  LoopTime/ms                     Double  scheduler loop time
  UltraShooter/PrimaryCurrent_A   Double  flywheel motor 1 current
  UltraShooter/SecondaryCurrent_A Double  flywheel motor 2 current
  UltraShooter/TertiaryCurrent_A  Double  flywheel motor 3 current

NetworkTables reads (Pi heartbeats, direct tables)
---------------------------------------------------
  UltraShooter/Pi Heartbeat   Integer  physics engine
  BallDetection/Heartbeat     Integer  ball detector
  PiVision/Heartbeat          Integer  AprilTag vision
  BumpTuner/Status/Heartbeat  Integer  bump tuner

NetworkTables publishes (table: "RobotHealth")
-----------------------------------------------
  BatteryVoltage_V   Double  last sampled bus voltage
  BatteryStatus      String  "OK" | "LOW" | "CRITICAL"
  LoopTimeOk         Boolean roboRIO loop time is within limit
  MaxMotorCurrent_A  Double  highest current across flywheel motors (A)
  PiModulesOk        Boolean all Pi modules reporting fresh heartbeats
  Status             String  one-line summary for Elastic ("OK" or fault list)
  Heartbeat          Integer increments every cycle — watchdog alive check

Health report: /home/pi/health/prematch_YYYYMMDD_HHMMSS.txt
"""

import os
import time
import ntcore

from datetime import datetime

HEALTH_DIR = "/home/pi/health"

# ── Thresholds ─────────────────────────────────────────────────────────────────
BATT_LOW_V       = 12.4   # warn below this voltage
BATT_CRITICAL_V  = 11.5   # critical below this voltage
LOOP_WARN_MS     = 25.0   # roboRIO loop time warning threshold (ms)
MOTOR_WARN_A     = 60.0   # sustained flywheel motor current warning (A)
HB_STALE_CYCLES  = 10     # missed heartbeat cycles before module is flagged dead

WATCHDOG_HZ = 2.0   # 2 Hz — negligible CPU load


def run(table: ntcore.NetworkTable) -> None:
    """
    Blocking loop.  Call from main.py in a daemon thread.

    :param table: Pre-initialized NT table ("RobotHealth") — used for
                  publishing health outputs and obtaining the shared instance.
    """
    inst    = table.getInstance()
    ak      = inst.getTable("AdvantageKit")
    us      = ak.getSubTable("UltraShooter")
    robot_t = ak.getSubTable("Robot")
    loop_t  = ak.getSubTable("LoopTime")

    # ── Inputs — roboRIO health ────────────────────────────────────────────────
    batt_sub  = robot_t.getDoubleTopic("BatteryVoltage_V").subscribe(0.0)
    loop_sub  = loop_t.getDoubleTopic("ms").subscribe(0.0)
    cur1_sub  = us.getDoubleTopic("PrimaryCurrent_A").subscribe(0.0)
    cur2_sub  = us.getDoubleTopic("SecondaryCurrent_A").subscribe(0.0)
    cur3_sub  = us.getDoubleTopic("TertiaryCurrent_A").subscribe(0.0)

    # ── Inputs — Pi module heartbeats ─────────────────────────────────────────
    pi_hb_subs = {
        "physics":  inst.getTable("UltraShooter").getIntegerTopic("Pi Heartbeat").subscribe(-1),
        "ball":     inst.getTable("BallDetection").getIntegerTopic("Heartbeat").subscribe(-1),
        "aptag":    inst.getTable("PiVision").getIntegerTopic("Heartbeat").subscribe(-1),
        "bumptune": inst.getTable("BumpTuner").getSubTable("Status").getIntegerTopic("Heartbeat").subscribe(-1),
    }

    # ── Outputs ────────────────────────────────────────────────────────────────
    batt_v_pub  = table.getDoubleTopic("BatteryVoltage_V").publish()
    batt_s_pub  = table.getStringTopic("BatteryStatus").publish()
    loop_ok_pub = table.getBooleanTopic("LoopTimeOk").publish()
    max_cur_pub = table.getDoubleTopic("MaxMotorCurrent_A").publish()
    pi_ok_pub   = table.getBooleanTopic("PiModulesOk").publish()
    status_pub  = table.getStringTopic("Status").publish()
    hb_pub      = table.getIntegerTopic("Heartbeat").publish()

    last_hb      = {name: -1 for name in pi_hb_subs}
    stale_counts = {name:  0 for name in pi_hb_subs}
    heartbeat        = 0
    prematch_written = False

    os.makedirs(HEALTH_DIR, exist_ok=True)
    print("Health watchdog: running at 2 Hz.")

    while True:
        batt    = batt_sub.get()
        loop_ms = loop_sub.get()
        cur1    = cur1_sub.get()
        cur2    = cur2_sub.get()
        cur3    = cur3_sub.get()
        max_cur = max(cur1, cur2, cur3)

        # ── Battery status ─────────────────────────────────────────────────────
        if batt > 0.1 and batt < BATT_CRITICAL_V:
            batt_status = "CRITICAL"
        elif batt > 0.1 and batt < BATT_LOW_V:
            batt_status = "LOW"
        else:
            batt_status = "OK"

        # ── Loop time ──────────────────────────────────────────────────────────
        loop_ok = loop_ms == 0.0 or loop_ms < LOOP_WARN_MS   # 0.0 = not yet published

        # ── Pi module heartbeat staleness ──────────────────────────────────────
        for name, sub in pi_hb_subs.items():
            hb = sub.get()
            if hb == last_hb[name]:
                stale_counts[name] += 1
            else:
                stale_counts[name] = 0
            last_hb[name] = hb

        pi_ok        = all(s < HB_STALE_CYCLES for s in stale_counts.values())
        dead_modules = [n for n, s in stale_counts.items() if s >= HB_STALE_CYCLES]

        # ── Overall status string ──────────────────────────────────────────────
        issues = []
        if batt_status != "OK":
            issues.append(f"BATT {batt_status} ({batt:.1f}V)")
        if not loop_ok:
            issues.append(f"SLOW_LOOP({loop_ms:.0f}ms)")
        if max_cur > MOTOR_WARN_A:
            issues.append(f"HIGH_CUR({max_cur:.0f}A)")
        if dead_modules:
            issues.append(f"PI_DEAD:{','.join(dead_modules)}")
        status = "OK" if not issues else " | ".join(issues)

        # ── Publish ────────────────────────────────────────────────────────────
        batt_v_pub.set(batt)
        batt_s_pub.set(batt_status)
        loop_ok_pub.set(loop_ok)
        max_cur_pub.set(max_cur)
        pi_ok_pub.set(pi_ok)
        status_pub.set(status)
        heartbeat += 1
        hb_pub.set(heartbeat)

        # ── Pre-match report (written once when battery first reads valid) ──────
        if not prematch_written and batt > 0.1:
            fname = os.path.join(
                HEALTH_DIR, datetime.now().strftime("prematch_%Y%m%d_%H%M%S.txt"))
            with open(fname, "w") as f:
                f.write("=== Team 539 Pre-Match Health Report ===\n")
                f.write(f"Time:             {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Battery:          {batt:.2f} V  ({batt_status})\n")
                f.write(f"Loop time:        {loop_ms:.1f} ms\n")
                f.write(f"Max motor curr:   {max_cur:.1f} A\n")
                f.write(f"Pi modules:       {'OK' if pi_ok else 'DEGRADED — ' + str(dead_modules)}\n")
                f.write(f"Overall status:   {status}\n")
            print(f"Health watchdog: pre-match report written to {fname}")
            prematch_written = True

        time.sleep(1.0 / WATCHDOG_HZ)
