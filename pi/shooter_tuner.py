"""
Shooter Parameter Tuner  —  Team 539
=====================================
Runs as a module inside main.py.  On startup it loads saved parameters from
/home/pi/shooter_tuner.json (creating the file with defaults if it does not
exist yet), publishes them to the ShooterTuner NetworkTable, and saves them
back whenever the robot code sends a save command.

This lets engineers iterate on shooter parameters from Elastic or Shuffleboard
during practice without recompiling the Java code.

NetworkTables layout  (table: "ShooterTuner")
----------------------------------------------
  Params/NearShotOffsetPercent    double  — percent offset applied to flywheel speed for near shots
  Params/FarShotOffsetPercent     double  — percent offset applied to flywheel speed for far shots
  Params/ReadyTolerance           double  — ft/s tolerance band around target speed to consider shooter ready
  Params/ShootReadyTimeoutSeconds double  — seconds to wait for shooter to reach ready speed before giving up
  Params/FloorFeedDelaySeconds    double  — seconds to delay floor roller engagement after feeder starts
  Params/Kp                       double  — flywheel proportional gain (written by ShooterAutoTuner)
  Params/FlywheelEfficiency       double  — ball-to-flywheel velocity ratio (0–1); tune to match field data
  Params/DragCoefficient          double  — aero drag constant B = 0.5×Cd×rho×A (kg/m); 0 disables drag
  Params/BallMassKg               double  — ball mass (kg); used in drag deceleration term B/m
  Cmd/Save                        boolean — Java sets True → Pi saves current values + resets
  Status/Heartbeat                integer — increments every loop for staleness detection
  Status/SavedOk                  boolean — True after the most recent save succeeded
  Status/ConfigFile               string  — absolute path of the active config file on the Pi

Persistent storage
-------------------
/home/pi/shooter_tuner.json  — written on first boot (defaults) and on every
                               save command.  Survives Pi reboots.
"""

import json
import os
import time
import ntcore

CONFIG_FILE = "/home/pi/shooter_tuner.json"

# ── Default parameter values (mirrors Java Constants.ShooterConstants /
#                              Constants.UltraShooterConstants) ──────────────
DEFAULTS: dict[str, float] = {
    "CloseShotOffsetPercent":   0.0,     # parabolic anchor at 1 m
    "MidShotOffsetPercent":     0.0,     # parabolic anchor at 4 m
    "FarShotOffsetPercent":     0.0,     # parabolic anchor at 7 m
    "ReadyTolerance":           1.745,   # ft/s — 100 RPM converted
    "ShootReadyTimeoutSeconds": 1.33,
    "FloorFeedDelaySeconds":    0.25,
    "Kp":                       0.003,   # flywheel proportional gain (autotuned)
    "FlywheelEfficiency":       0.49,    # ball exit speed / flywheel surface speed (0–1); calibrated vs ShooterOrca 2–5 m
    "DragCoefficient":          0.0132,  # B = 0.5×Cd×rho×A (kg/m); 0 = disable drag
    "BallMassKg":               0.270,   # ball mass (kg) for drag term B/m
}


# ── Helpers ────────────────────────────────────────────────────────────────────

def _load_or_create_config() -> dict[str, float]:
    """
    Loads the config from disk if it exists, filling any missing keys with
    defaults.  If no file exists, writes the defaults to disk immediately so
    permanent storage is established on first boot.
    """
    config = dict(DEFAULTS)

    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                saved = json.load(f)
            # Accept only keys that are in DEFAULTS; fill gaps with defaults.
            for key in DEFAULTS:
                if key in saved:
                    config[key] = float(saved[key])
            print(f"[ShooterTuner] Loaded config from {CONFIG_FILE}")
        except Exception as exc:
            print(f"[ShooterTuner] Failed to load config ({exc}), using defaults")
    else:
        print(f"[ShooterTuner] No config file found — writing defaults to {CONFIG_FILE}")
        _save_config(config)  # create on first boot

    return config


def _save_config(config: dict[str, float]) -> bool:
    """Writes *config* to disk as pretty-printed JSON.  Returns True on success."""
    try:
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        with open(CONFIG_FILE, "w") as f:
            json.dump(config, f, indent=2)
        print(f"[ShooterTuner] Saved config to {CONFIG_FILE}")
        return True
    except Exception as exc:
        print(f"[ShooterTuner] Save failed: {exc}")
        return False


# ── Main entry point ───────────────────────────────────────────────────────────

def run(table: ntcore.NetworkTable) -> None:
    """
    Blocking loop.  Called from main.py in a daemon thread.

    :param table: Pre-initialized NT table (e.g. inst.getTable("ShooterTuner")).
    """
    params_table = table.getSubTable("Params")
    cmd_table    = table.getSubTable("Cmd")
    status_table = table.getSubTable("Status")

    # ── Load persistent config ─────────────────────────────────────────────────
    config = _load_or_create_config()

    # ── NT publishers for each parameter ──────────────────────────────────────
    param_pubs: dict[str, ntcore.DoublePublisher] = {}
    for key in DEFAULTS:
        pub = params_table.getDoubleTopic(key).publish()
        pub.set(config[key])
        param_pubs[key] = pub

    # ── NT subscribers — read values changed by the dashboard ─────────────────
    param_subs: dict[str, ntcore.DoubleSubscriber] = {}
    for key in DEFAULTS:
        param_subs[key] = params_table.getDoubleTopic(key).subscribe(config[key])

    # ── Save command ───────────────────────────────────────────────────────────
    # Java writes True to trigger a save; Pi resets it to False after handling.
    save_sub = cmd_table.getBooleanTopic("Save").subscribe(False)
    save_pub = cmd_table.getBooleanTopic("Save").publish()
    save_pub.set(False)

    # ── Status entries ─────────────────────────────────────────────────────────
    hb_pub        = status_table.getIntegerTopic("Heartbeat").publish()
    saved_ok_pub  = status_table.getBooleanTopic("SavedOk").publish()
    cfg_file_pub  = status_table.getStringTopic("ConfigFile").publish()

    cfg_file_pub.set(CONFIG_FILE)
    saved_ok_pub.set(os.path.exists(CONFIG_FILE))

    print("[ShooterTuner] Running — listening for save commands")

    heartbeat = 0
    while True:
        heartbeat += 1
        hb_pub.set(heartbeat)

        # ── Handle save command ────────────────────────────────────────────────
        if save_sub.get():
            # Snapshot the current NT values (may differ from what was loaded).
            for key in DEFAULTS:
                config[key] = param_subs[key].get()
            ok = _save_config(config)
            saved_ok_pub.set(ok)
            save_pub.set(False)  # acknowledge by resetting the flag

        time.sleep(0.02)  # 50 Hz — same cadence as other Pi modules
