"""
Bump Detection Parameter Tuner  —  Team 539
============================================
Runs as a module inside main.py.  On startup it loads saved parameters from
/home/pi/bump_tuner.json (creating the file with defaults if it does not
exist yet), publishes them to the BumpTuner NetworkTable, and saves them back
whenever the robot code sends a save command.

This lets engineers iterate on bump-detection thresholds from Elastic or
Shuffleboard during practice without recompiling the Java code.

NetworkTables layout  (table: "BumpTuner")
-------------------------------------------
  Params/BumpPitchThresholdDeg      double  — pitch angle (deg) above which a bump is active
  Params/BumpAccelZDeviation        double  — Pigeon Z-accel deviation (g) for bump detection
  Params/LimelightAccelZDeviation   double  — Limelight Z-accel deviation (g) for bump confirmation
  Params/WheelSlipThresholdMps2     double  — encoder-vs-IMU accel mismatch (m/s²) for slip detection
  Params/HighConfidenceThreshold    double  — min vision confidence to trust Limelight during slip
  Params/SlipHighConfMultiplier     double  — stdDev multiplier when slipping + high confidence (< 1)
  Params/BumpVisionMultiplier       double  — stdDev multiplier when slipping + low confidence (> 1)
  Cmd/Save                          boolean — Java sets True → Pi saves current values + resets
  Status/Heartbeat                  integer — increments every loop for staleness detection
  Status/SavedOk                    boolean — True after the most recent save succeeded
  Status/ConfigFile                 string  — absolute path of the active config file on the Pi

Persistent storage
-------------------
/home/pi/bump_tuner.json  — written on first boot (defaults) and on every
                            save command.  Survives Pi reboots.
"""

import json
import os
import time
import ntcore

CONFIG_FILE = "/home/pi/bump_tuner.json"

# ── Default parameter values (mirrors Java Constants.BumpDetectionConstants) ──
DEFAULTS: dict[str, float] = {
    "BumpPitchThresholdDeg":    8.0,
    "BumpAccelZDeviation":      0.25,
    "LimelightAccelZDeviation": 0.25,
    "WheelSlipThresholdMps2":   6.0,
    "HighConfidenceThreshold":  1.0,
    "SlipHighConfMultiplier":   0.5,
    "BumpVisionMultiplier":     10.0,
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
            print(f"[BumpTuner] Loaded config from {CONFIG_FILE}")
        except Exception as exc:
            print(f"[BumpTuner] Failed to load config ({exc}), using defaults")
    else:
        print(f"[BumpTuner] No config file found — writing defaults to {CONFIG_FILE}")
        _save_config(config)  # create on first boot

    return config


def _save_config(config: dict[str, float]) -> bool:
    """Writes *config* to disk as pretty-printed JSON.  Returns True on success."""
    try:
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        with open(CONFIG_FILE, "w") as f:
            json.dump(config, f, indent=2)
        print(f"[BumpTuner] Saved config to {CONFIG_FILE}")
        return True
    except Exception as exc:
        print(f"[BumpTuner] Save failed: {exc}")
        return False


# ── Main entry point ───────────────────────────────────────────────────────────

def run(table: ntcore.NetworkTable) -> None:
    """
    Blocking loop.  Called from main.py in a daemon thread.

    :param table: Pre-initialized NT table (e.g. inst.getTable("BumpTuner")).
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

    print("[BumpTuner] Running — listening for save commands")

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
