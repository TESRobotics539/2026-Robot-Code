#!/usr/bin/env python3
"""
WPILibPi Startup Script  —  Team 539
======================================
Deploy THIS file to the Raspberry Pi as the single startup application.
It initializes NetworkTables once, then launches both co-processors in
parallel daemon threads so each runs at its own rate independently.

Deploy steps
------------
1. Open the WPILibPi web dashboard:
      http://wpilibpi.local   (or http://10.5.39.XX on the field network)
2. Go to "Application" → set mode to "Uploaded Python file".
3. Upload THIS file (main.py).
   Also upload ball_detector.py, physics_coprocessor.py, shooter_tuner.py,
   apriltag_vision.py, bump_tuner.py, and trajectory_visualizer.py to the
   same directory — use the web dashboard's "File Upload" section.
4. Click Save.  The Pi will run main.py automatically on every boot.

Directory layout on the Pi
---------------------------
  /home/pi/
    main.py                  ← this file (set as the startup script)
    physics_coprocessor.py   ← physics module
    ball_detector.py         ← vision module

Camera streams
--------------
  Ball detection feed:       http://10.5.39.XX:1182/stream.mjpg
  AprilTag fallback feed:    http://10.5.39.XX:1183/stream.mjpg
  Add these URLs to Elastic as camera widgets.
  (The shot trajectory is rendered as a native Mechanism2d widget on the RoboRIO.)
"""

import threading
import ntcore

import physics_coprocessor
import ball_detector
import apriltag_vision
import bump_tuner
import shooter_tuner
# trajectory_visualizer is available as a Pi-side MJPEG debug tool but is not
# started by default — the RoboRIO's Mechanism2d widget is used instead.

TEAM_NUMBER = 539


def main() -> None:
    # ── NetworkTables — single shared connection for all modules ──────────────
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("frc539-pi")
    inst.setServerTeam(TEAM_NUMBER)
    inst.startDSClient()   # also accepts the DS-forwarded address at competitions

    # Each module gets its own NT subtable to keep entries organised.
    physics_table       = inst.getTable("UltraShooter")   # shares the shooter table
    ball_table          = inst.getTable("BallDetection")
    apriltag_table      = inst.getTable("PiVision")
    bump_tuner_table    = inst.getTable("BumpTuner")
    shooter_tuner_table = inst.getTable("ShooterTuner")
    # ── Launch co-processors in daemon threads ─────────────────────────────────
    # Daemon threads are killed automatically when the main process exits,
    # so a crash in one module doesn't hang the Pi.

    threads = [
        threading.Thread(
            target=physics_coprocessor.run,
            args=(physics_table,),
            name="physics",
            daemon=True,
        ),
        threading.Thread(
            target=ball_detector.run,
            args=(ball_table,),
            name="ball-vision",
            daemon=True,
        ),
        threading.Thread(
            target=apriltag_vision.run,
            args=(apriltag_table,),
            name="apriltag-vision",
            daemon=True,
        ),
        threading.Thread(
            target=bump_tuner.run,
            args=(bump_tuner_table,),
            name="bump-tuner",
            daemon=True,
        ),
        threading.Thread(
            target=shooter_tuner.run,
            args=(shooter_tuner_table,),
            name="shooter-tuner",
            daemon=True,
        ),
    ]

    for t in threads:
        t.start()
        print(f"Started thread: {t.name}")

    # Keep the main thread alive — if either worker crashes it can be
    # restarted here without rebooting the Pi.
    for t in threads:
        t.join()


if __name__ == "__main__":
    main()
