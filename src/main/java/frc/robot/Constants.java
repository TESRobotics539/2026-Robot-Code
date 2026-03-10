package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final double DEADBAND = 0.05;
  }

  // ── Drivetrain ────────────────────────────────────────────────────────────
  public static class DrivetrainConstants {
    /** Maximum drivetrain translation speed (meters per second). */
    public static final double kMaxSpeed = Units.feetToMeters(17);
  }

  // ── Intake ────────────────────────────────────────────────────────────────
  public static class IntakeConstants {
    /** If true, the intake is locked stowed from the start of autonomous through the end of the match. */
    public static final boolean kStowIntakeForMatch = false;

    /** Maximum seconds between two trigger pulls to count as a double-tap. */
    public static final double kDoubleTapWindowSeconds = 0.4;
    /** Roller target speed when running (RPM). */
    public static final double kRollerRPM = 6500.0;
    /** Roller current (amps) threshold that indicates a fuel pickup spike. */
    public static final double kRollerLoadCurrentThreshold = 25.0;
    /** Number of current spikes required before flywheel pre-spin is enabled in teleop. */
    public static final int kRollerFuelSpikeCount = 4;
    /** Seconds the roller can run without a load spike before being automatically cut off. */
    public static final double kRollerNoLoadTimeoutSeconds = 15.0;
    /** Pivot motor current (amps) that indicates the intake has reached the deployed hard stop. */
    public static final double kPivotDeployedCurrentThreshold = 30.0;
    /** Minimum encoder travel (rotations) before the deployed hard-stop current spike is checked. */
    public static final double kPivotDeployedTravelThreshold = 0.2;
    /** Seconds the current must stay above threshold to confirm the deployed hard stop (debounce). */
    public static final double kPivotDeployedCurrentDebounceSeconds = 0.08;

    // Agitation pattern during shooting
    /** Pivot percent output going up during agitation. */
    public static final double kAgitateUpPower = 0.25;
    /** Duration of the upward agitation pulse (seconds). */
    public static final double kAgitateUpSeconds = 0.33;
    /** Pivot percent output going down during agitation (negative = down). */
    public static final double kAgitateDownPower = -0.05;
    /** Duration of the downward agitation pulse (seconds). */
    public static final double kAgitateDownSeconds = 0.2;

    // Pivot positions (absolute encoder, 0.0–1.0)
    public static final double kStowedPosition   = 0.8;
    public static final double kDeployedPosition = 0.57;
    public static final double kMinPosition       = 0.53;
    public static final double kMaxPosition       = 0.9;
  }

  // ── Shooter ───────────────────────────────────────────────────────────────
  public static class ShooterConstants {
    /** Seconds to wait after shooter reaches speed (and aim is confirmed) before feeding the fuel. */
    public static final double kShootWaitSeconds = 0.5;
    /** Maximum seconds to wait for the shooter to reach speed before giving up and feeding anyway. */
    public static final double kShootReadyTimeoutSeconds = 1.33;
    /** Delay before floor motor starts feeding, so the feeder gets the fuel first. */
    public static final double kFloorFeedDelaySeconds = 0.25;

    /** Fraction of the distance-based map speed to hold during pre-spin (0.0–1.0). */
    public static final double kPreSpinFraction = 0.60;

    /** Flywheel speed for the close-range dump shot (ft/s, converted from 1850 RPM with 4" wheel). */
    public static final double kDumpShotFlywheelSpeed = 32.0;
  }

  // ── Hanger ────────────────────────────────────────────────────────────────
  public static class HangerConstants {
    /** Percent output for manual d-pad down control. */
    public static final double kManualDownPower = -0.8;
    /** Percent output for manual d-pad up control. */
    public static final double kManualUpPower = 0.8;
    /** Primary current limit (amps) for the hanger motor. */
    public static final int kSmartCurrentLimit = 70;
    /** Secondary (backup) current limit (amps) for the hanger motor. */
    public static final int kSecondaryCurrentLimit = 120;
    /** Percent output for the initial autonomous climb (full speed). */
    public static final double kAutoClimbFullPower = -0.95;
    /** Current threshold (amps) that indicates the climber has reached a hard stop during auto. */
    public static final double kAutoClimbCurrentThreshold = 50.0;
    /** Seconds the current must stay above threshold before stall is confirmed (debounce). */
    public static final double kAutoClimbCurrentDebounceSeconds = 0.1;
    /** Percent output for the release pulse after the auto climb stalls (opposite direction). */
    public static final double kAutoClimbReleasePower = 0.5;
    /** Duration of the release pulse after the auto climb stalls (seconds). */
    public static final double kAutoClimbReleaseSeconds = 0.3;
  }

  // ── UltraShooter Physics ──────────────────────────────────────────────────
  public static class UltraShooterConstants {
    // ── Geometry ───────────────────────────────────────────────────────────

    /**
     * Height of the hood exit (ball launch point) above the floor (inches).
     * Measure from the ground to the center of the shooter barrel exit.
     */
    public static final double kHoodHeightFromFloorInches = 27.0;

    /**
     * Height of the hub center (scoring target) above the floor (inches).
     * Look this up from the game manual or measure on the field.
     */
    public static final double kHubCenterHeightFromFloorInches = 96.0; // TODO: confirm from game manual

    /**
     * Horizontal distance from the robot center to the shooter exit point,
     * measured toward the hub (inches). Subtracted from the odometry
     * distance so the physics model uses the true shooter-to-hub range.
     */
    public static final double kShooterCenterlineOffsetInches = 8.0;

    /**
     * Fixed launch angle above horizontal (degrees). With a static hood this
     * is a constant — tune to match the actual shooter barrel angle.
     */
    public static final double kLaunchAngleDegrees = 75.0;

    // ── Motor / PID (mirrors ShooterOrca values — adjust if hardware differs) ─
    public static final int kSmartCurrentLimit    = 60;
    public static final int kFreeCurrentLimit     = 40;
    public static final int kStatorCurrentLimit   = 120;

    public static final double kP = 0.003;
    public static final double kI = 0.000;
    public static final double kD = 0.25;
    public static final double kS = 0.15;

    /** Setpoint ramp-up rate (ft/s per 20 ms cycle). */
    public static final double kRampUpRate   = 200.0 * (Math.PI * (4.0 / 12.0) / 60.0);
    /** Setpoint ramp-down rate (ft/s per 20 ms cycle). */
    public static final double kRampDownRate = 400.0 * (Math.PI * (4.0 / 12.0) / 60.0);

    /** Rolling-average window for encoder noise filtering (samples at 50 Hz). */
    public static final int kVelocityAvgSamples = 8;

    /** Flywheel is "ready" within this tolerance of target (ft/s). */
    public static final double kReadyTolerance = 100.0 * (Math.PI * (4.0 / 12.0) / 60.0);

    /** Fraction of the physics-calculated speed used for pre-spin (0.0–1.0). */
    public static final double kPreSpinFraction = 0.60;

    // ── Fine-tune offsets ──────────────────────────────────────────────────
    // Added on top of the physics-calculated flywheel speed as a percentage.
    // The offset is linearly blended between the near (20 in) and far (120 in)
    // reference distances. Positive = spin faster, negative = spin slower.
    //
    // Example: kNearShotOffsetPercent = 5.0, kFarShotOffsetPercent = 10.0
    //   → +5 % added at 20 in
    //   → +7.5 % added at 70 in  (midpoint)
    //   → +10 % added at 120 in
    // The offset is clamped — distances outside 20–120 in use the nearest value.

    /** Speed offset (%) applied at the near reference distance (20 in). */
    public static final double kNearShotOffsetPercent = 0.0; // tune me

    /** Speed offset (%) applied at the far reference distance (120 in). */
    public static final double kFarShotOffsetPercent  = 0.0; // tune me
  }

  // ── Hood ──────────────────────────────────────────────────────────────────
  public static class HoodConstants {
    /** Minimum servo position (0.0–1.0) — prevents over-retraction. */
    public static final double kMinPosition = 0.06;
    /** Maximum servo position (0.0–1.0) — prevents over-extension. */
    public static final double kMaxPosition = 0.95;
    /** Distance at which hood tracking begins interpolating (meters). */
    public static final double kTrackingMinDistanceMeters = 1.0;
    /** Distance at which hood tracking reaches its maximum angle (meters). */
    public static final double kTrackingMaxDistanceMeters = 5.0;
    /** Hood position at minimum tracking distance. */
    public static final double kTrackingMinPosition = 0.05;
    /** Hood position at maximum tracking distance. */
    public static final double kTrackingMaxPosition = 0.85;
  }
}
