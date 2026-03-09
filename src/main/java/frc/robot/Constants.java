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

    /** Trigger hold duration that separates a short press from a long press (seconds). */
    public static final double kLongPressThresholdSeconds = 0.5;
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

    /** Fraction of the distance-based map RPM to hold during pre-spin (0.0–1.0). */
    public static final double kPreSpinFraction = 0.60;

    /** Flywheel speed for the close-range dump shot (RPM). */
    public static final double kDumpShotFlywheelRPM = 1850;
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
