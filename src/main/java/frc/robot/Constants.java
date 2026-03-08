package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

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
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.05;
  }

  // ── Drivetrain ────────────────────────────────────────────────────────────
  /** Maximum drivetrain translation speed (meters per second). */
  public static final double maxSpeed = Units.feetToMeters(12);

  // ── Shoot timing ─────────────────────────────────────────────────────────
  /** Seconds to wait after shooter reaches speed before feeding the note. */
  public static final double shootWaitSeconds = 1.1;
  /** Maximum seconds to wait for the shooter to reach speed before giving up and feeding anyway. */
  public static final double shootReadyTimeoutSeconds = 1.33;
  /** Delay before floor motor starts feeding, so the feeder gets the note first. */
  public static final double floorFeedDelaySeconds = 0.25;

  // ── Intake ────────────────────────────────────────────────────────────────
  public static class IntakeConstants {
    /** NEO Vortex free speed (used for velocity scaling). */
    public static final AngularVelocity kMaxSpeed = RPM.of(6784);

    /** Trigger hold duration that separates a short press from a long press (seconds). */
    public static final double kLongPressThresholdSeconds = 0.5;
    /** Roller percent output when running. */
    public static final double kRollerSpeed = 1.0;

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

  // ── Hanger ────────────────────────────────────────────────────────────────
  public static class HangerConstants {
    /** Encoder rotations traveled per toggle cycle (up or down). */
    public static final double kToggleDistanceRotations = 200.0;
    /** Percent output applied when holding the hanger down (X button). */
    public static final double kHoldDownPower = -0.3;
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
    /** Percent output after stall is detected during autonomous climb (hold). */
    public static final double kAutoClimbHoldPower = -0.5;
    /** Current threshold (amps) that indicates the climber has reached a hard stop during auto. */
    public static final double kAutoClimbCurrentThreshold = 50.0;
    /** Seconds the current must stay above threshold before stall is confirmed (debounce). */
    public static final double kAutoClimbCurrentDebounceSeconds = 0.1;
    /** Percent output for the release pulse after the auto climb stalls (opposite direction). */
    public static final double kAutoClimbReleasePower = 0.5;
    /** Duration of the release pulse after the auto climb stalls (seconds). */
    public static final double kAutoClimbReleaseSeconds = 0.3;
  }

  // ── Shooter ───────────────────────────────────────────────────────────────
  public static class ShooterConstants {
    /** Smart current limit (amps) applied to each flywheel motor at stall. */
    public static final int kSmartCurrentLimit = 60;
    /** Free-speed current limit (amps) applied to each flywheel motor. */
    public static final int kFreeCurrentLimit = 40;
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
