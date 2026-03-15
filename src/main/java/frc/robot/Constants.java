package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
    /** Maximum drivetrain translation speed (meters per second). SDS MK4 L1 (8.14:1). */
    public static final double kMaxSpeed = Units.feetToMeters(13);

    /**
     * Maximum drive wheel linear acceleration used by the setpoint generator (m/s²).
     * Limits how aggressively modules accelerate between cycles to prevent wheel slip
     * and excessive current draw during rapid direction changes.
     */
    public static final double kMaxAccelerationMps2 = 15.0;

    /**
     * Maximum module azimuth angular velocity used by the setpoint generator (rad/s).
     * NEO at 5676 RPM through 21.43:1 steering gear → ~27.7 rad/s theoretical;
     * 75% of theoretical used as a practical cap to protect the steering motor.
     */
    public static final double kMaxSteeringVelocityRadPerSec = 20.0;
  }

  // ── Intake ────────────────────────────────────────────────────────────────
  public static class IntakeConstants {

    // ── General ──────────────────────────────────────────────────────────
    /** If true, the intake is locked stowed from the start of autonomous through the end of the match. */
    public static final boolean kStowIntakeForMatch = false;
    /** Maximum seconds between two trigger pulls to count as a double-tap. */
    public static final double kDoubleTapWindowSeconds = 0.4;
    /** Seconds the roller can run without a load spike before being automatically cut off. */
    public static final double kRollerNoLoadTimeoutSeconds = 15.0;

    // ── Roller ───────────────────────────────────────────────────────────
    public static final IdleMode kRollerIdleMode = IdleMode.kCoast;
    /** Roller target speed when running (RPM). */
    public static final double kRollerRPM = 6500.0;
    /** Roller current (amps) threshold that indicates a fuel pickup spike. */
    public static final double kRollerLoadCurrentThreshold = 25.0;
    /** Number of current spikes required before flywheel pre-spin is enabled in teleop. */
    public static final int kRollerFuelSpikeCount = 4;
    /** Free-spin RPM of the roller motor (used for FF calculation). */
    public static final double kRollerFreeSpeedRPM = 6784.0;
    /** Roller closed-loop PID gains. */
    public static final double kRollerP = 0.0002;
    public static final double kRollerI = 0.0;
    public static final double kRollerD = 0.0;

    // ── Pivot ────────────────────────────────────────────────────────────
    /** Pivot motor current (amps) that indicates the intake has reached the deployed hard stop. */
    public static final double kPivotDeployedCurrentThreshold = 30.0;
    /** Minimum encoder travel (rotations) before the deployed hard-stop current spike is checked. */
    public static final double kPivotDeployedTravelThreshold = 0.2;
    /** Seconds the current must stay above threshold to confirm the deployed hard stop (debounce). */
    public static final double kPivotDeployedCurrentDebounceSeconds = 0.08;
    /** Pivot closed-loop PID gains (absolute encoder feedback). */
    public static final double kPivotP              = 1.5;
    public static final double kPivotI              = 0.0;
    public static final double kPivotD              = 5.0;
    public static final double kPivotOutputRangeMin = -0.15;
    public static final double kPivotOutputRangeMax =  0.4;

    // ── Pivot positions (absolute encoder, 0.0–1.0) ──────────────────────
    public static final double kStowedPosition   = 0.8;
    public static final double kDeployedPosition = 0.57;
    public static final double kMinPosition       = 0.53;
    public static final double kMaxPosition       = 0.9;

    // ── Roller anti-jam ──────────────────────────────────────────────────
    /** Current (amps) above which a roller jam is suspected while intaking. */
    public static final double kRollerJamCurrentThreshold = 40.0;
    /** Velocity (RPM) below which a high-current reading is treated as a jam, not just spinup. */
    public static final double kRollerJamVelocityThresholdRPM = 1000.0;
    /** Reverse speed (RPM, magnitude) applied during the roller anti-jam pulse. */
    public static final double kRollerUnjamReverseRPM = 2500.0;
    /** Duration of the reverse pulse during roller anti-jam (seconds). */
    public static final double kRollerUnjamReverseSeconds = 0.34;
    /** Velocity (RPM) above which the roller is considered to be running freely after an unjam. */
    public static final double kRollerFreeVelocityThresholdRPM = 3000.0;

    // ── Agitation (pivot motion during shooting to settle fuel) ──────────
    /** Pivot percent output going up during agitation. */
    public static final double kAgitateUpPower = 0.25;
    /** Duration of the upward agitation pulse (seconds). */
    public static final double kAgitateUpSeconds = 0.33;
    /** Pivot percent output going down during agitation (negative = down). */
    public static final double kAgitateDownPower = -0.05;
    /** Duration of the downward agitation pulse (seconds). */
    public static final double kAgitateDownSeconds = 0.2;
  }

  // ── Feeder ────────────────────────────────────────────────────────────────
  public static class FeederConstants {

    // ── Motor ────────────────────────────────────────────────────────────
    /** Primary current limit (amps). */
    public static final int kSmartCurrentLimit = 50;
    public static final IdleMode kIdleMode = IdleMode.kCoast;

    // ── Setpoints ────────────────────────────────────────────────────────
    /** Feed target speed (RPM, positive = forward). */
    public static final double kFeedRPM    =  5500.0;
    /** Reverse target speed (RPM, negative = backward). */
    public static final double kReverseRPM = -5500.0;

    // ── PID ──────────────────────────────────────────────────────────────
    public static final double kP          = 0.0001;
    public static final double kI          = 0.0;
    public static final double kD          = 0.0;
    public static final double kVelocityFF = 0.000175;

    // ── Anti-jam ──────────────────────────────────────────────────────────
    /** Current (amps) above which a jam is suspected while feeding. */
    public static final double kJamCurrentThreshold = 35.0;
    /** Velocity (RPM) below which a high-current reading is treated as a jam, not just spinup. */
    public static final double kJamVelocityThresholdRPM = 1000.0;
    /** Reverse speed (RPM, magnitude) applied during the anti-jam pulse. */
    public static final double kUnjamReverseRPM = 2500.0;
    /** Duration of the reverse pulse during anti-jam (seconds). */
    public static final double kUnjamReverseSeconds = 0.5;
    /** Velocity (RPM) above which the feeder is considered to be running freely after an unjam. */
    public static final double kFreeVelocityThresholdRPM = 3000.0;
  }

  // ── Floor ─────────────────────────────────────────────────────────────────
  public static class FloorConstants {
    /** Percent output (0.0–1.0) when feeding fuel toward the shooter. */
    public static final double kFeedPercentOutput = 0.75;
    public static final IdleMode kIdleMode = IdleMode.kCoast;
  }

  // ── Shooter ───────────────────────────────────────────────────────────────
  public static class ShooterConstants {

    // ── Timing ───────────────────────────────────────────────────────────
    /** Seconds to wait after shooter reaches speed (and aim is confirmed) before feeding the fuel. */
    public static final double kShootWaitSeconds = 0.5;
    /** Maximum seconds to wait for the shooter to reach speed before giving up and feeding anyway. */
    public static final double kShootReadyTimeoutSeconds = 1.33;
    /** Delay before floor motor starts feeding, so the feeder gets the fuel first. */
    public static final double kFloorFeedDelaySeconds = 0.25;

    // ── Setpoints ────────────────────────────────────────────────────────
    /** Fraction of the distance-based map speed to hold during pre-spin (0.0–1.0). */
    public static final double kPreSpinFraction = 0.60;
    /** Flywheel speed for the close-range dump shot (ft/s, converted from 1850 RPM with 4" wheel). */
    public static final double kDumpShotFlywheelSpeed = 32.0;
  }

  // ── Hanger ────────────────────────────────────────────────────────────────
  public static class HangerConstants {

    // ── Motor ────────────────────────────────────────────────────────────
    /** Primary current limit (amps) for the hanger motor. */
    public static final int kSmartCurrentLimit = 70;
    public static final IdleMode kIdleMode = IdleMode.kCoast;
    /** Secondary (backup) current limit (amps) for the hanger motor. */
    public static final int kSecondaryCurrentLimit = 120;

    // ── Manual control ───────────────────────────────────────────────────
    /** Percent output for manual d-pad down control. */
    public static final double kManualDownPower = -0.8;
    /** Percent output for manual d-pad up control. */
    public static final double kManualUpPower = 0.8;
    /** Percent output for the driver A-button nudge. */
    public static final double kNudgePower = 0.5;
    /** Duration of the A-button nudge (seconds). */
    public static final double kNudgeSeconds = 0.33;

    // ── Auto climb ───────────────────────────────────────────────────────
    /** Percent output for the initial autonomous climb (full speed). */
    public static final double kAutoClimbFullPower = -0.95;
    /** Current threshold (amps) that indicates the climber has reached a hard stop during auto. */
    public static final double kAutoClimbCurrentThreshold = 50.0;
    /** Seconds the current must stay above threshold before stall is confirmed (debounce). */
    public static final double kAutoClimbCurrentDebounceSeconds = 0.1;
    /** Percent output for the release pulse after the auto climb stalls (opposite direction). */
    public static final double kAutoClimbReleasePower = 0.5;
    /** Duration of the release pulse after the auto climb stalls (seconds). */
    public static final double kAutoClimbReleaseSeconds = 0.4;

    // ── Homing ───────────────────────────────────────────────────────────
    /** Homing percent output (negative = retract toward hard stop). */
    public static final double kHomingPower = -0.05;
    /** Current threshold (amps) used to detect the homing hard stop. */
    public static final double kHomingCurrentThreshold = 7.0;
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
    public static final double kHubCenterHeightFromFloorInches = 72.0;

    /**
     * Horizontal distance from the robot center to the shooter (hood) exit
     * point (inches). The hood is at the BACK of the robot, so this value is
     * ADDED to the odometry distance (robot center → hub) to get the true
     * shooter-to-hub range used by the physics model.
     */
    public static final double kShooterCenterlineOffsetInches = 8.0;

    /**
     * Fixed launch angle above horizontal (degrees). With a static hood this
     * is a constant — tune to match the actual shooter barrel angle.
     */
    public static final double kLaunchAngleDegrees = 75.0;

    // ── Motor ────────────────────────────────────────────────────────────
    public static final int kSmartCurrentLimit  = 60;
    public static final int kFreeCurrentLimit   = 40;
    public static final int kStatorCurrentLimit = 120;

    // ── PID ──────────────────────────────────────────────────────────────
    public static final double kP = 0.003;
    public static final double kI = 0.000;
    public static final double kD = 0.25;
    public static final double kS = 0.15;

    // ── Flywheel behavior ─────────────────────────────────────────────────
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

    // ── Voltage bias (971-style integral flywheel) ────────────────────────
    /** Volts added to feedforward per ft/s of velocity error, per 20 ms cycle.
     *  Accumulates only after the ramp has reached target — no spinup windup. */
    public static final double kVoltageBiasRate = 0.001;
    /** Maximum absolute feedforward voltage bias (V). Caps the integrator. */
    public static final double kMaxVoltageBias  = 1.5;

    // ── Parabolic fine-tune offsets ───────────────────────────────────────
    // Three anchor points define a parabola (Lagrange quadratic) that is
    // added on top of the physics-calculated flywheel speed as a percentage.
    // Tune close/mid/far independently on the field; the parabola fills in
    // smoothly between anchors.  Distance is clamped to [3 ft, 12 ft, 20 ft].
    // Positive = spin faster, negative = spin slower.

    /** Distance (ft) of the close anchor. */
    public static final double kCloseShotAnchorFeet = 3.0;
    /** Distance (ft) of the mid anchor. */
    public static final double kMidShotAnchorFeet   = 12.0;
    /** Distance (ft) of the far anchor. */
    public static final double kFarShotAnchorFeet   = 20.0;

    /** Speed offset (%) applied at the close anchor (3 ft). */
    public static final double kCloseShotOffsetPercent = 0.0; // tune me
    /** Speed offset (%) applied at the mid anchor (12 ft). */
    public static final double kMidShotOffsetPercent   = 0.0; // tune me
    /** Speed offset (%) applied at the far anchor (20 ft). */
    public static final double kFarShotOffsetPercent   = 0.0; // tune me

    // ── Aerodynamics & efficiency ─────────────────────────────────────────

    /**
     * Flywheel-to-ball velocity transfer efficiency (0–1).
     * Accounts for energy lost in wheel compression and slip during contact.
     * Calibrated against ShooterOrca empirical map at 2 m (primary test distance):
     * implied η = 0.85 × (25.2 ft/s physics / 50.0 ft/s empirical) ≈ 0.43.
     * Tune on the field via ShooterTuner/Params/FlywheelEfficiency.
     * Applied as: ball_exit_speed = flywheel_surface_speed × efficiency.
     */
    public static final double kFlywheelEfficiency = 0.43;

    /**
     * Ball diameter (inches). Used to compute cross-sectional area for drag.
     * Change this when swapping game pieces; kDragCoefficient must be updated
     * accordingly: B = 0.5 x Cd x rho_air x pi x (diameter_m / 2)^2
     */
    public static final double kBallDiameterInches = 6.0;

    /**
     * Ball mass (lbs).  Used in the aerodynamic drag term (B / m).
     * 2026 game piece: adjust once the game manual publishes the spec.
     * Internally converted to kg for SI physics calculations.
     */
    public static final double kBallMassLbs = 0.595;

    /**
     * Aerodynamic drag constant B = 0.5 x Cd x rho_air x A_cross  (kg/m).
     * For a 6-inch-diameter foam sphere (kBallDiameterInches = 6.0):
     *   Cd ~= 0.47,  rho = 1.225 kg/m^3,  A = pi*(0.0762)^2 ~= 0.01824 m^2
     *   → B ≈ 0.00525 kg/m
     * Set to 0.0 to disable drag compensation and fall back to the
     * analytic (vacuum) projectile formula.
     */
    public static final double kDragCoefficient = 0.00525;

    // ── Close-range backup ─────────────────────────────────────────────────

    /**
     * If true, pressing the right trigger while within kCloseRangeThresholdInches
     * of the hub will first reverse the robot kCloseRangeBackupInches away from
     * the hub before beginning the aim-and-fire sequence.
     * Set false to disable entirely.
     */
    public static final boolean kEnableCloseRangeBackup = true;

    /** Hub centerline distance (inches) at or below which close-range backup activates. */
    public static final double kCloseRangeThresholdInches = 35.0;

    /** Distance (inches) to drive away from the hub before aiming when backup triggers. */
    public static final double kCloseRangeBackupInches = 12.0;

    /** Speed (ft/s) at which the robot drives away from the hub during backup. */
    public static final double kCloseRangeBackupSpeedFps = 3.0;
  }

  // ── Rumble ────────────────────────────────────────────────────────────────
  public static class RumbleConstants {

    // ── Fuel pickup (intake roller spike) ────────────────────────────────
    /** Rumble intensity for the fuel-pickup double-pulse (0.0–1.0). */
    public static final double kFuelPickupIntensity        = 0.55;
    /** Duration of each on-pulse in the fuel-pickup double-pulse (seconds). */
    public static final double kFuelPickupPulseOnSeconds   = 0.12;
    /** Gap between the two fuel-pickup pulses (seconds). */
    public static final double kFuelPickupPauseBetweenSeconds = 0.08;

    // ── Shoot pulse (bumper spin-up + trigger feed phase) ─────────────────
    /** Rumble intensity during the repeating shoot pulse (0.0–1.0). */
    public static final double kShootPulseIntensity  = 0.85;
    /** Duration of the on-phase of each shoot pulse (seconds). */
    public static final double kShootPulseOnSeconds  = 0.15;
    /** Duration of the off-phase of each shoot pulse (seconds). */
    public static final double kShootPulseOffSeconds = 0.15;

    // ── Shoot announcement (right-trigger sequence start) ─────────────────
    /** Duration of the single announcement pulse when the aim-and-fire sequence begins (seconds). */
    public static final double kShootAnnouncementOnSeconds = 0.33;

    // ── Intake stow ───────────────────────────────────────────────────────
    /** Rumble intensity for the intake-stow confirmation pulse (0.0–1.0). */
    public static final double kIntakeStowIntensity    = 0.6;
    /** Duration of the intake-stow confirmation pulse (seconds). */
    public static final double kIntakeStowPulseSeconds = 0.5;

    // ── Climb success ─────────────────────────────────────────────────────
    /** Rumble intensity for the post-climb celebratory pulse (0.0–1.0). */
    public static final double kClimbSuccessIntensity      = 1.0;
    /** Duration of the on-phase of each climb-success pulse (seconds). */
    public static final double kClimbSuccessPulseOnSeconds = 0.4;
    /** Duration of the off-phase of each climb-success pulse (seconds). */
    public static final double kClimbSuccessPulseOffSeconds = 0.1;
    /** Maximum duration of the climb-success rumble sequence (seconds). */
    public static final double kClimbSuccessDurationSeconds = 15.0;
  }

  // ── Drivetrain IMU / Bump Detection ───────────────────────────────────────
  public static class BumpDetectionConstants {

    // ── Filtering ────────────────────────────────────────────────────────
    /**
     * Low-pass filter alpha for Pigeon 2 accelerometer axes.
     * Range [0, 1]: lower = heavier filtering (more bump rejection), higher = faster response.
     */
    public static final double kAccelFilterAlpha = 0.15;

    // ── Detection thresholds ─────────────────────────────────────────────
    /**
     * Absolute robot pitch (degrees) above which a bump is considered in progress.
     * Compared against |pitch|, so both nose-up (front-first) and nose-down
     * (back-first) traversals are detected equally.
     */
    public static final double kBumpPitchThresholdDegrees = 8.0;

    /**
     * Deviation from 1 g on the Pigeon 2 Z-axis (g) that indicates a bump.
     * Normal flat-ground reading is ~1 g; bumps cause it to spike significantly.
     */
    public static final double kBumpAccelZDeviationThreshold = 0.25;

    /**
     * Deviation from 1 g on the Limelight Z-axis accelerometer (g) used to
     * confirm the bump independently. Both sensors must agree before vision
     * measurements are rejected.
     */
    public static final double kLimelightAccelZDeviationThreshold = 0.25;

    /**
     * Encoder-vs-IMU linear acceleration mismatch magnitude (m/s²) above
     * which wheel slip is flagged. During bump traversal a slipping wheel
     * causes the encoder-derived acceleration to spike while the robot body
     * (measured by the IMU) barely moves — this difference is the slip signal.
     * Set conservatively high to avoid false positives from differentiation noise.
     */
    public static final double kWheelSlipDetectionThresholdMps2 = 6.0;

    // ── Vision trust adjustment ───────────────────────────────────────────
    /**
     * Standard-deviation multiplier applied to vision measurements while a
     * confirmed bump is in progress AND vision confidence is LOW.
     * Effectively tells the pose estimator to discount vision when the camera
     * is tilted and we have no reliable tag fix.
     */
    public static final double kBumpVisionStdDevMultiplier = 10.0;

    /**
     * Minimum vision confidence score (avgTagArea × tagCount) required to
     * bias the pose estimator toward Limelight during detected wheel slip.
     * A value of ~1.0 corresponds roughly to one AprilTag visible at moderate
     * range (~10 ft). Below this threshold the low-confidence path is used.
     */
    public static final double kHighConfidenceThreshold = 1.0;

    /**
     * Standard-deviation multiplier applied to vision measurements when
     * wheel slip is detected AND vision confidence is HIGH.
     * Values below 1.0 increase trust in vision; 0.5 makes the pose estimator
     * weight the Limelight fix twice as heavily as normal, actively correcting
     * the odometry error introduced by the slipping wheels.
     */
    public static final double kSlipHighConfStdDevMultiplier = 0.5;
  }
}
