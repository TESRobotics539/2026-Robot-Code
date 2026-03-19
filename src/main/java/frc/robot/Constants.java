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
    /**
     * Maximum drivetrain translation speed used as the setpoint generator velocity ceiling (m/s).
     * SDS Mk4i L1 (8.14:1) theoretical max is ~4.11 m/s (~13.5 ft/s); 13 ft/s is the tuned
     * match limit, leaving a small margin below theoretical for controllability headroom.
     */
    public static final double kMaxSpeed = Units.feetToMeters(13);

    /**
     * Maximum drive wheel linear acceleration used by the setpoint generator (m/s²).
     * Limits how aggressively modules accelerate between cycles to prevent wheel slip
     * and excessive current draw during rapid direction changes.
     */
    public static final double kMaxAccelerationMps2 = 12; // ~40 ft/s²

    /**
     * Maximum module azimuth angular velocity used by the setpoint generator (rad/s).
     * NEO 550 at 5676 RPM through 21.43:1 steering gear → ~27.7 rad/s theoretical;
     * 20 rad/s (~72% of theoretical) is a practical cap to protect the steering motor.
     */
    public static final double kMaxSteeringVelocityRadPerSec = 20.0;

    /**
     * Center-to-module distance (meters) used by the setpoint generator kinematics.
     * SDS Mk4i on our frame: 10.875 in (276.225 mm) from robot center to each module pivot.
     * Must match the module positions in the YAGSL swerve JSON configs.
     * DO NOT CHANGE without also updating swerve/ORCA2026/*.json.
     */
    public static final double kModuleOffsetMeters = 0.276225; // 10.875 in

    /**
     * Exponent for the joystick input power curve: {@code output = copyDirectionPow(input, n)}.
     * A value of 1.5 softens fine control near the joystick center while preserving full
     * speed at the edges. Applied to all three axes (forward, strafe, rotation).
     * Increase toward 2.0 for even softer center feel; decrease toward 1.0 for more linear response.
     */
    public static final double kDriveInputCurvePower = 1.5;
  }

  // ── Intake ────────────────────────────────────────────────────────────────
  public static class IntakeConstants {

    // ── General ──────────────────────────────────────────────────────────
    /** If true, the intake is locked stowed from the start of autonomous through the end of the match. */
    public static final boolean kStowIntakeForMatch = false;
    /** Maximum seconds between two trigger pulls to count as a double-tap. */
    public static final double kDoubleTapWindowSeconds = 0.4;
    /** Seconds the roller can run without a load spike before being automatically cut off. */
    public static final double kRollerNoLoadTimeoutSeconds = 12.0;

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
    public static final double kStowedPosition   = 0.350;
    public static final double kDeployedPosition = 0.097;
    public static final double kMinPosition       = 0.100;
    public static final double kMaxPosition       = 0.200;

    // ── Roller duty cycle ─────────────────────────────────────────────────
    /**
     * Roller duty cycle (0.0–1.0) applied during normal intaking and the post-jam
     * recovery phase.  The roller PID runs in RPM mode for steady-state; this
     * open-loop percent output is used by the anti-jam state machine so that the
     * jam recovery path is independent of the closed-loop controller.
     */
    public static final double kRollerForwardPercent = 0.9;
    /**
     * Roller duty cycle applied during the anti-jam reverse pulse (negative = backward).
     * Magnitude matched to {@link #kRollerForwardPercent} so the reverse pulse clears
     * the jam with the same authority as normal intaking.
     */
    public static final double kRollerReversePercent = -0.9;

    // ── Stow calibration ──────────────────────────────────────────────────
    /**
     * Maximum absolute encoder deviation from {@link #kStowedPosition} that is still
     * accepted as a valid stow calibration reading at boot.  Within ±0.05 encoder units
     * the robot is effectively stowed; outside this window the stow position is not
     * snapshotted and the nominal constant is used as the PID target instead.
     */
    public static final double kStowCalibrationTolerance = 0.05;

    /**
     * Minimum encoder gap between the current target position and {@link #kStowedPosition}
     * required for the pivot to be considered deployed.  The pivot target must be more than
     * 0.05 units below the stowed position; values above this threshold are treated as stowed
     * regardless of commanded direction.
     */
    public static final double kIsDeployedTolerance = 0.05;

    // ── Roller anti-jam ──────────────────────────────────────────────────
    /** Current (amps) above which a roller jam is suspected while intaking. */
    public static final double kRollerJamCurrentThreshold = 60.0;
    /** Velocity (RPM) below which a high-current reading is treated as a jam, not just spinup. */
    public static final double kRollerJamVelocityThresholdRPM = 1000.0;
    /** Reverse speed (RPM, magnitude) applied during the roller anti-jam pulse. */
    public static final double kRollerUnjamReverseRPM = 750.0;
    /** Duration of the reverse pulse during roller anti-jam (seconds). */
    public static final double kRollerUnjamReverseSeconds = 0.5;
    /** Velocity (RPM) above which the roller is considered to be running freely after an unjam. */
    public static final double kRollerFreeVelocityThresholdRPM = 3000.0;

    // ── Ramp rate ─────────────────────────────────────────────────────────
    /** Closed-loop ramp rate for the roller (seconds from 0 to full output). Applies to both acceleration and deceleration. */
    public static final double kRollerRampRateSeconds = 0.33;

    // ── Agitation (pivot motion during shooting to settle fuel) ──────────
    /** Fraction (0 = deployed hard-stop, 1 = stowed) for the upper agitation bound. */
    public static final double kAgitateHighPercent = 0.447;
    /** Fraction (0 = deployed hard-stop, 1 = stowed) for the lower agitation bound. */
    public static final double kAgitateLowPercent  = 0.209;
    /** Duration of one full oscillation cycle (seconds). */
    public static final double kAgitatePeriodSeconds = 1.0;
    /** Pivot closed-loop output range during agitation (symmetric ±). Controls how fast the pivot tracks the oscillating setpoint. */
    public static final double kAgitateOutputRangeMax = 0.35;
  }

  // ── Feeder ────────────────────────────────────────────────────────────────
  public static class FeederConstants {

    // ── Motor ────────────────────────────────────────────────────────────
    /** Primary current limit (amps). */
    public static final int kSmartCurrentLimit = 50;
    public static final IdleMode kIdleMode = IdleMode.kCoast;

    // ── Setpoints ────────────────────────────────────────────────────────
    /** Feed target speed (RPM, positive = forward). */
    public static final double kFeedRPM    =  4500.0;
    /** Reverse target speed (RPM, negative = backward). */
    public static final double kReverseRPM = -5500.0;

    // ── PID ──────────────────────────────────────────────────────────────
    public static final double kP          = 0.0001;
    public static final double kI          = 0.0;
    public static final double kD          = 0.0;
    public static final double kVelocityFF = 0.000175;

    // ── Anti-jam ──────────────────────────────────────────────────────────
    /** Current (amps) above which a jam is suspected while feeding. */
    public static final double kJamCurrentThreshold = 80.0;
    /** Velocity (RPM) below which a high-current reading is treated as a jam, not just spinup. */
    public static final double kJamVelocityThresholdRPM = 1000.0;
    /** Reverse speed (RPM, magnitude) applied during the anti-jam pulse. */
    public static final double kUnjamReverseRPM = 750.0;
    /** Duration of the reverse pulse during anti-jam (seconds). */
    public static final double kUnjamReverseSeconds = 0.5;
    /** Velocity (RPM) above which the feeder is considered to be running freely after an unjam. */
    public static final double kFreeVelocityThresholdRPM = 3000.0;

    // ── Ramp rate ─────────────────────────────────────────────────────────
    /** Closed-loop ramp rate for the feeder (seconds from 0 to full output). Applies to both acceleration and deceleration. */
    public static final double kRampRateSeconds = 0.25;
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
    /**
     * Maximum seconds the feeder + floor run in autonomous shoot sequences before cutting off.
     * Acts as a safety timeout so a jammed autonomous sequence doesn't block the rest of the auto.
     */
    public static final double kLongFeedTimeoutSeconds = 7.0;

    // ── Setpoints ────────────────────────────────────────────────────────
    /** Fraction of the distance-based map speed to hold during pre-spin (0.0–1.0). */
    public static final double kPreSpinFraction = 0.60;
    /** Flywheel speed for the close-range dump shot (ft/s, converted from 1850 RPM with 4" wheel). */
    public static final double kDumpShotFlywheelSpeed = 32.0;

    // ── Aim tolerance ─────────────────────────────────────────────────────
    /**
     * Heading error (degrees) below which the robot is considered "aimed" at the hub.
     * {@link frc.robot.commands.AimAndDriveCommand#isAimed()} returns true when the
     * absolute heading difference is within this window.  Decrease for tighter aim
     * gates (more accurate shots); increase if the robot oscillates around the target
     * before shots are allowed to release.
     */
    public static final double kAimToleranceDegrees = 3.0;

    // ── Drive-to-ideal-distance command ───────────────────────────────────
    /**
     * Target distance from hub center for {@code driveToIdealShootingDistanceCommand()} (ft).
     * Chosen as the midpoint of the 6–10 ft high-efficiency zone from physics simulation.
     * Adjust if empirical testing shows a different optimal distance.
     */
    public static final double kIdealShootDistanceFt = 8.0;
    /**
     * Stopping tolerance for {@code driveToIdealShootingDistanceCommand()} (ft).
     * The command ends when the robot is within ±½ ft (~6 in) of the target distance.
     */
    public static final double kIdealShootToleranceFt = 0.5;
    /**
     * Proportional gain for the drive-to-ideal-distance P controller (ft/s per ft of error).
     * Also reused as the proportional gain for the close-range backup command.
     * Increase to reach the target faster; decrease if the robot overshoots.
     */
    public static final double kIdealShootKpFps = 1.5;
    /**
     * Maximum speed cap for the drive-to-ideal-distance command (ft/s).
     * Prevents the P controller from commanding dangerous speeds when far from the target.
     */
    public static final double kIdealShootMaxSpeedFps = 10.0;

    // ── Sequencing ────────────────────────────────────────────────────────
    /**
     * Seconds the right-trigger aim-and-fire sequence holds flywheel speed and heading after
     * the driver releases the trigger.  Allows the last ball to clear the barrel before the
     * flywheel spins down.  After this window, the flywheel idles to pre-spin speed.
     */
    public static final double kHoldAimAndSpeedSeconds = 1.0;

    /**
     * Delay (seconds) before the intake automatically deploys at the start of teleop
     * when no auto climb occurred.  Gives the drivetrain and Limelight time to initialise
     * before the pivot moves.
     */
    public static final double kTeleopIntakeDeployDelaySeconds = 1.0;

    /**
     * Seconds before or after the hub-shift boundary where the hub is treated as active
     * for the right-trigger shoot gate.  This small expansion prevents the gate from closing
     * right as the driver pulls the trigger on a hub that just switched states.
     */
    public static final double kHubActiveExpansionSeconds = 5.0;
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
    /**
     * Maximum seconds to wait for the homing current spike before giving up.
     * If the hard stop is never reached (e.g., mechanical obstruction or disconnected motor)
     * the homing command times out here rather than running forever.
     */
    public static final double kHomingTimeoutSeconds = 10.0;

    // ── Declimb / teleop reversal ──────────────────────────────────────────
    /**
     * Encoder position (rotations, absolute value from zero) within which the declimb
     * reversal command considers itself finished.  After an autonomous climb the encoder
     * is reset to zero at the climb start; the reverse runs until the position falls back
     * inside this window, indicating the arm is near its starting height.
     */
    public static final double kDeclimbReturnThresholdRotations = 10.0;
    /**
     * Minimum distance (meters) the robot must drive away from its teleop-start position
     * after the declimb sequence before the intake is deployed.  Ensures the robot has
     * cleared the tower structure before the intake swings out.
     */
    public static final double kDeclimbDriveDistanceMeters = 2.0;
  }

  // ── UltraShooter Physics ──────────────────────────────────────────────────
  public static class UltraShooterConstants {
    // ── Geometry ───────────────────────────────────────────────────────────

    /**
     * Height of the hood exit (ball launch point) above the floor (inches).
     * Measure from the ground to the center of the shooter barrel exit.
     */
    public static final double kHoodHeightFromFloorInches = 26.5;

    /**
     * Height of the hub center (scoring target) above the floor (inches).
     * Look this up from the game manual or measure on the field.
     * DO NOT CHANGE!!!
     */
    public static final double kHubCenterHeightFromFloorInches = 72.0;

    /**
     * Horizontal distance from the robot center to the shooter (hood) exit
     * point (inches). The hood is at the BACK of the robot, so this value is
     * ADDED to the odometry distance (robot center → hub) to get the true
     * shooter-to-hub range used by the physics model.
     */
    public static final double kShooterCenterlineOffsetInches = 10.5;

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
    // Initial values ported from ORCA3136 2026 (ShooterSubsystem.java).
    // ORCA uses RPM units; ours are ft/s. Conversion: 1 ft/s = 60/(π×4/12) ≈ 57.3 RPM.
    //   kP: 0.0003/RPM × 57.3 ≈ 0.017/fps
    //   kD: 0.01/RPM  × 57.3 ≈ 0.57/fps
    //   kS: 0.21 V — same units, direct copy
    public static final double kP = 0.025;
    public static final double kI = 0.000;
    public static final double kD = 0.0;
    public static final double kS = 0.1;
    /** KV feedforward: V/(ft/s). Derived from Neo Vortex free speed (6784 RPM) and wheel size.
     *  At target speed, motor voltage ≈ kS + kV * velocityFPS. */
    public static final double kV = 11.0 / (6784.0 * Math.PI * (4.0 / 12.0) / 60.0);

    // ── Flywheel behavior ─────────────────────────────────────────────────
    /** Setpoint ramp-up rate (ft/s per 20 ms cycle). */
    public static final double kRampUpRate   = 200.0 * (Math.PI * (4.0 / 12.0) / 60.0);
    /** Setpoint ramp-down rate (ft/s per 20 ms cycle). */
    public static final double kRampDownRate = 400.0 * (Math.PI * (4.0 / 12.0) / 60.0);
    /** Rolling-average window for encoder noise filtering (samples at 50 Hz). 3 × 20 ms = 60 ms. */
    public static final int kVelocityAvgSamples = 3;
    /** Flywheel is "ready" within this tolerance of target (ft/s). */
    public static final double kReadyTolerance = 100.0 * (Math.PI * (4.0 / 12.0) / 60.0);
    /** Fraction of the physics-calculated speed used for pre-spin (0.0–1.0). */
    public static final double kPreSpinFraction = 0.60;

    // ── Voltage bias (971-style integral flywheel) ────────────────────────
    /** Volts added to feedforward per ft/s of velocity error, per 20 ms cycle.
     *  Accumulates only after the ramp has reached target — no spinup windup. */
    public static final double kVoltageBiasRate = 0.004;
    /** Maximum absolute feedforward voltage bias (V). Caps the integrator. */
    public static final double kMaxVoltageBias  = 2.0;

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
    public static final double kFlywheelEfficiency = 0.6;

    /**
     * Ball diameter (inches). Used to compute cross-sectional area for drag.
     * Change this when swapping game pieces; kDragCoefficient must be updated
     * accordingly: B = 0.5 x Cd x rho_air x pi x (diameter_m / 2)^2
     */
    public static final double kBallDiameterInches = 5.9;

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

    // ── Pre-spin gating ───────────────────────────────────────────────────

    /**
     * If true, the FMS-aware pre-spin state machine is active: the flywheel only
     * spins up to pre-spin speed when the hub is within
     * its active window, and the robot is in the scoring zone.
     *
     * <p>Set to {@code false} to always pre-spin to the pre-spin speed regardless
     * of FMS state, fuel detection, or field position — useful for practice matches,
     * scrimmages, or debugging when the full state machine is not desired.
     */
    public static final boolean kEnableFMSAwarePreSpinLatch = false;

    // ── Pi physics engine ─────────────────────────────────────────────────
    /**
     * Cycles without a heartbeat change before the Pi physics engine is declared
     * disconnected and the local Java physics fallback is used instead (20 ms/cycle → 500 ms).
     * Matches the staleness threshold used by {@link frc.robot.subsystems.vision.PiAprilTagVision}.
     */
    public static final int kPiStaleThreshold = 25;

    // ── Distance rolling average ──────────────────────────────────────────
    /**
     * Number of samples in the hub-distance rolling average buffer (1 second at 50 Hz).
     * When the robot is stationary, the average smooths odometry noise.  When moving,
     * the instantaneous distance is used to avoid the lag this window would introduce.
     * See {@link frc.robot.subsystems.robot.UltraShooter#getAverageDistanceToHub()}.
     */
    public static final int kDistanceAvgSamples = 50; // 50 Hz × 1 s

    /**
     * Robot speed (ft/s) above which the instantaneous hub distance is used instead of the
     * rolling average.  Below this threshold the robot is considered stationary and the
     * smoothed average is preferred to reject odometry noise from small drivetrain vibrations.
     */
    public static final double kMovingSpeedThresholdFps = 1.0;

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
    public static final double kCloseRangeBackupInches = 18.0;

    /** Speed (ft/s) at which the robot drives away from the hub during backup. */
    public static final double kCloseRangeBackupSpeedFps = 4.0;
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
    public static final double kShootPulseIntensity  = 0.70;
    /** Duration of the on-phase of each shoot pulse (seconds). */
    public static final double kShootPulseOnSeconds  = 0.10;
    /** Duration of the off-phase of each shoot pulse (seconds). */
    public static final double kShootPulseOffSeconds = 0.10;

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

  // ── Vision ────────────────────────────────────────────────────────────────
  // Adapted from ORCA3136/ORCABot2026 VisionSubsystem constants.
  public static class VisionConstants {

    // ── Rejection thresholds ──────────────────────────────────────────────

    /** Maximum yaw rate (deg/s) before rejecting MegaTag2 estimates.
     *  270 deg/s matches ORCA3136; more conservative than Limelight's 720 deg/s ceiling. */
    public static final double kMaxYawRateDegPerSec = 270.0;

    /** Maximum age (seconds) of a pose estimate timestamp before rejection.
     *  Matches ORCA3136 kMaxTimestampAgeSec. */
    public static final double kMaxTimestampAgeSec = 0.5;

    // ── Field boundary (2026 REBUILT: 54 ft 3.2 in x 26 ft 5.7 in) ──────
    // Field is 16.54 m x 8.21 m with 0.5 m margin to avoid rejecting near-wall poses.
    public static final double kFieldMinX = -0.5;
    public static final double kFieldMaxX = 17.04;
    public static final double kFieldMinY = -0.5;
    public static final double kFieldMaxY = 8.71;

    // ── Standard deviation tuning ─────────────────────────────────────────
    // Formula: xyStdDev = max(kMinXYStdDev, kXYStdDevBase * dist^2 * (1/tagCount) * singleTagPenalty)
    // Adapted from ORCA3136.

    /** Base coefficient for XY standard deviation calculation. */
    public static final double kXYStdDevBase = 0.5;

    /** Multiplier applied when only one tag is visible (higher uncertainty). */
    public static final double kSingleTagPenalty = 2.0;

    /** Minimum XY standard deviation floor (~4 in). */
    public static final double kMinXYStdDev = 0.1;

    // ── IMU lifecycle ─────────────────────────────────────────────────────

    /** Cycles to wait after an IMU mode switch before accepting measurements (~300 ms at 50 Hz).
     *  Matches ORCA3136/ORCABot2026 VisionSubsystem.kImuSettleCycles. */
    public static final int kImuSettleCycles = 15;

    /** If no measurement is accepted within this many seconds, vision is reported unhealthy. */
    public static final double kVisionHealthyTimeoutSec = 0.5;

    // ── Disabled-period odometry seeding ──────────────────────────────────

    /** Minimum tag count required for disabled-period odometry seeding. */
    public static final int kSeedMinTagCount = 2;

    /** Maximum average tag distance (meters, ~13 ft) for disabled-period seeding. */
    public static final double kSeedMaxDistM = 4.0;

    // ── Raspberry Pi AprilTag vision ──────────────────────────────────────

    /**
     * Cycles without a Pi heartbeat change before the Pi is declared disconnected (500 ms).
     * At 50 Hz, 25 stale cycles = 500 ms, matching the timeout used in UltraShooter for
     * the physics engine heartbeat.
     */
    public static final int kPiStaleThreshold = 25;

    /**
     * Approximate camera + processing pipeline latency subtracted from
     * {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} to produce the Pi measurement
     * timestamp.  Increase if the robot overshoots in autos (measurement is too fresh);
     * decrease if it undershoots (measurement is too stale).
     */
    public static final double kPiPipelineLatencySeconds = 0.10;

    /**
     * Minimum AprilTag decision margin accepted from the Pi.  Mirrors
     * {@code MIN_DECISION_MARGIN} in {@code apriltag_vision.py} — the Pi already filters
     * on this value; this is a secondary Java-side gate to handle any Python-side changes.
     * Higher values require more-confident tag detections but reject more frames.
     */
    public static final double kPiMinDecisionMargin = 35.0;

    /**
     * Pi single-tag pose XY standard deviation (meters).
     * Higher uncertainty than Limelight due to an uncalibrated consumer USB camera.
     */
    public static final double kPiSingleTagXyStddev = 0.7;
    /** Pi single-tag pose heading standard deviation (radians). High — single-tag heading is ambiguous. */
    public static final double kPiSingleTagThetaStddev = 25.0;
    /**
     * Pi multi-tag pose XY standard deviation (meters).
     * Improved over single-tag because multiple tags reduce pose ambiguity.
     */
    public static final double kPiMultiTagXyStddev = 0.3;
    /** Pi multi-tag pose heading standard deviation (radians). */
    public static final double kPiMultiTagThetaStddev = 15.0;
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
     * Deviation from 1 g on the Limelight Z-axis accelerometer (g) used as
     * one vote in the 2-of-3 bump majority (Pigeon + front LL + rear LL).
     *
     * <p><b>Tuning:</b> Log {@code Vision/BumpVotes} while driving over the bump
     * at match speed. Decrease this value if bump traversals only register 1 vote
     * (threshold too high); increase it if flat-ground vibration triggers false
     * votes (threshold too low). Start around 0.20–0.35 g and adjust in 0.05 g
     * increments. The Pigeon threshold ({@link #kBumpAccelZDeviationThreshold})
     * should be tuned the same way via {@code Swerve/PigeonOverBump} logging.
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
     * Standard-deviation multiplier applied when a confirmed bump is in progress
     * AND vision confidence is LOW. Heavy inflation tells the pose estimator to
     * nearly ignore vision — the camera is shaking and there is no reliable tag fix.
     */
    public static final double kBumpVisionStdDevMultiplier = 10.0;

    /**
     * Standard-deviation multiplier applied when a confirmed bump is in progress
     * AND vision confidence is HIGH (solid tag fix despite camera shake).
     * Mild inflation — still somewhat discounts the noisy reading, but lets a
     * close-range multi-tag fix contribute meaningfully to the estimate.
     * Tune by observing pose jump magnitude on Elastic while crossing the bump
     * near tags; increase if pose still jumps, decrease if drift goes uncorrected.
     */
    public static final double kBumpHighConfStdDevMultiplier = 3.0;

    /**
     * Minimum vision confidence score (avgTagArea × tagCount) required to
     * distinguish high-confidence from low-confidence paths.
     * A value of ~1.0 corresponds roughly to one AprilTag visible at moderate
     * range (~10 ft). Below this threshold the low-confidence (inflate) path is used.
     */
    public static final double kHighConfidenceThreshold = 1.0;

    /**
     * Standard-deviation multiplier applied when wheel slip is detected AND
     * vision confidence is HIGH. A value of 1.0 passes through stddevs unchanged —
     * trust vision at face value rather than artificially boosting it. The wheels
     * are unreliable, but that is not a reason to over-weight a single vision frame.
     */
    public static final double kSlipHighConfStdDevMultiplier = 1.0;
  }
}
