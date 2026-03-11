package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.GameData;
import frc.robot.Landmarks;
import frc.robot.Ports;

/**
 * UltraShooter — physics-based shooter subsystem.
 *
 * <p>Built on the same three-motor SparkFlex hardware as {@link ShooterOrca},
 * but derives the required flywheel surface speed from a projectile-motion
 * formula rather than an empirical distance-to-speed look-up table.
 *
 * <p>Key constants live in {@link Constants.UltraShooterConstants}; update the
 * geometry values (hood height, shooter offset, launch angle) after physically
 * measuring the robot.
 *
 * <h2>Physics model</h2>
 * With a fixed launch angle θ, horizontal distance d (shooter exit → hub
 * center), and height h (hub center above the hood exit):
 * <pre>
 *   v₀ = d × √( g / (2 · cos²θ · (d·tanθ − h)) )
 * </pre>
 * where g = 9.81 m/s².  The result is converted from m/s to ft/s to match the
 * existing flywheel velocity convention.
 */
public class UltraShooter extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────

    private static final double WHEEL_DIAMETER_FEET       = 4.0 / 12.0;
    private static final double WHEEL_CIRCUMFERENCE_FEET  = Math.PI * WHEEL_DIAMETER_FEET;
    /** Converts motor RPM → wheel surface velocity (ft/s). */
    private static final double VELOCITY_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE_FEET / 60.0;
    /** Neo Vortex free-speed surface velocity (ft/s) — used for FF. */
    private static final double NEO_VORTEX_FREE_SPEED_FPS  = 6784.0 * VELOCITY_CONVERSION_FACTOR;

    /** kV in V/(ft/s), derived from physics-based FF: 12 V / free-speed. */
    private static final double KV = 12.0 / NEO_VORTEX_FREE_SPEED_FPS;

    private final SparkFlex primaryMotor   = new SparkFlex(Ports.kShooterLeft,   MotorType.kBrushless);
    private final SparkFlex secondaryMotor = new SparkFlex(Ports.kShooterMiddle, MotorType.kBrushless);
    private final SparkFlex tertiaryMotor  = new SparkFlex(Ports.kShooterRight,  MotorType.kBrushless);

    private final RelativeEncoder primaryEncoder   = primaryMotor.getEncoder();
    private final RelativeEncoder secondaryEncoder = secondaryMotor.getEncoder();
    private final RelativeEncoder tertiaryEncoder  = tertiaryMotor.getEncoder();

    private final SparkClosedLoopController primaryPID   = primaryMotor.getClosedLoopController();
    private final SparkClosedLoopController secondaryPID = secondaryMotor.getClosedLoopController();
    private final SparkClosedLoopController tertiaryPID  = tertiaryMotor.getClosedLoopController();

    // ── State ─────────────────────────────────────────────────────────────────

    /** Final velocity target set by commands (ft/s). */
    private double velocityTarget = 0;
    /** Ramped setpoint currently fed to the PID controllers (ft/s). */
    private double rampedSetpoint = 0;
    /** Latches true once the flywheel first reaches target; cleared on target change. */
    private boolean readyLatch = false;

    /** Circular buffer for rolling-average velocity filtering. */
    private final double[] velocityBuffer =
            new double[Constants.UltraShooterConstants.kVelocityAvgSamples];
    private int    velocityBufferIndex = 0;
    private double velocityBufferSum   = 0.0;

    /** Circular buffer for 1-second (50-sample) rolling-average distance filtering. */
    private static final int DISTANCE_AVG_SAMPLES = 50; // 50 Hz × 1 s
    private final double[] distanceBuffer = new double[DISTANCE_AVG_SAMPLES];
    private int    distanceBufferIndex = 0;
    private double distanceBufferSum   = 0.0;

    // ── Swerve reference (for distance-to-hub queries) ────────────────────────

    private final Swerve swerve;

    // ── ShooterTuner (live-adjustable parameters) ─────────────────────────────

    private final ShooterTuner shooterTuner;

    // ── NetworkTable telemetry ────────────────────────────────────────────────

    private final NetworkTable        nt           = NetworkTableInstance.getDefault().getTable("UltraShooter");
    private final NetworkTableEntry   ntVelocity   = nt.getEntry("Velocity ft/s");
    private final NetworkTableEntry   ntTarget     = nt.getEntry("Target ft/s");
    private final NetworkTableEntry   ntRamped     = nt.getEntry("Ramped Setpoint ft/s");
    private final NetworkTableEntry   ntAvg        = nt.getEntry("Avg Velocity ft/s");
    private final NetworkTableEntry   ntReady      = nt.getEntry("Ready");
    private final NetworkTableEntry   ntPhysics    = nt.getEntry("Physics Velocity ft/s");
    private final NetworkTableEntry   ntDistance    = nt.getEntry("Distance to Hub (m)");
    private final NetworkTableEntry   ntDistanceAvg = nt.getEntry("Avg Distance to Hub (m)");
    private final NetworkTableEntry   ntAngle      = nt.getEntry("Launch Angle (deg)");
    private final NetworkTableEntry   ntHoodHeight = nt.getEntry("Hood Height From Floor (in)");
    private final NetworkTableEntry   ntHubHeight  = nt.getEntry("Hub Center Height From Floor (in)");
    private final NetworkTableEntry   ntPrimCur    = nt.getEntry("Primary Current");
    private final NetworkTableEntry   ntSecCur     = nt.getEntry("Secondary Current");

    // ── Pi co-processor integration ───────────────────────────────────────────
    // physics_coprocessor.py (on WPILibPi) publishes raw physics velocity and a
    // heartbeat counter every 20 ms.  We prefer the Pi result when the heartbeat
    // has changed within the last 25 cycles (500 ms); otherwise we fall back to
    // the local calculateRequiredVelocityFPS() call transparently.

    private final NetworkTableEntry   ntPiPhysics  = nt.getEntry("Pi Physics Velocity ft/s");
    private final NetworkTableEntry   ntPiHB       = nt.getEntry("Pi Heartbeat");
    private final NetworkTableEntry   ntPiActive   = nt.getEntry("Pi Active");

    /** Last heartbeat counter seen from the Pi. */
    private long piLastHeartbeat = Long.MIN_VALUE;
    /** Cycles since the heartbeat last changed (each cycle = 20 ms). */
    private int  piStaleFrames   = 0;
    /** A Pi is considered disconnected after this many stale cycles (500 ms). */
    private static final int PI_STALE_THRESHOLD = 25;

    /** kP value most recently written to the SparkFlex controllers. */
    private double appliedKp = Constants.UltraShooterConstants.kP;

    // ─────────────────────────────────────────────────────────────────────────
    // Construction
    // ─────────────────────────────────────────────────────────────────────────

    public UltraShooter(Swerve swerve, ShooterTuner shooterTuner) {
        this.swerve        = swerve;
        this.shooterTuner  = shooterTuner;

        configureMotor(primaryMotor,   /* inverted */ false);
        configureMotor(secondaryMotor, /* inverted */ false);
        configureMotor(tertiaryMotor,  /* inverted */ true);

        SmartDashboard.putData(this);
    }

    private void configureMotor(SparkFlex motor, boolean inverted) {
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.inverted(inverted)
           .idleMode(IdleMode.kCoast)
           .smartCurrentLimit(
               Constants.UltraShooterConstants.kSmartCurrentLimit,
               Constants.UltraShooterConstants.kFreeCurrentLimit)
           .secondaryCurrentLimit(Constants.UltraShooterConstants.kStatorCurrentLimit);
        cfg.encoder
           .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
        cfg.closedLoop
           .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
           .pid(Constants.UltraShooterConstants.kP,
                Constants.UltraShooterConstants.kI,
                Constants.UltraShooterConstants.kD)
           .velocityFF(KV);
        motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Projectile-motion calculator
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Calculates the required flywheel surface velocity (ft/s) to arc a ball into
     * the hub from the given robot-center-to-hub distance.
     *
     * <p>Physics derivation — range equation solved for initial speed:
     * <pre>
     *   Horizontal:  x(t) = v₀·cosθ·t
     *   Vertical:    y(t) = v₀·sinθ·t − ½·g·t²
     *
     *   At impact: x = d,  y = h
     *   → v₀ = d · √( g / (2·cos²θ·(d·tanθ − h)) )
     * </pre>
     *
     * @param distanceToHubMeters  Odometry distance from <em>robot center</em> to hub
     *                             center (meters).
     * @return Required flywheel surface velocity in ft/s, or 0 if the geometry
     *         makes the shot physically impossible.
     */
    public static double calculateRequiredVelocityFPS(double distanceToHubMeters) {
        final double angleRad = Math.toRadians(Constants.UltraShooterConstants.kLaunchAngleDegrees);

        // Convert inch-based geometry constants to meters for SI physics.
        final double shooterOffsetM = Units.inchesToMeters(
                Constants.UltraShooterConstants.kShooterCenterlineOffsetInches);
        final double hoodHeightM = Units.inchesToMeters(
                Constants.UltraShooterConstants.kHoodHeightFromFloorInches);
        final double hubHeightM  = Units.inchesToMeters(
                Constants.UltraShooterConstants.kHubCenterHeightFromFloorInches);

        // True horizontal distance: subtract shooter-exit offset from odometry distance.
        final double d = distanceToHubMeters - shooterOffsetM;

        // Net vertical rise the ball must travel (positive = hub is above hood exit).
        final double h = hubHeightM - hoodHeightM;

        if (d <= 0) return 0; // degenerate — shooter is behind/at the hub

        final double cosTheta = Math.cos(angleRad);
        final double tanTheta = Math.tan(angleRad);

        // The denominator is 2·cos²θ·(d·tanθ − h).
        // If d·tanθ ≤ h the ball cannot reach the hub at this angle — return 0.
        final double denom = 2.0 * cosTheta * cosTheta * (d * tanTheta - h);
        if (denom <= 0) return 0;

        final double v0_mps = d * Math.sqrt(9.81 / denom);

        // Convert m/s → ft/s (1 m = 3.28084 ft)
        return v0_mps * 3.28084;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Fine-tune offset interpolation
    // ─────────────────────────────────────────────────────────────────────────

    private static final double NEAR_DISTANCE_M = Units.inchesToMeters(20.0);
    private static final double FAR_DISTANCE_M  = Units.inchesToMeters(120.0);

    /**
     * Linearly interpolates the fine-tune speed offset (%) between the near (20 in)
     * and far (120 in) reference distances, then clamps to the defined range.
     *
     * @param distanceMeters Distance from shooter to hub (meters).
     * @return Offset as a fraction (e.g. 5.0 % → 0.05).
     */
    private double interpolateOffsetFraction(double distanceMeters) {
        final double nearPct = shooterTuner.getNearShotOffsetPercent();
        final double farPct  = shooterTuner.getFarShotOffsetPercent();
        final double t = Math.max(0.0, Math.min(1.0,
                (distanceMeters - NEAR_DISTANCE_M) / (FAR_DISTANCE_M - NEAR_DISTANCE_M)));
        return (nearPct + t * (farPct - nearPct)) / 100.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Target control
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Sets the flywheel velocity target (ft/s).
     *
     * <p>The ready latch is only cleared when transitioning between stopped (0) and
     * running (non-zero). Mid-shot physics recalculations that nudge the target
     * value will NOT clear the latch, so a flywheel that has already been confirmed
     * ready stays ready for the duration of the shot.
     */
    public void setTarget(double targetFPS) {
        boolean wasRunning = velocityTarget != 0;
        boolean willRun    = targetFPS      != 0;
        if (wasRunning != willRun) readyLatch = false;
        velocityTarget = targetFPS;
    }

    /**
     * Updates the flywheel target using the projectile-motion calculator from the
     * 1-second averaged distance, with the fine-tune offset blended in.
     *
     * <p>When the Raspberry Pi co-processor ({@code physics_coprocessor.py}) is
     * live, its pre-computed physics velocity is used instead of the local
     * calculation.  If the Pi heartbeat goes stale for more than 500 ms the
     * fallback is automatic and transparent.
     */
    public void setPhysicsTarget() {
        final double distance       = getAverageDistanceToHub();
        final double offsetFraction = interpolateOffsetFraction(distance);

        final double physicsSpeed;
        if (isPiResultFresh()) {
            physicsSpeed = ntPiPhysics.getDouble(0.0);
        } else {
            physicsSpeed = calculateRequiredVelocityFPS(distance);
        }

        setTarget(physicsSpeed * (1.0 + offsetFraction));
    }

    /**
     * Returns true when the Pi co-processor heartbeat has updated within the
     * last {@value #PI_STALE_THRESHOLD} cycles (~500 ms).
     */
    private boolean isPiResultFresh() {
        return piStaleFrames < PI_STALE_THRESHOLD;
    }

    /** Called every periodic cycle to track whether the Pi heartbeat is advancing. */
    private void updatePiStaleness() {
        long hb = ntPiHB.getInteger(Long.MIN_VALUE);
        if (hb != piLastHeartbeat) {
            piLastHeartbeat = hb;
            piStaleFrames   = 0;
        } else {
            piStaleFrames++;
        }
    }

    public void stop() {
        setTarget(0);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Getters
    // ─────────────────────────────────────────────────────────────────────────

    /** Instantaneous primary encoder velocity (ft/s). */
    public double getVelocity() {
        return primaryEncoder.getVelocity();
    }

    /** 160 ms rolling average of primary encoder velocity (ft/s). */
    public double getAverageVelocity() {
        return velocityBufferSum / Constants.UltraShooterConstants.kVelocityAvgSamples;
    }

    public double getTarget()        { return velocityTarget; }
    public double getRampedSetpoint(){ return rampedSetpoint; }

    /**
     * True once the flywheel has reached target speed (latches through the RPM
     * dip caused by ball contact). Resets when {@link #setTarget} is called with
     * a new value.
     */
    public boolean isReady() {
        if (velocityTarget <= 0) {
            readyLatch = false;
            return false;
        }
        if (!readyLatch) {
            readyLatch = rampedSetpoint >= velocityTarget
                    && Math.abs(getAverageVelocity() - velocityTarget)
                            < shooterTuner.getReadyTolerance();
        }
        return readyLatch;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Commands
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Continuously spins up the flywheel using the physics calculator, updating
     * every cycle as the robot moves. Stops on interrupt.
     */
    public Command spinUpPhysicsCommand() {
        return runEnd(this::setPhysicsTarget, this::stop).withName("SpinUpPhysics");
    }

    /**
     * Pre-spins the flywheel to a fraction of the physics-calculated speed when:
     * <ol>
     *   <li>Fuel has been detected ({@code fuelReady}).</li>
     *   <li>The hub is active (within the 5-second expanded window).</li>
     *   <li>The robot is in its own alliance zone or the neutral zone.</li>
     * </ol>
     * Runs as the default command and is automatically interrupted by any shoot command.
     */
    public Command preSpinCommand(BooleanSupplier fuelReady) {
        return run(() -> {
            if (fuelReady.getAsBoolean()
                    && GameData.isHubActiveExpanded(5.0)
                    && Landmarks.isInScoringZone(swerve.getPose())) {
                double physicsSpeed = calculateRequiredVelocityFPS(swerve.getDistanceToHub());
                setTarget(physicsSpeed * Constants.UltraShooterConstants.kPreSpinFraction);
            } else {
                setTarget(0);
            }
        }).withName("PreSpin");
    }

    /** Holds the current target for the given duration, then stops. */
    public Command holdSpeedCommand(double seconds) {
        return run(() -> {}).withTimeout(seconds).andThen(runOnce(this::stop));
    }

    /** Continuously spins down to zero (for endgame). */
    public Command spinDownCommand() {
        return run(() -> setTarget(0)).withName("SpinDown");
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal periodic helpers
    // ─────────────────────────────────────────────────────────────────────────

    private void updateDistanceBuffer() {
        double newest = swerve.getDistanceToHub();
        distanceBufferSum -= distanceBuffer[distanceBufferIndex];
        distanceBuffer[distanceBufferIndex] = newest;
        distanceBufferSum += newest;
        distanceBufferIndex = (distanceBufferIndex + 1) % DISTANCE_AVG_SAMPLES;
    }

    /** @return 1-second rolling average of distance to hub (meters). */
    public double getAverageDistanceToHub() {
        return distanceBufferSum / DISTANCE_AVG_SAMPLES;
    }

    private void updateVelocityBuffer() {
        double newest = primaryEncoder.getVelocity();
        velocityBufferSum -= velocityBuffer[velocityBufferIndex];
        velocityBuffer[velocityBufferIndex] = newest;
        velocityBufferSum += newest;
        velocityBufferIndex =
                (velocityBufferIndex + 1) % Constants.UltraShooterConstants.kVelocityAvgSamples;
    }

    private void rampSetpoint() {
        if (rampedSetpoint < velocityTarget) {
            rampedSetpoint = Math.min(
                    rampedSetpoint + Constants.UltraShooterConstants.kRampUpRate, velocityTarget);
        } else if (rampedSetpoint > velocityTarget) {
            rampedSetpoint = Math.max(
                    rampedSetpoint - Constants.UltraShooterConstants.kRampDownRate, velocityTarget);
        }
    }

    /**
     * Re-configures all three SparkFlex controllers with a new kP value.
     * Uses {@code kNoResetSafeParameters} so only the closed-loop block is
     * touched, and {@code kNoPersistParameters} to avoid burning motor flash
     * (the Pi JSON is the persistence layer instead).
     */
    private void applyKpToMotors(double kP) {
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.closedLoop
           .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
           .pid(kP, Constants.UltraShooterConstants.kI, Constants.UltraShooterConstants.kD)
           .velocityFF(KV);
        primaryMotor  .configure(cfg, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters,
                                       com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);
        secondaryMotor.configure(cfg, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters,
                                       com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);
        tertiaryMotor .configure(cfg, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters,
                                       com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);
        appliedKp = kP;
    }

    private void applyPID() {
        double ff = Constants.UltraShooterConstants.kS + rampedSetpoint * KV;
        if (Math.abs(rampedSetpoint) > 3.0) {
            primaryPID  .setSetpoint(rampedSetpoint, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff);
            secondaryPID.setSetpoint(rampedSetpoint, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff);
            tertiaryPID .setSetpoint(rampedSetpoint, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff);
        } else {
            primaryMotor  .set(0);
            secondaryMotor.set(0);
            tertiaryMotor .set(0);
        }
    }

    private void updateNetworkTable() {
        ntVelocity   .setDouble(getVelocity());
        ntTarget     .setDouble(velocityTarget);
        ntRamped     .setDouble(rampedSetpoint);
        ntAvg        .setDouble(getAverageVelocity());
        ntReady      .setBoolean(isReady());
        ntPhysics    .setDouble(calculateRequiredVelocityFPS(getAverageDistanceToHub()));
        ntDistance   .setDouble(swerve.getDistanceToHub());
        ntDistanceAvg.setDouble(getAverageDistanceToHub());
        ntAngle     .setDouble(Constants.UltraShooterConstants.kLaunchAngleDegrees);
        ntHoodHeight.setDouble(Constants.UltraShooterConstants.kHoodHeightFromFloorInches);
        ntHubHeight .setDouble(Constants.UltraShooterConstants.kHubCenterHeightFromFloorInches);
        ntPrimCur   .setDouble(primaryMotor.getOutputCurrent());
        ntSecCur    .setDouble(secondaryMotor.getOutputCurrent());
        ntPiActive  .setBoolean(isPiResultFresh());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // SubsystemBase overrides
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        updatePiStaleness();
        updateDistanceBuffer();
        updateVelocityBuffer();
        rampSetpoint();
        applyPID();
        updateNetworkTable();

        // Apply any autotuner-updated kP while the robot is disabled so the
        // motors are never reconfigured mid-match.
        if (DriverStation.isDisabled()) {
            double newKp = shooterTuner.getKp();
            if (newKp != appliedKp) {
                applyKpToMotors(newKp);
            }
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Left ft/s",      () -> primaryEncoder.getVelocity(),   null);
        builder.addDoubleProperty("Middle ft/s",    () -> secondaryEncoder.getVelocity(), null);
        builder.addDoubleProperty("Right ft/s",     () -> tertiaryEncoder.getVelocity(),  null);
        builder.addDoubleProperty("Left Current",   () -> primaryMotor.getOutputCurrent(),   null);
        builder.addDoubleProperty("Middle Current", () -> secondaryMotor.getOutputCurrent(), null);
        builder.addDoubleProperty("Right Current",  () -> tertiaryMotor.getOutputCurrent(),  null);
        builder.addDoubleProperty("Target ft/s",    () -> velocityTarget,       null);
        builder.addDoubleProperty("Ramped ft/s",    () -> rampedSetpoint,       null);
        builder.addDoubleProperty("Avg ft/s",       this::getAverageVelocity,   null);
        builder.addDoubleProperty("Physics ft/s",
                () -> calculateRequiredVelocityFPS(swerve.getDistanceToHub()), null);
        builder.addBooleanProperty("Ready",         this::isReady,              null);
        builder.addStringProperty("Command",
                () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none", null);
    }
}
