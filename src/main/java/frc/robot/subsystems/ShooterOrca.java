// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.GameData;
import frc.robot.Landmarks;
import frc.robot.Ports;


/*
 *
 * This subsystem is for all Shooter operations
 * Including:
 *    logging shooter data
 *    Managing the speeds of the flywheels
 *
 */


public class ShooterOrca extends SubsystemBase {
    public enum Speed {
        SHOOT(87),   // ~5000 RPM with 4" wheel
        DASHBOARD(0);

        public final double metersPerSecond;

        private Speed(double metersPerSecond) {
            this.metersPerSecond = metersPerSecond;
        }
    }

    public static final class ShooterConstants {
        // Flywheel wheel size
        public static final double kWheelDiameterFeet = 4.0 / 12.0; // 4 inches
        public static final double kWheelCircumferenceFeet = Math.PI * kWheelDiameterFeet;
        // Converts motor RPM → wheel surface velocity (ft/s)
        public static final double kVelocityConversionFactor = kWheelCircumferenceFeet / 60.0;

        // Motor free speed in ft/s — used for physics-based FF calculation
        public static final double kNeoVortexFreeSpeedFPS = 6784.0 * kVelocityConversionFactor; // ~118.39 ft/s

        // PID gains
        public static final double kP = 0.003;
        public static final double kI = 0.000;
        public static final double kD = 0.25;
        public static final double kS = 0.15;
        public static final double kV = 0.0033 / kVelocityConversionFactor; // V/(m/s)

        // Setpoint ramp rates (ft/s per 20ms cycle)
        // At spin-up rate: 0 → ~87.3 ft/s takes ~0.5 seconds
        // At spin-down rate: ~87.3 → 0 ft/s takes ~0.25 seconds
        public static final double kRampUpRate = 200.0 * kVelocityConversionFactor;
        public static final double kRampDownRate = 400.0 * kVelocityConversionFactor;

        // Rolling average window for encoder noise filtering (samples at 50Hz)
        public static final int kVelocityAvgSamples = 8; // 8 × 20ms = 160ms

        // Flywheel is "ready" when the 160ms average is within this tolerance of target (ft/s)
        public static final double kReadyTolerance = 100 * kVelocityConversionFactor; // ~2 ft/s

        // Motor current limits (amps)
        public static final int kSmartCurrentLimit = 60;
        public static final int kFreeCurrentLimit = 40;
        public static final int kStatorCurrentLimit = 120;
    }

    public static final class TelemetryKeys {
        public static final String kTable = "Shooter";
        public static final String kVelocity = "Velocity ft/s";
        public static final String kTarget = "Target ft/s";
        public static final String kRampedSetpoint = "Ramped Setpoint ft/s";
        public static final String kPrimaryCurrent = "Primary Current";
        public static final String kSecondaryCurrent = "Secondary Current";
    }

    private Swerve m_swerveSubsystem;

    private double shooterVelocityTarget = 0;  // Where we want to be (set by commands)
    private double shooterVelocity = 0;        // Current ramped setpoint (fed to PID each cycle)
    private boolean shooterReadyLatch = false; // Stays true once ready; cleared on new target

    // Circular buffer for 160ms rolling average of primary encoder velocity
    private final double[] velocityBuffer = new double[ShooterConstants.kVelocityAvgSamples];
    private int velocityBufferIndex = 0;
    private double velocityBufferSum = 0.0;

    // CAN IDs 55 - 57
    private final SparkFlex shooterPrimaryMotor = new SparkFlex(Ports.kShooterLeft, MotorType.kBrushless);
    private final SparkFlex shooterSecondaryMotor = new SparkFlex(Ports.kShooterMiddle, MotorType.kBrushless);
    private final SparkFlex shooterTertiaryMotor = new SparkFlex(Ports.kShooterRight, MotorType.kBrushless);

    private final RelativeEncoder primaryEncoder = shooterPrimaryMotor.getEncoder();
    private final RelativeEncoder secondaryEncoder = shooterSecondaryMotor.getEncoder();
    private final RelativeEncoder tertiaryEncoder = shooterTertiaryMotor.getEncoder();

    private final SparkClosedLoopController shooterPrimaryPIDController = shooterPrimaryMotor.getClosedLoopController();
    private final SparkClosedLoopController shooterSecondaryPIDController = shooterSecondaryMotor.getClosedLoopController();
    private final SparkClosedLoopController shooterTertiaryPIDController = shooterTertiaryMotor.getClosedLoopController();

    private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
    private final NetworkTable shooterTable = networkTable.getTable(TelemetryKeys.kTable);

    // Cached NetworkTable entries — avoids hash lookups every cycle (50Hz)
    private final NetworkTableEntry velocityEntryShooter = shooterTable.getEntry(TelemetryKeys.kVelocity);
    private final NetworkTableEntry targetEntryShooter = shooterTable.getEntry(TelemetryKeys.kTarget);
    private final NetworkTableEntry rampedSetpointEntryShooter = shooterTable.getEntry(TelemetryKeys.kRampedSetpoint);
    private final NetworkTableEntry primaryCurrentEntryShooter = shooterTable.getEntry(TelemetryKeys.kPrimaryCurrent);
    private final NetworkTableEntry secondaryCurrentEntryShooter = shooterTable.getEntry(TelemetryKeys.kSecondaryCurrent);
    private final NetworkTableEntry readyEntryShooter = shooterTable.getEntry("Ready");
    private final NetworkTableEntry avgVelocityEntryShooter = shooterTable.getEntry("Avg Velocity ft/s");
    private final NetworkTableEntry distanceToHubEntry = shooterTable.getEntry("Distance to Hub (m)");

    private final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();


    /** Creates a new ShooterSubsystem. */
    public ShooterOrca(Swerve swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;

        configureMotor(shooterPrimaryMotor, false);
        configureMotor(shooterSecondaryMotor, false);
        configureMotor(shooterTertiaryMotor, true);

        addMapValues();

        SmartDashboard.putData(this);
    }

    private void configureMotor(SparkFlex motor, boolean inverted) {
        SparkFlexConfig config = new SparkFlexConfig();
        config.inverted(inverted)
              .idleMode(IdleMode.kCoast)
              .smartCurrentLimit(ShooterConstants.kSmartCurrentLimit, ShooterConstants.kFreeCurrentLimit)
              .secondaryCurrentLimit(ShooterConstants.kStatorCurrentLimit);
        config.encoder
            .velocityConversionFactor(ShooterConstants.kVelocityConversionFactor);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD)
            .velocityFF(12.0 / ShooterConstants.kNeoVortexFreeSpeedFPS);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void addMapValues() {
        // Keys: distance in meters; values: wheel surface velocity in ft/s (converted from RPM with 4" wheel)
        shooterSpeedMap.put(1.0, 47.0); // 2700 RPM
        shooterSpeedMap.put(2.0, 50.0); // 2865 RPM — Untested
        shooterSpeedMap.put(3.0, 53.0); // 3050 RPM
        shooterSpeedMap.put(4.5, 57.0); // 3250 RPM — Untested
        shooterSpeedMap.put(6.5, 72.0); // 4100 RPM
    }

    public double calculateShooterFeedForward() {
        // FF = Ksta + Kvel * TarVel
        double ff = ShooterConstants.kS + shooterVelocity * ShooterConstants.kV;
        return ff;
    }

    /**
     * Sets the shooters velocity
     *
     * use "setShooterVelocityTarget" to change shooterVelocity variable in this subsystem
     */
    private void setShooterPIDVelocity() {
        // Calculate FF once and reuse — it uses the ramped setpoint (not the final target)
        // so the FF matches what the PID is currently tracking.
        double ff = calculateShooterFeedForward();

        if (Math.abs(shooterVelocity) > 3.0) { // ~200 RPM with 4" wheel
            shooterPrimaryPIDController.setSetpoint(shooterVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff);
            shooterSecondaryPIDController.setSetpoint(shooterVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff);
            shooterTertiaryPIDController.setSetpoint(shooterVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff);
        } else {
            shooterPrimaryMotor.set(0);
            shooterSecondaryMotor.set(0);
            shooterTertiaryMotor.set(0);
        }
    }

    public void setPercentOutput(double percentOutput) {
        double voltage = percentOutput * 12.0;
        shooterPrimaryMotor.setVoltage(voltage);
        shooterSecondaryMotor.setVoltage(voltage);
        shooterTertiaryMotor.setVoltage(voltage);
    }

    public void setShooterTarget(double target) {
        if (target != shooterVelocityTarget) {
            shooterReadyLatch = false;
        }
        shooterVelocityTarget = target;
    }

    /** @return The final target velocity (before ramping) */
    public double getShooterTarget() {
        return shooterVelocityTarget;
    }

    /** @return The current ramped setpoint being fed to PID */
    public double getRampedSetpoint() {
        return shooterVelocity;
    }

    /** @return Primary motor for simulation access */
    public SparkFlex getShooterPrimaryMotor() {
        return shooterPrimaryMotor;
    }

    /**
     * Returns true once the flywheel has reached target speed and latches — the ready
     * state is held through the RPM dip caused by fuel contacting the flywheel.
     * The latch resets when {@link #setShooterTarget} is called with a new value.
     */
    public boolean isShooterReady() {
        if (shooterVelocityTarget <= 0) {
            shooterReadyLatch = false;
            return false;
        }
        if (!shooterReadyLatch) {
            shooterReadyLatch = shooterVelocity >= shooterVelocityTarget
                && Math.abs(getAverageVelocity() - shooterVelocityTarget) < ShooterConstants.kReadyTolerance;
        }
        return shooterReadyLatch;
    }

    public void setShooterMap() {
        double distanceToHub = m_swerveSubsystem.getDistanceToHub();
        shooterVelocityTarget = shooterSpeedMap.get(distanceToHub) * 1.0;
    }

    /** @return Primary encoder velocity in ft/s (instantaneous) */
    public double getShooterVelocity() {
        return primaryEncoder.getVelocity();
    }

    /** @return 160ms rolling average of primary encoder velocity in ft/s */
    public double getAverageVelocity() {
        return velocityBufferSum / ShooterConstants.kVelocityAvgSamples;
    }

    /** Update the circular buffer with the latest encoder reading. Called once per periodic(). */
    private void updateVelocityBuffer() {
        double newest = primaryEncoder.getVelocity();
        velocityBufferSum -= velocityBuffer[velocityBufferIndex];
        velocityBuffer[velocityBufferIndex] = newest;
        velocityBufferSum += newest;
        velocityBufferIndex = (velocityBufferIndex + 1) % ShooterConstants.kVelocityAvgSamples;
    }

    /** @return Current in Amps */
    public double getShooterPrimaryCurrent() {
        return shooterPrimaryMotor.getOutputCurrent();
    }

    /** @return Current in Amps */
    public double getShooterSecondaryCurrent() {
        return shooterSecondaryMotor.getOutputCurrent();
    }

    /** Ramp the setpoint toward the target each cycle. */
    private void rampSetpoint() {
        if (shooterVelocity < shooterVelocityTarget) {
            shooterVelocity = Math.min(shooterVelocity + ShooterConstants.kRampUpRate, shooterVelocityTarget);
        } else if (shooterVelocity > shooterVelocityTarget) {
            shooterVelocity = Math.max(shooterVelocity - ShooterConstants.kRampDownRate, shooterVelocityTarget);
        }
    }

    /** Publish continuous values to network table */
    public void updateNetworkTable() {
        velocityEntryShooter.setDouble(getShooterVelocity());
        targetEntryShooter.setDouble(getShooterTarget());
        rampedSetpointEntryShooter.setDouble(getRampedSetpoint());
        primaryCurrentEntryShooter.setDouble(getShooterPrimaryCurrent());
        secondaryCurrentEntryShooter.setDouble(getShooterSecondaryCurrent());
        readyEntryShooter.setBoolean(isShooterReady());
        avgVelocityEntryShooter.setDouble(getAverageVelocity());
        distanceToHubEntry.setDouble(m_swerveSubsystem.getDistanceToHub());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Left ft/s",   () -> primaryEncoder.getVelocity(),   null);
        builder.addDoubleProperty("Middle ft/s", () -> secondaryEncoder.getVelocity(), null);
        builder.addDoubleProperty("Right ft/s",  () -> tertiaryEncoder.getVelocity(),  null);
        builder.addDoubleProperty("Left Current",   () -> shooterPrimaryMotor.getOutputCurrent(),   null);
        builder.addDoubleProperty("Middle Current", () -> shooterSecondaryMotor.getOutputCurrent(), null);
        builder.addDoubleProperty("Right Current",  () -> shooterTertiaryMotor.getOutputCurrent(),  null);
        builder.addDoubleProperty("Target ft/s",   () -> shooterVelocityTarget, null);
        builder.addDoubleProperty("Ramped ft/s",   () -> shooterVelocity,       null);
        builder.addDoubleProperty("Avg ft/s",      () -> getAverageVelocity(),  null);
        builder.addBooleanProperty("Ready",       () -> isShooterReady(),      null);
        builder.addStringProperty("Command",
            () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none", null);
    }

    /** This method will be called once per scheduler run */
    @Override
    public void periodic() {
        updateVelocityBuffer();
        rampSetpoint();
        updateNetworkTable();
        setShooterPIDVelocity();
    }

    /** This method will be called once per scheduler run during simulation */
    @Override
    public void simulationPeriodic() {}

    public void stop() {
        setShooterTarget(0);
    }

    /**
     * Pre-spins the flywheel to {@link ShooterConstants#kPreSpinFraction} of the
     * distance-based map RPM when both conditions are met:
     * <ol>
     *   <li>The hub is active (same 5-second expanded window used for shooting).</li>
     *   <li>The robot is in its own alliance zone or the neutral zone — not in the
     *       opponent's half of the field.</li>
     * </ol>
     * Drops to 0 whenever either condition is false.
     *
     * <p>Intended to run as the shooter's default command so it is automatically
     * interrupted by any real shoot command and resumes afterward.
     */
    public Command preSpinCommand(BooleanSupplier fuelReady) {
        return run(() -> {
            if (DriverStation.isAutonomous()) {
                // Auton: spin at full map speed immediately, no conditions
                setShooterMap();
            } else if (fuelReady.getAsBoolean()
                    && GameData.isHubActiveExpanded(5.0)
                    && Landmarks.isInScoringZone(m_swerveSubsystem.getPose())) {
                double distanceToHub = m_swerveSubsystem.getDistanceToHub();
                double mapSpeed = shooterSpeedMap.get(distanceToHub);
                shooterVelocityTarget = mapSpeed * Constants.ShooterConstants.kPreSpinFraction;
            } else {
                shooterVelocityTarget = 0;
            }
        }).withName("PreSpin");
    }

    /**
     * Continuously holds the flywheel target at 0, spinning it down and suppressing
     * pre-spin for as long as this command runs. Intended for endgame when climbing.
     */
    public Command spinDownCommand() {
        return run(() -> setShooterTarget(0)).withName("SpinDown");
    }

    public Command spinUpCommand() {
        return startEnd(() -> setShooterTarget(Speed.SHOOT.metersPerSecond), () -> stop());
    }

    public Command spinUpCommand(Speed speed) {
        return startEnd(() -> setShooterTarget(speed.metersPerSecond), () -> stop());
    }

    public Command dashboardSpinUpCommand() {
        return runOnce(() -> setShooterTarget(Speed.SHOOT.metersPerSecond));
    }

    /**
     * Spins up the flywheel using the distance-to-RPM map, updated every cycle
     * from the robot pose distance to the hub.
     */
    public Command spinUpMapCommand() {
        return runEnd(() -> setShooterMap(), () -> stop());
    }

    /**
     * Like {@link #spinUpMapCommand()}, but adds a fixed speed offset (ft/s) on top of the map value.
     * Useful for auto routines that need slightly more speed than the standard map.
     */
    public Command spinUpMapCommand(double speedOffset) {
        return runEnd(() -> {
            double distanceToHub = m_swerveSubsystem.getDistanceToHub();
            shooterVelocityTarget = shooterSpeedMap.get(distanceToHub) + speedOffset;
        }, () -> stop());
    }

    /** Holds the current flywheel target speed for the given duration, then stops. */
    public Command holdSpeedCommand(double seconds) {
        return run(() -> {}).withTimeout(seconds).andThen(runOnce(() -> stop()));
    }
}
