// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

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

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
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
        SHOOT(5000),
        DASHBOARD(0);

        private final double rpm;

        private Speed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity angularVelocity() {
            return RPM.of(rpm);
        }
    }

    public static final class ShooterConstants {
        // Motor free speed (RPM) — used for physics-based FF calculation
        public static final double kNeoVortexFreeSpeed = 6784.0;

        // PID gains
        public static final double kP = 0.003;
        public static final double kI = 0.000;
        public static final double kD = 0.25;
        public static final double kS = 0.15;
        public static final double kV = .0033;

        // Setpoint ramp rates (RPM per 20ms cycle)
        // At 200 RPM/cycle spin-up: 0 → 5000 RPM takes ~0.5 seconds
        // At 400 RPM/cycle spin-down: 5000 → 0 RPM takes ~0.25 seconds
        public static final double kRampUpRate = 200.0;
        public static final double kRampDownRate = 400.0;

        // Rolling average window for encoder noise filtering (samples at 50Hz)
        public static final int kVelocityAvgSamples = 8; // 8 × 20ms = 160ms

        // Flywheel is "ready" when the 160ms average is within this tolerance of target RPM
        public static final double kReadyToleranceRPM = 100;

        // Motor current limits (amps)
        public static final int kSmartCurrentLimit = 60;
        public static final int kFreeCurrentLimit = 40;
        public static final int kStatorCurrentLimit = 120;
    }

    public static final class TelemetryKeys {
        public static final String kTable = "Shooter";
        public static final String kVelocityRPM = "Velocity RPM";
        public static final String kTargetRPM = "Target RPM";
        public static final String kRampedSetpoint = "Ramped Setpoint RPM";
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
    private final NetworkTableEntry velocityEntryShooter = shooterTable.getEntry(TelemetryKeys.kVelocityRPM);
    private final NetworkTableEntry targetEntryShooter = shooterTable.getEntry(TelemetryKeys.kTargetRPM);
    private final NetworkTableEntry rampedSetpointEntryShooter = shooterTable.getEntry(TelemetryKeys.kRampedSetpoint);
    private final NetworkTableEntry primaryCurrentEntryShooter = shooterTable.getEntry(TelemetryKeys.kPrimaryCurrent);
    private final NetworkTableEntry secondaryCurrentEntryShooter = shooterTable.getEntry(TelemetryKeys.kSecondaryCurrent);
    private final NetworkTableEntry readyEntryShooter = shooterTable.getEntry("Ready");
    private final NetworkTableEntry avgVelocityEntryShooter = shooterTable.getEntry("Avg Velocity RPM");
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
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD)
            .velocityFF(12.0 / ShooterConstants.kNeoVortexFreeSpeed);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void addMapValues() {
        shooterSpeedMap.put(1.0, 2700.0);
        shooterSpeedMap.put(2.0, 2865.0); // Untested
        shooterSpeedMap.put(3.0, 3050.0);
        shooterSpeedMap.put(4.5, 3250.0); // Untested
        shooterSpeedMap.put(6.5, 4100.0);
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

        if (Math.abs(shooterVelocity) > 200) {
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
                && Math.abs(getAverageVelocity() - shooterVelocityTarget) < ShooterConstants.kReadyToleranceRPM;
        }
        return shooterReadyLatch;
    }

    public void setShooterMap() {
        double distanceToHub = m_swerveSubsystem.getDistanceToHub();
        shooterVelocityTarget = shooterSpeedMap.get(distanceToHub) * 1.0;
    }

    /** @return Primary encoder velocity in RPM (instantaneous) */
    public double getShooterVelocity() {
        return primaryEncoder.getVelocity();
    }

    /** @return 160ms rolling average of primary encoder velocity in RPM */
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
        builder.addDoubleProperty("Left RPM",   () -> primaryEncoder.getVelocity(),   null);
        builder.addDoubleProperty("Middle RPM", () -> secondaryEncoder.getVelocity(), null);
        builder.addDoubleProperty("Right RPM",  () -> tertiaryEncoder.getVelocity(),  null);
        builder.addDoubleProperty("Left Current",   () -> shooterPrimaryMotor.getOutputCurrent(),   null);
        builder.addDoubleProperty("Middle Current", () -> shooterSecondaryMotor.getOutputCurrent(), null);
        builder.addDoubleProperty("Right Current",  () -> shooterTertiaryMotor.getOutputCurrent(),  null);
        builder.addDoubleProperty("Target RPM",   () -> shooterVelocityTarget, null);
        builder.addDoubleProperty("Ramped RPM",   () -> shooterVelocity,       null);
        builder.addDoubleProperty("Avg RPM",      () -> getAverageVelocity(),  null);
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
    public Command preSpinCommand() {
        return run(() -> {
            if (GameData.isHubActiveExpanded(5.0)
                    && Landmarks.isInScoringZone(m_swerveSubsystem.getPose())) {
                double distanceToHub = m_swerveSubsystem.getDistanceToHub();
                double mapRPM = shooterSpeedMap.get(distanceToHub);
                shooterVelocityTarget = mapRPM * Constants.ShooterConstants.kPreSpinFraction;
            } else {
                shooterVelocityTarget = 0;
            }
        }).withName("PreSpin");
    }

    public Command spinUpCommand() {
        return startEnd(() -> setShooterTarget(5000), () -> stop());
    }

    public Command spinUpCommand(Speed speed) {
        return startEnd(() -> setShooterTarget(speed.rpm), () -> stop());
    }

    public Command dashboardSpinUpCommand() {
        return runOnce(() -> setShooterTarget(5000));
    }

    /**
     * Spins up the flywheel using the distance-to-RPM map, updated every cycle
     * from the robot pose distance to the hub.
     */
    public Command spinUpMapCommand() {
        return runEnd(() -> setShooterMap(), () -> stop());
    }

    /**
     * Like {@link #spinUpMapCommand()}, but adds a fixed RPM offset on top of the map value.
     * Useful for auto routines that need slightly more speed than the standard map.
     */
    public Command spinUpMapCommand(double rpmOffset) {
        return runEnd(() -> {
            double distanceToHub = m_swerveSubsystem.getDistanceToHub();
            shooterVelocityTarget = shooterSpeedMap.get(distanceToHub) + rpmOffset;
        }, () -> stop());
    }

    /** Holds the current flywheel target speed for the given duration, then stops. */
    public Command holdSpeedCommand(double seconds) {
        return run(() -> {}).withTimeout(seconds).andThen(runOnce(() -> stop()));
    }
}
