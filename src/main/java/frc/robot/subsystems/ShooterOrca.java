// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Ports;
import frc.robot.subsystems.Swerve;


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
        // Shooter speeds, RPM
        public static final double kVelocityLow = 500;
        public static final double kVelocityMedium = 1500;
        public static final double kVelocityHigh = 2500;
        public static final double kVelocityMax = 3500;

        public static final double kP = 0.003;
        public static final double kI = 0.;
        public static final double kD = 0.25;
        public static final double kG = 0.;
        public static final double kS = 0.15;
        public static final double kV = .0033;

        // Setpoint ramp rates (RPM per 20ms cycle)
        // At 200 RPM/cycle spin-up: 0 → 5000 RPM takes ~0.5 seconds
        // At 400 RPM/cycle spin-down: 5000 → 0 RPM takes ~0.25 seconds
        public static final double kRampUpRate = 200.0;
        public static final double kRampDownRate = 400.0;

        // Flywheel is "ready" when within this tolerance of target RPM
        public static final double kReadyToleranceRPM = 200;
    }

    public static final class ShooterConfigs {
        // tertiary - right is the only inverted motor
        public static final SparkFlexConfig primaryShooterConfig = new SparkFlexConfig();
        public static final SparkFlexConfig secondaryShooterConfig = new SparkFlexConfig();
        public static final SparkFlexConfig tertiaryShooterConfig = new SparkFlexConfig();

               static {
            primaryShooterConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60, 40);
            secondaryShooterConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60, 40);
            tertiaryShooterConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60, 40);
            primaryShooterConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
            secondaryShooterConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
            tertiaryShooterConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
            
        }
    }

    public static final class Shooter {
      public static final String kTable = "Shooter";
      public static final String kVelocityRPM = "Velocity RPM";
      public static final String kTargetRPM = "Target RPM";
      public static final String kRampedSetpoint = "Ramped Setpoint RPM";
      public static final String kPrimaryCurrent = "Primary Current";
      public static final String kSecondaryCurrent = "Secondary Current";
    }

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

  Swerve m_swerveSubsystem;

  double shooterVelocityTarget = 0;  // Where we want to be (set by commands)
  double shooterVelocity = 0;        // Current ramped setpoint (fed to PID each cycle)
  boolean toggleDirection = false;
  double hoodTarget; // Position of the Hood in Rotations
  boolean hoodMovingForward = true; // true is positive
  
  // 55 - 57
  final SparkFlex shooterPrimaryMotor = new SparkFlex(Ports.kShooterLeft, MotorType.kBrushless);
  final SparkFlex shooterSecondaryMotor = new SparkFlex(Ports.kShooterMiddle, MotorType.kBrushless);
  final SparkFlex shooterTertiaryMotor = new SparkFlex(Ports.kShooterRight, MotorType.kBrushless);

  final RelativeEncoder shooterEncoder = shooterPrimaryMotor.getEncoder();

  final SparkClosedLoopController shooterPrimaryPIDController = shooterPrimaryMotor.getClosedLoopController();
  final SparkClosedLoopController shooterSecondaryPIDController = shooterSecondaryMotor.getClosedLoopController();
  final SparkClosedLoopController shooterTertiaryPIDController = shooterTertiaryMotor.getClosedLoopController();

  final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  final NetworkTable shooterTable = networkTable.getTable(Shooter.kTable);

  // Cached NetworkTable entries — avoids hash lookups every cycle (50Hz)
  private final NetworkTableEntry velocityEntryShooter = shooterTable.getEntry(Shooter.kVelocityRPM);
  private final NetworkTableEntry targetEntryShooter = shooterTable.getEntry(Shooter.kTargetRPM);
  private final NetworkTableEntry rampedSetpointEntryShooter = shooterTable.getEntry(Shooter.kRampedSetpoint);
  private final NetworkTableEntry primaryCurrentEntryShooter = shooterTable.getEntry(Shooter.kPrimaryCurrent);
  private final NetworkTableEntry secondaryCurrentEntryShooter = shooterTable.getEntry(Shooter.kSecondaryCurrent);
  private final NetworkTableEntry readyEntryShooter = shooterTable.getEntry("Ready");



  private InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();


  /** Creates a new ShooterSubsystem. */
  public ShooterOrca(Swerve swerveSubsystem) {

    m_swerveSubsystem = swerveSubsystem;

    shooterPrimaryMotor.configure(ShooterConfigs.primaryShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterSecondaryMotor.configure(ShooterConfigs.secondaryShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    addMapValues();
  }

  private void addMapValues() {
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(  8)), Double.valueOf(1615));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters( 20)), Double.valueOf(1685));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters( 32)), Double.valueOf(1755));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters( 44)), Double.valueOf(1825));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters( 56)), Double.valueOf(1895));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters( 68)), Double.valueOf(1965));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters( 80)), Double.valueOf(2035));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters( 92)), Double.valueOf(2105));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(104)), Double.valueOf(2175));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(116)), Double.valueOf(2300));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(128)), Double.valueOf(2350));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(140)), Double.valueOf(2400));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(152)), Double.valueOf(2450));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(164)), Double.valueOf(2525));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(176)), Double.valueOf(2595));
  }

  public double calculateShooterFeedForward() {
    // FF pivot = Ksta + Kvel * TarVel + Kgrav * cos(angle) + Kaccel * RobAccel * sin(angle)
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

  public void setShooterTarget(double target) {
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

  /** @return true if the shooter ramp has finished and flywheel is within tolerance of target RPM */
  public boolean isShooterReady() {
    return shooterVelocityTarget > 0
        && shooterVelocity >= shooterVelocityTarget
        && Math.abs(getShooterVelocity() - shooterVelocityTarget) < ShooterConstants.kReadyToleranceRPM;
  }

  public void setShooterMap() {
    double distanceToHub = m_swerveSubsystem.getDistanceToHub();
    // set shooter based on distance
    shooterVelocityTarget = shooterSpeedMap.get(distanceToHub);
  }

  /** @return Velocity in RPM */
  public double getShooterVelocity() {
    return shooterEncoder.getVelocity();
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
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
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

    public Command spinUpCommand() {
        return startEnd(() -> setShooterTarget(5000), () -> stop());
        //return runOnce(() -> setRPM(rpm));
            //.andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command spinUpCommand(Speed speed) {
        return startEnd(() -> setShooterTarget(speed.rpm), () -> stop());
        //return runOnce(() -> setRPM(rpm));
            //.andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command dashboardSpinUpCommand() {
        return runOnce(() -> setShooterTarget(5000)); 
    }
}