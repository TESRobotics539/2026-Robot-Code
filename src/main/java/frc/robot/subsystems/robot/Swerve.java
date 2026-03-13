// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.robot;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Landmarks;
import frc.robot.subsystems.tuning.BumpTuner;
import frc.util.LowPassFilter;
import frc.util.MetricTracker;
import frc.util.SwerveSetpoint;
import frc.util.SwerveSetpointGenerator;
import frc.util.SwerveSetpointGenerator.KinematicLimits;

import java.io.File;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase
{
  private final SwerveDrive swerveDrive;

  private final Field2d field;
  private final BumpTuner bumpTuner;

  // Pigeon 2 accelerometer filtering — rejects transient spikes from bump traversal.
  // Retrieved from YAGSL after swerveDrive construction so we share the same Phoenix 6
  // handle that YAGSL owns, rather than creating a second device object with a hardcoded ID.
  private Pigeon2 pigeon2;
  // Cached signal objects — refreshed together via refreshAll() each cycle to guarantee
  // all five values come from the same CAN frame before the filters and rolling averages run.
  private StatusSignal<?> pigeonAccelX;
  private StatusSignal<?> pigeonAccelY;
  private StatusSignal<?> pigeonAccelZ;
  private StatusSignal<?> pigeonPitch;
  private StatusSignal<?> pigeonRoll;
  private final LowPassFilter accelXFilter = new LowPassFilter(Constants.BumpDetectionConstants.kAccelFilterAlpha);
  private final LowPassFilter accelYFilter = new LowPassFilter(Constants.BumpDetectionConstants.kAccelFilterAlpha);
  private final LowPassFilter accelZFilter = new LowPassFilter(Constants.BumpDetectionConstants.kAccelFilterAlpha);

  // Wheel-slip detection — compare encoder-derived acceleration to IMU acceleration.
  // Mismatch indicates wheels are spinning faster than the robot body is moving.
  private double prevVxMetersPerSecond = 0.0;
  private double prevVyMetersPerSecond = 0.0;
  private double wheelSlipScore = 0.0; // rolling-averaged magnitude in m/s², cached for external callers

  // 60 ms (3-cycle) rolling average for the raw slip magnitude.
  // Differentiation amplifies sample-to-sample encoder noise; averaging over 3 cycles
  // smooths that noise without meaningfully delaying detection of true wheel slip events.
  private static final int    SLIP_AVG_SAMPLES   = 3;
  private static final double LOOP_PERIOD_SECONDS = 0.02;
  private static final double GRAVITY_MPS2        = 9.81;
  private final double[] slipBuffer = new double[SLIP_AVG_SAMPLES];
  private int    slipBufferIndex = 0;
  private double slipBufferSum   = 0.0;

  // Setpoint generator — enforces kinematic limits (max accel, max steering velocity)
  // on robot-relative ChassisSpeeds before passing to YAGSL. Adapted from frc5687/2023-robot
  // (originally Team 254). Prevents wheel scrub and motor torque violations on rapid inputs.
  private static final double MODULE_OFFSET_M = 0.276225; // 10.875 in from center
  private static final KinematicLimits KINEMATIC_LIMITS = new KinematicLimits(
      Constants.DrivetrainConstants.kMaxSpeed,
      Constants.DrivetrainConstants.kMaxAccelerationMps2,
      Constants.DrivetrainConstants.kMaxSteeringVelocityRadPerSec);
  private SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint          currentSetpoint;

  // MetricTracker column indices — registered once in constructor, used every periodic().
  private int metricWheelSlip;
  private int metricOverBump;
  private int metricWheelSlipping;


  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory  Directory of swerve drive config files.
   * @param field      Field2d object for visualizing the robot's position on the field.
   * @param bumpTuner  Live-tunable bump detection parameters (from the Pi).
   */
  public Swerve(File directory, Field2d field, BumpTuner bumpTuner)
  {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    this.field = field;
    this.bumpTuner = bumpTuner;

    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.DrivetrainConstants.kMaxSpeed);
    } 
    catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    pigeon2      = (Pigeon2) swerveDrive.getGyro().getIMU();
    pigeonAccelX = pigeon2.getAccelerationX();
    pigeonAccelY = pigeon2.getAccelerationY();
    pigeonAccelZ = pigeon2.getAccelerationZ();
    pigeonPitch  = pigeon2.getPitch();
    pigeonRoll   = pigeon2.getRoll();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0,
        pigeonAccelX, pigeonAccelY, pigeonAccelZ, pigeonPitch, pigeonRoll);

    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); // Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
    // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible

    setupAutoBuilder();
    initSetpointGenerator();
    initMetricColumns();
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   * @param field         Field2d object for visualizing the robot's position on the field.
   * @param bumpTuner     Live-tunable bump detection parameters (from the Pi).
   */
  public Swerve(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg, Field2d field, BumpTuner bumpTuner)
  {
    this.field = field;
    this.bumpTuner = bumpTuner;

    swerveDrive = new SwerveDrive(driveCfg,
                                  controllerCfg,
                                  Constants.DrivetrainConstants.kMaxSpeed,
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                             Rotation2d.fromDegrees(0)));
    pigeon2      = (Pigeon2) swerveDrive.getGyro().getIMU();
    pigeonAccelX = pigeon2.getAccelerationX();
    pigeonAccelY = pigeon2.getAccelerationY();
    pigeonAccelZ = pigeon2.getAccelerationZ();
    pigeonPitch  = pigeon2.getPitch();
    pigeonRoll   = pigeon2.getRoll();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0,
        pigeonAccelX, pigeonAccelY, pigeonAccelZ, pigeonPitch, pigeonRoll);

    setupAutoBuilder();
    initSetpointGenerator();
    initMetricColumns();
  }

  private void initSetpointGenerator() {
    Translation2d[] modulePositions = {
        new Translation2d( MODULE_OFFSET_M,  MODULE_OFFSET_M),
        new Translation2d( MODULE_OFFSET_M, -MODULE_OFFSET_M),
        new Translation2d(-MODULE_OFFSET_M,  MODULE_OFFSET_M),
        new Translation2d(-MODULE_OFFSET_M, -MODULE_OFFSET_M),
    };
    setpointGenerator = new SwerveSetpointGenerator(swerveDrive.kinematics, modulePositions);
    SwerveModuleState[] initStates = {
        new SwerveModuleState(), new SwerveModuleState(),
        new SwerveModuleState(), new SwerveModuleState()
    };
    currentSetpoint = new SwerveSetpoint(new ChassisSpeeds(), initStates);
  }

  private void initMetricColumns() {
    metricWheelSlip     = MetricTracker.getInstance().addColumn("WheelSlipScore (m/s^2)");
    metricOverBump      = MetricTracker.getInstance().addColumn("IsOverBump");
    metricWheelSlipping = MetricTracker.getInstance().addColumn("IsWheelSlipping");
  }

  private void setupAutoBuilder() {
    // RobotConfig values sourced from:
    //   mass/MOI/module positions — pathplanner/settings.json
    //   wheel radius/gear ratio/COF/current limit — swerve/modules/physicalproperties.json
    RobotConfig config = new RobotConfig(
        74.088, // robot mass (kg)
        6.883,  // moment of inertia (kg·m²)
        new ModuleConfig(
            0.0508,                                   // wheel radius: 4" wheel → 2" = 0.0508 m
            5.45,                                     // max drive speed (m/s)
            1.19,                                     // wheel COF
            DCMotor.getNeoVortex(1).withReduction(6.12), // SparkFlex + NEO Vortex, L? gearing
            40.0,                                     // drive current limit (A)
            1                                         // motors per module
        ),
        new Translation2d( 0.273,  0.273), // FL
        new Translation2d( 0.273, -0.273), // FR
        new Translation2d(-0.273,  0.273), // BL
        new Translation2d(-0.273, -0.273)  // BR
    );
    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getRobotVelocity,
        (speeds, feedforwards) -> drive(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // translation
            new PIDConstants(5.0, 0.0, 0.0)  // rotation
        ),
        config,
        () -> DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false),
        this
    );
  }

  @Override
  public void periodic()
  {
    field.setRobotPose(swerveDrive.getPose());

    double flPos = swerveDrive.getModules()[0].getAbsoluteEncoder().getAbsolutePosition();
    double frPos = swerveDrive.getModules()[1].getAbsoluteEncoder().getAbsolutePosition();
    double blPos = swerveDrive.getModules()[2].getAbsoluteEncoder().getAbsolutePosition();
    double brPos = swerveDrive.getModules()[3].getAbsoluteEncoder().getAbsolutePosition();

    Logger.recordOutput("Swerve/FL/AbsAngleDeg", flPos);
    Logger.recordOutput("Swerve/FL/RelAngleDeg", swerveDrive.getModules()[0].getState().angle.getDegrees());
    Logger.recordOutput("Swerve/FL/DriftDeg",    flPos - swerveDrive.getModules()[0].getState().angle.getDegrees());

    Logger.recordOutput("Swerve/FR/AbsAngleDeg", frPos);
    Logger.recordOutput("Swerve/FR/RelAngleDeg", swerveDrive.getModules()[1].getState().angle.getDegrees());
    Logger.recordOutput("Swerve/FR/DriftDeg",    frPos - swerveDrive.getModules()[1].getState().angle.getDegrees());

    Logger.recordOutput("Swerve/BL/AbsAngleDeg", blPos);
    Logger.recordOutput("Swerve/BL/RelAngleDeg", swerveDrive.getModules()[2].getState().angle.getDegrees());
    Logger.recordOutput("Swerve/BL/DriftDeg",    blPos - swerveDrive.getModules()[2].getState().angle.getDegrees());

    Logger.recordOutput("Swerve/BR/AbsAngleDeg", brPos);
    Logger.recordOutput("Swerve/BR/RelAngleDeg", swerveDrive.getModules()[3].getState().angle.getDegrees());
    Logger.recordOutput("Swerve/BR/DriftDeg",    brPos - swerveDrive.getModules()[3].getState().angle.getDegrees());

    // Filtered Pigeon 2 accelerometer — bump spikes are smoothed out.
    // refreshAll() guarantees all five signals come from the same CAN frame before
    // the low-pass filters and wheel-slip rolling averages consume them.
    BaseStatusSignal.refreshAll(pigeonAccelX, pigeonAccelY, pigeonAccelZ, pigeonPitch, pigeonRoll);
    double rawX = pigeonAccelX.getValueAsDouble();
    double rawY = pigeonAccelY.getValueAsDouble();
    double rawZ = pigeonAccelZ.getValueAsDouble();
    double filtX = accelXFilter.calculate(rawX);
    double filtY = accelYFilter.calculate(rawY);
    double filtZ = accelZFilter.calculate(rawZ);

    Logger.recordOutput("Swerve/Accel/XRaw_g",   rawX);
    Logger.recordOutput("Swerve/Accel/YRaw_g",   rawY);
    Logger.recordOutput("Swerve/Accel/ZRaw_g",   rawZ);
    Logger.recordOutput("Swerve/Accel/XFilt_g",  filtX);
    Logger.recordOutput("Swerve/Accel/YFilt_g",  filtY);
    Logger.recordOutput("Swerve/Accel/ZFilt_g",  filtZ);

    // Wheel-slip detection — encoder-derived acceleration vs IMU acceleration.
    //
    // When wheels slip over the bump, they spin faster than the robot body moves.
    // The encoder velocity (and therefore its derivative) spikes while the IMU
    // measures the actual body motion. The mismatch is the slip signal.
    //
    // The Pigeon 2 accelerometer reports body-frame accelerations that include
    // a gravity component proportional to pitch/roll. Subtract that projection
    // so only true linear body acceleration remains before comparing.
    ChassisSpeeds currentVel = swerveDrive.getRobotVelocity();
    double encoderAccelX = (currentVel.vxMetersPerSecond - prevVxMetersPerSecond) / LOOP_PERIOD_SECONDS; // m/s²
    double encoderAccelY = (currentVel.vyMetersPerSecond - prevVyMetersPerSecond) / LOOP_PERIOD_SECONDS;
    prevVxMetersPerSecond = currentVel.vxMetersPerSecond;
    prevVyMetersPerSecond = currentVel.vyMetersPerSecond;

    double pitchRad = Math.toRadians(pigeonPitch.getValueAsDouble());
    double rollRad  = Math.toRadians(pigeonRoll.getValueAsDouble());
    double imuAccelX = (filtX - Math.sin(pitchRad)) * GRAVITY_MPS2; // gravity-compensated, m/s²
    double imuAccelY = (filtY - Math.sin(rollRad))  * GRAVITY_MPS2;

    double rawSlip = Math.hypot(encoderAccelX - imuAccelX, encoderAccelY - imuAccelY);
    slipBufferSum -= slipBuffer[slipBufferIndex];
    slipBuffer[slipBufferIndex] = rawSlip;
    slipBufferSum += rawSlip;
    slipBufferIndex = (slipBufferIndex + 1) % SLIP_AVG_SAMPLES;
    wheelSlipScore = slipBufferSum / SLIP_AVG_SAMPLES;

    Logger.recordOutput("Swerve/WheelSlip/RawMps2", rawSlip);
    Logger.recordOutput("Swerve/WheelSlip/AvgMps2", wheelSlipScore);
    Logger.recordOutput("Swerve/WheelSlip/Slipping", isWheelSlipping());

    MetricTracker.getInstance().newRow();
    MetricTracker.getInstance().set(metricWheelSlip,     wheelSlipScore);
    MetricTracker.getInstance().set(metricOverBump,      isOverBump() ? 1.0 : 0.0);
    MetricTracker.getInstance().set(metricWheelSlipping, isWheelSlipping() ? 1.0 : 0.0);
  }

  @Override
  public void simulationPeriodic()
  {
    // YAGSL's SwerveDrive.periodic() already calls updateOdometry() each cycle.
    // Calling it again here would double-integrate position in simulation.
    Pose2d pose = swerveDrive.getPose();
    field.setRobotPose(pose);
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that tells the robot to drive forward until the command ends.
   *
   * @return a Command that tells the robot to drive forward until the command ends
   */
  public Command driveForward()
  {
    return run(() -> {
      swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
    }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
  {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true, 
                        false);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * <p>Runs the requested speeds through the {@link SwerveSetpointGenerator} before commanding
   * YAGSL, enforcing per-module acceleration and steering-rate limits to reduce wheel scrub
   * and protect motor controllers during rapid direction changes. (Adapted from frc5687/2023-robot.)
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    currentSetpoint = setpointGenerator.generateSetpoint(KINEMATIC_LIMITS, currentSetpoint, velocity, 0.02);
    swerveDrive.drive(currentSetpoint.chassisSpeeds);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drivebase zero command
   */
  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        Constants.DrivetrainConstants.kMaxSpeed);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.DrivetrainConstants.kMaxSpeed);
  }

  /**
   * Enables or disables YAGSL heading correction.
   * Should only be active when the robot is being controlled via a target angle
   * (e.g. during {@link frc.robot.commands.AimAndDriveCommand}).
   */
  public void setHeadingCorrection(boolean enabled)
  {
    swerveDrive.setHeadingCorrection(enabled);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} for the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /** Returns Pigeon 2 pitch in degrees. Positive = nose up. */
  public double getPitchDegrees()
  {
    return pigeonPitch.getValueAsDouble();
  }

  /** Returns Pigeon 2 roll in degrees. Positive = left side up. */
  public double getRollDegrees()
  {
    return pigeonRoll.getValueAsDouble();
  }

  /**
   * Returns the robot yaw rate in degrees per second from the Pigeon 2.
   * Positive = counter-clockwise when viewed from above.
   */
  public double getYawRateDegPerSec()
  {
    return pigeon2.getAngularVelocityZWorld().getValueAsDouble();
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

  /**
   * Add a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   * @param measurementStdDevs Standard deviations of the vision pose measurement (x, y, and rotation).
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> measurementStdDevs) {
      swerveDrive.addVisionMeasurement(visionPose, timestamp, measurementStdDevs);
  }

  /**
   * Add a vision measurement to the pose estimator with default standard deviations.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
      swerveDrive.addVisionMeasurement(visionPose, timestamp);
  }

  /**
   * Returns the latest low-pass-filtered Pigeon 2 Z-axis acceleration (g).
   * Used by external callers (e.g. RobotContainer) for bump consensus with the Limelight IMU.
   */
  public double getFilteredAccelZ() {
    return accelZFilter.get();
  }

  /**
   * Returns true when the Pigeon 2 alone indicates the robot is traversing the bump.
   * Checks both pitch angle and filtered Z-acceleration deviation from 1 g.
   * For higher-confidence bump detection, also check the Limelight accelerometer externally.
   */
  public boolean isOverBump() {
    double pitchDeg   = Math.abs(swerveDrive.getPitch().getDegrees());
    double zDeviation = Math.abs(accelZFilter.get() - 1.0);
    return pitchDeg   > bumpTuner.getBumpPitchThresholdDeg()
        || zDeviation > bumpTuner.getBumpAccelZDeviation();
  }

  /**
   * Returns the latest wheel-slip score (m/s²), computed as the magnitude of the
   * mismatch between encoder-derived acceleration and gravity-compensated IMU
   * acceleration. Higher values indicate more slip.
   */
  public double getWheelSlipScore() {
    return wheelSlipScore;
  }

  /**
   * Returns true when wheel slip is detected.
   *
   * <p>Two independent signals must agree before flagging slip, reducing false positives
   * from differentiation noise:
   * <ul>
   *   <li>The encoder-vs-IMU acceleration mismatch exceeds {@code kWheelSlipDetectionThresholdMps2}</li>
   *   <li>The robot is simultaneously traversing the bump (pitch or Z-accel threshold)</li>
   * </ul>
   * Requiring both means flat-ground hard-braking events won't falsely trigger slip mode.
   */
  public boolean isWheelSlipping() {
    return isOverBump()
        && wheelSlipScore > bumpTuner.getWheelSlipThresholdMps2();
  }

  public double getDistanceToHub() {
    Translation2d hubTranslation = Landmarks.hubPosition();
    Translation2d robotTranslation = swerveDrive.getPose().getTranslation();
    return robotTranslation.getDistance(hubTranslation);
  }
}
