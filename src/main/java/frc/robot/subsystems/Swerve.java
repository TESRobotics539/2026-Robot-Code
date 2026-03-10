// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.util.LowPassFilter;

import java.io.File;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.encoders.CANCoderSwerve;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase
{
  private final SwerveDrive swerveDrive;
  private final NetworkTable telemetryTable;

  private final SwerveAbsoluteEncoder absoluteEncoder_fl;
  private final SwerveAbsoluteEncoder absoluteEncoder_fr;
  private final SwerveAbsoluteEncoder absoluteEncoder_bl;
  private final SwerveAbsoluteEncoder absoluteEncoder_br;

  private final Field2d field;

  // Pigeon 2 accelerometer filtering — rejects transient spikes from bump traversal.
  private final Pigeon2 pigeon2 = new Pigeon2(62, "rio");
  private final LowPassFilter accelXFilter = new LowPassFilter(Constants.BumpDetectionConstants.kAccelFilterAlpha);
  private final LowPassFilter accelYFilter = new LowPassFilter(Constants.BumpDetectionConstants.kAccelFilterAlpha);
  private final LowPassFilter accelZFilter = new LowPassFilter(Constants.BumpDetectionConstants.kAccelFilterAlpha);

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   * @param field Field2d object for visualizing the robot's position on the field.
   */
  public Swerve(File directory, Field2d field)
  { 
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    this.telemetryTable = NetworkTableInstance.getDefault().getTable("AbsoluteEncoders");
    this.field = field;

    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.DrivetrainConstants.kMaxSpeed);
    } 
    catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); // Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false, 
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
    // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible

    absoluteEncoder_fl = new CANCoderSwerve(9);
    absoluteEncoder_fr = new CANCoderSwerve(12);
    absoluteEncoder_bl = new CANCoderSwerve(6);
    absoluteEncoder_br = new CANCoderSwerve(3);

    setupAutoBuilder();
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   * @param field Field2d object for visualizing the robot's position on the field.
   */
  public Swerve(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg, Field2d field)
  {
    this.telemetryTable = NetworkTableInstance.getDefault().getTable("AbsoluteEncoders");
    this.field = field;

    swerveDrive = new SwerveDrive(driveCfg,
                                  controllerCfg,
                                  Constants.DrivetrainConstants.kMaxSpeed,
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                             Rotation2d.fromDegrees(0)));

    absoluteEncoder_fl = new CANCoderSwerve(9);
    absoluteEncoder_fr = new CANCoderSwerve(12);
    absoluteEncoder_bl = new CANCoderSwerve(6);
    absoluteEncoder_br = new CANCoderSwerve(3);

    setupAutoBuilder();
  }

  private void setupAutoBuilder() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
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
    } catch (Exception e) {
      throw new RuntimeException("Failed to load PathPlanner RobotConfig", e);
    }
  }

  @Override
  public void periodic()
  {
    field.setRobotPose(swerveDrive.getPose());

    telemetryTable.getEntry("Module FL Absolute Angle").setNumber(absoluteEncoder_fl.getAbsolutePosition());
    telemetryTable.getEntry("Module FL Relative Angle").setNumber(swerveDrive.getModules()[0].getState().angle.getDegrees());
    telemetryTable.getEntry("Module FL Drift").setNumber(absoluteEncoder_fl.getAbsolutePosition() - swerveDrive.getModules()[0].getState().angle.getDegrees());

    telemetryTable.getEntry("Module FR Absolute Angle").setNumber(absoluteEncoder_fr.getAbsolutePosition());
    telemetryTable.getEntry("Module FR Relative Angle").setNumber(swerveDrive.getModules()[1].getState().angle.getDegrees());
    telemetryTable.getEntry("Module FR Drift").setNumber(absoluteEncoder_fr.getAbsolutePosition() - swerveDrive.getModules()[1].getState().angle.getDegrees());

    telemetryTable.getEntry("Module BL Absolute Angle").setNumber(absoluteEncoder_bl.getAbsolutePosition());
    telemetryTable.getEntry("Module BL Relative Angle").setNumber(swerveDrive.getModules()[2].getState().angle.getDegrees());
    telemetryTable.getEntry("Module BL Drift").setNumber(absoluteEncoder_bl.getAbsolutePosition() - swerveDrive.getModules()[2].getState().angle.getDegrees());

    telemetryTable.getEntry("Module BR Absolute Angle").setNumber(absoluteEncoder_br.getAbsolutePosition());
    telemetryTable.getEntry("Module BR Relative Angle").setNumber(swerveDrive.getModules()[3].getState().angle.getDegrees());
    telemetryTable.getEntry("Module BR Drift").setNumber(absoluteEncoder_br.getAbsolutePosition() - swerveDrive.getModules()[3].getState().angle.getDegrees());

    // Filtered Pigeon 2 accelerometer — bump spikes are smoothed out.
    double rawX = pigeon2.getAccelerationX().getValueAsDouble();
    double rawY = pigeon2.getAccelerationY().getValueAsDouble();
    double rawZ = pigeon2.getAccelerationZ().getValueAsDouble();
    double filtX = accelXFilter.calculate(rawX);
    double filtY = accelYFilter.calculate(rawY);
    double filtZ = accelZFilter.calculate(rawZ);

    telemetryTable.getEntry("Accel X Raw (g)").setNumber(rawX);
    telemetryTable.getEntry("Accel Y Raw (g)").setNumber(rawY);
    telemetryTable.getEntry("Accel Z Raw (g)").setNumber(rawZ);
    telemetryTable.getEntry("Accel X Filtered (g)").setNumber(filtX);
    telemetryTable.getEntry("Accel Y Filtered (g)").setNumber(filtY);
    telemetryTable.getEntry("Accel Z Filtered (g)").setNumber(filtZ);
  }

  @Override
  public void simulationPeriodic()
  {
    Pose2d pose = swerveDrive.getPose();
    field.setRobotPose(pose);

    swerveDrive.updateOdometry(); 
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
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
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
    return pigeon2.getPitch().getValueAsDouble();
  }

  /** Returns Pigeon 2 roll in degrees. Positive = left side up. */
  public double getRollDegrees()
  {
    return pigeon2.getRoll().getValueAsDouble();
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
    double pitchDeg = Math.abs(swerveDrive.getPitch().getDegrees());
    double zDeviation = Math.abs(accelZFilter.get() - 1.0);
    return pitchDeg > Constants.BumpDetectionConstants.kBumpPitchThresholdDegrees
        || zDeviation > Constants.BumpDetectionConstants.kBumpAccelZDeviationThreshold;
  }

  public double getDistanceToHub() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Blue hub is near x=4.597m, Red hub is near x=11.938m
    Translation2d hubTranslation = alliance == Alliance.Red ?
          new Translation2d(11.938, 4.035) : new Translation2d(4.597, 4.035);
    Translation2d robotTranslation = swerveDrive.getPose().getTranslation();
    return robotTranslation.getDistance(hubTranslation);
  }
}
