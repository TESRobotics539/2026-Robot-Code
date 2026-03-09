// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.GameData;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.BlinkinLed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterOrca;
import frc.robot.subsystems.Swerve;

import java.io.File;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    private final BlinkinLed blinkinLed = new BlinkinLed();

    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();
    private final Limelight limelight = new Limelight("limelight-front");
    private final Field2d field = new Field2d();
    private final Swerve drivebase  = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"), field);
    private final ShooterOrca shooter = new ShooterOrca(drivebase);

    // Pre-spin during hub-active windows; interrupted automatically by any shoot command
    {
        shooter.setDefaultCommand(shooter.preSpinCommand(intake::hasPickedUpFuel));
    }

    // private final Pivot pivot = new Pivot();

    final CommandXboxController driverXbox = new CommandXboxController(0);
    private double lastIntakeTriggerPressTime = Double.NEGATIVE_INFINITY;
    
    // private final AutoRoutines autoRoutines = new AutoRoutines(
    //     drivebase,
    //     intake,
    //     floor,
    //     feeder,
    //     shooter,
    //     hood,
    //     hanger,
    //     limelight
    // );

    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        drivebase,
        //intake,
        floor,
        feeder,
        shooter,
        hood,
        //hanger,
        () -> -driverXbox.getLeftY(),
        () -> -driverXbox.getLeftX()
    );
    
    // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
    private final SendableChooser<Command> autoChooser;

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                  () -> driverXbox.getLeftY() * -1,
                                                                  () -> driverXbox.getLeftX() * -1)
                                                              .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                              //.aim(new Pose2d(Landmarks.hubPosition(), new Rotation2d()))                                                           
                                                              .deadband(OperatorConstants.DEADBAND)
                                                              .scaleTranslation(1.0)
                                                              .allianceRelativeControl(true);

    public RobotContainer()
    {
      configureNamedCommands();
      autoChooser = AutoBuilder.buildAutoChooser();

      // Configure the trigger bindings
      configureBindings();
      DriverStation.silenceJoystickConnectionWarning(true);

      limelight.setDefaultCommand(updateVisionCommand());


      blinkinLed.setDefaultCommand(Commands.run(blinkinLed::setPhasePattern, blinkinLed));


      SmartDashboard.putData("Auto Chooser", autoChooser);
      SmartDashboard.putData("Field", field);

      // TODO: Uncomment when subsystem commands are implemented
      //driverXbox.rightBumper().whileTrue(floor.feedCommand());
      //driverXbox.leftTrigger().whileTrue(feeder.reverseCommand());
      //driverXbox.leftBumper().whileTrue(Commands.parallel(Commands.sequence(Commands.waitSeconds(2.0), feeder.feedCommand()), shooter.spinUpCommand()));



        //driverXbox.x().whileTrue(hood.positionCommand(1.5));
        //driverXbox.a().whileTrue(hood.positionCommand(0.15));

        // Hood tracking: continuously adjust position based on distance to hub
        hood.setDefaultCommand(hood.trackHubCommand(drivebase::getPose));

        // At autonomous start: enforce brake mode and snapshot the current encoder position
        // as the PID target. If kStowIntakeForMatch is enabled, the pivot is locked to that
        // position for the rest of the match.
        RobotModeTriggers.autonomous().onTrue(intake.runOnce(() -> {
            intake.enforceBrakeMode();
            intake.lockCurrentPositionAsStow();
        }));

        // driverXbox.leftTrigger().whileTrue(subsystemCommands.shootManually());
        driverXbox.leftBumper().whileTrue(feeder.reverseCommand());

        // At teleop start: if auto climb completed during autonomous, run the declimb sequence
        // first, then wait until the robot has driven 2 meters, then deploy the intake.
        // Otherwise, deploy the intake normally after 1 second.
        RobotModeTriggers.teleop().onTrue(
            Commands.defer(() -> {
                if (hanger.isAutoClimbCompleted()) {
                    Pose2d startPose = drivebase.getPose();
                    return hanger.reverseClimbIfNeededCommand()
                        .andThen(Commands.waitUntil(() ->
                            drivebase.getPose().getTranslation()
                                .getDistance(startPose.getTranslation()) >= 2.0))
                        .andThen(intake.runOnce(intake::setInitialDeployPosition));
                } else {
                    return Commands.waitSeconds(1.0)
                        .andThen(intake.runOnce(intake::setInitialDeployPosition));
                }
            }, Set.of(hanger, intake)));

        // Single pull → deploy/toggle rollers. Double-tap → stow.
        driverXbox.leftTrigger().onTrue(
            Commands.defer(() -> {
                double now = Timer.getFPGATimestamp();
                boolean isDoubleTap = (now - lastIntakeTriggerPressTime)
                    < Constants.IntakeConstants.kDoubleTapWindowSeconds;
                lastIntakeTriggerPressTime = now;
                return isDoubleTap ? intake.stowCommand() : intake.intakePressCommand();
            }, Set.of(intake)));
        driverXbox.rightBumper().whileTrue(
            Commands.parallel(
                shooter.spinUpMapCommand(),
                intake.agitateCommand(),
                Commands.waitUntil(shooter::isShooterReady)
                    .withTimeout(Constants.ShooterConstants.kShootReadyTimeoutSeconds)
                    .andThen(Commands.waitSeconds(Constants.ShooterConstants.kShootWaitSeconds))
                    .andThen(Commands.parallel(
                        feeder.feedCommand(),
                        Commands.waitSeconds(Constants.ShooterConstants.kFloorFeedDelaySeconds).andThen(floor.feedCommand())
                    ))
            ));
        driverXbox.x().whileTrue(
            Commands.parallel(
                shooter.startEnd(() -> shooter.setShooterTarget(Constants.ShooterConstants.kDumpShotFlywheelRPM), () -> shooter.stop()),
                Commands.waitUntil(shooter::isShooterReady)
                    .withTimeout(Constants.ShooterConstants.kShootReadyTimeoutSeconds)
                    .andThen(Commands.waitSeconds(Constants.ShooterConstants.kShootWaitSeconds))
                    .andThen(Commands.parallel(
                        feeder.feedCommand(),
                        Commands.waitSeconds(Constants.ShooterConstants.kFloorFeedDelaySeconds).andThen(floor.feedCommand())
                    ))
            ));
        driverXbox.rightTrigger().and(() -> GameData.isHubActiveExpanded(5.0)).whileTrue(
            Commands.parallel(subsystemCommands.shootMap(), intake.agitateCommand()))
            .onFalse(Commands.parallel(
                subsystemCommands.holdAimAndSpeedCommand(1.5),
                intake.runOnce(intake::resetFuelDetection)));

      // ORIGINAL COMMANDS
      // RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
      //     .onTrue(intake.homingCommand())
      //     .onTrue(hanger.homingCommand());

      // TODO: Uncomment when shoot commands are implemented
      //driverXbox.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
      //driverXbox.rightBumper().whileTrue(subsystemCommands.shootManually());

      // TODO: Uncomment when intake commands are implemented
      // driverXbox.leftTrigger().whileTrue(intake.intakeCommand());
      // driverXbox.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));

      // D-pad up/down → manual hanger control
      driverXbox.povUp().whileTrue(hanger.run(() -> hanger.setPercentOutput(Constants.HangerConstants.kManualUpPower)))
                        .onFalse(hanger.runOnce(() -> hanger.setPercentOutput(0)));
      driverXbox.povDown().whileTrue(hanger.run(() -> hanger.setPercentOutput(Constants.HangerConstants.kManualDownPower)))
                          .onFalse(hanger.runOnce(() -> hanger.setPercentOutput(0)));
      // D-pad left/right → hood fully retracted / fully extended
      driverXbox.povLeft().toggleOnTrue(hood.holdPositionCommand(Constants.HoodConstants.kMinPosition));
      driverXbox.povRight().toggleOnTrue(hood.holdPositionCommand(Constants.HoodConstants.kMaxPosition));
      driverXbox.a().onTrue(
          hanger.runOnce(() -> hanger.setPercentOutput(0.5))
              .andThen(Commands.waitSeconds(0.33))
              .andThen(hanger.runOnce(() -> hanger.setPercentOutput(0))));

      driverXbox.y().onTrue(hanger.autoClimbCommand());
      driverXbox.y()
          .and(() -> DriverStation.isTeleop() && DriverStation.getMatchTime() <= 30)
          .onTrue(shooter.spinDownCommand());
      // driverXbox.start().onTrue(hanger.positionCommand(Hanger.Position.HOMED));
      // driverXbox.b().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
    }

    private void configureBindings()
    {
      // Zero gyro to field-forward every time teleop starts
      RobotModeTriggers.teleop().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
      // Reset fuel detection so prespin only activates once the driver picks up fuel
      RobotModeTriggers.teleop().onTrue(intake.runOnce(intake::resetFuelDetection));

      // Manual mid-match gyro reset — press the Back (View) button on the Xbox controller
      driverXbox.back().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));

      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    private void configureNamedCommands() {
      NamedCommands.registerCommand("Shoot Command", subsystemCommands.shootMap().withTimeout(5.0));
      NamedCommands.registerCommand("Auto Shoot", subsystemCommands.autoShoot().andThen(shooter.spinDownCommand()));
      NamedCommands.registerCommand("Climber Toggle Command", hanger.toggleCommand());
      NamedCommands.registerCommand("Climber Down and Hold", hanger.autoClimbCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
      // Pass in the selected auto from the SmartDashboard as our desired autonomous command
      Command selected = autoChooser.getSelected();
      return selected != null ? selected : Commands.none();
    }

    public void resetOdometry(Pose2d pose)
    {
      drivebase.resetOdometry(pose);
    }

    public void zeroGyroWithAlliance()
    {
      drivebase.zeroGyroWithAlliance();
    }

    public void setMotorBrake(boolean brake)
    {
      drivebase.setMotorBrake(brake);
    }

    private Command updateVisionCommand() {
        return Commands.run(() -> {
            final Pose2d currentRobotPose = drivebase.getPose();
            limelight.getMeasurement(currentRobotPose).ifPresent(m -> drivebase.addVisionMeasurement(
                m.poseEstimate.pose,
                m.poseEstimate.timestampSeconds,
                m.standardDeviations
            ));
        }, limelight).ignoringDisable(true);
    }
}
