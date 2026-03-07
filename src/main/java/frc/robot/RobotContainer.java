// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
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
    private final Limelight limelightRear = new Limelight("limelight-rear");
    private final Field2d field = new Field2d();
    private final Swerve drivebase  = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"), field);
    private final ShooterOrca shooter = new ShooterOrca(drivebase);

    // private final Pivot pivot = new Pivot();

    final CommandXboxController  driverXbox = new CommandXboxController(0);
    
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


      blinkinLed.setDefaultCommand(Commands.run(blinkinLed::setDefaultPattern, blinkinLed));


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
        driverXbox.povDown().toggleOnTrue(hood.holdPositionCommand(0.1));

        // driverXbox.leftTrigger().whileTrue(subsystemCommands.shootManually());
        driverXbox.leftBumper().whileTrue(feeder.reverseCommand());

        driverXbox.leftTrigger()
            .onTrue(intake.intakePressCommand())
            .onFalse(intake.cancelPressCommand());
        driverXbox.rightBumper().whileTrue(
            Commands.parallel(
                shooter.spinUpMapCommand(),
                intake.agitateCommand(),
                Commands.waitUntil(shooter::isShooterReady)
                    .withTimeout(0.75)
                    .andThen(Commands.waitSeconds(Constants.shootWaitSeconds))
                    .andThen(Commands.parallel(
                        feeder.feedCommand(),
                        Commands.waitSeconds(0.125).andThen(floor.feedCommand())
                    ))
            ));
        driverXbox.rightTrigger().whileTrue(
            Commands.parallel(subsystemCommands.shootMap(), intake.agitateCommand()));

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

      driverXbox.x().onTrue(hanger.runOnce(() -> hanger.setPercentOutput(-0.3)))
                    .onFalse(hanger.runOnce(() -> hanger.setPercentOutput(0)));
      driverXbox.povLeft().whileTrue(hanger.run(() -> hanger.setPercentOutput(-0.8)))
                          .onFalse(hanger.runOnce(() -> hanger.setPercentOutput(0)));
      driverXbox.povRight().whileTrue(hanger.run(() -> hanger.setPercentOutput(0.8)))
                           .onFalse(hanger.runOnce(() -> hanger.setPercentOutput(0)));
      driverXbox.a().toggleOnTrue(hanger.toggleCommand());

      driverXbox.y().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
      driverXbox.start().onTrue(hanger.positionCommand(Hanger.Position.HOMED));
      driverXbox.b().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
    }

    private void configureBindings()
    {
      // Zero gyro to field-forward every time teleop starts
      RobotModeTriggers.teleop().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));

      // Manual mid-match gyro reset — press the Back (View) button on the Xbox controller
      driverXbox.back().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));

      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    private void configureNamedCommands() {
      NamedCommands.registerCommand("Shoot Command", subsystemCommands.shootMap().withTimeout(5.0));
      NamedCommands.registerCommand("Climber Toggle Command", hanger.toggleCommand());
      
      // For climber down the toggle command might suffice
      // TODO: If it doesn't make a specific command for staying off of the ground
      NamedCommands.registerCommand("Climber Down and Hold", hanger.toggleCommand());
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
            final Optional<Limelight.Measurement> front = limelight.getMeasurement(currentRobotPose);
            final Optional<Limelight.Measurement> rear = limelightRear.getMeasurement(currentRobotPose);

            final Optional<Limelight.Measurement> best;
            if (front.isPresent() && rear.isPresent()) {
                best = front.get().avgTagArea >= rear.get().avgTagArea ? front : rear;
            } else if (front.isPresent()) {
                best = front;
            } else {
                best = rear;
            }

            best.ifPresent(m -> drivebase.addVisionMeasurement(
                m.poseEstimate.pose,
                m.poseEstimate.timestampSeconds,
                m.standardDeviations
            ));
        }, limelight, limelightRear).ignoringDisable(true);
    }
}
