// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Hanger.Position;

import java.io.File;
import java.util.Optional;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();
    private final Limelight limelight = new Limelight("limelight-front");
    private final Field2d field = new Field2d();
    private final Swerve drivebase  = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"), field);

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
        intake,
        floor,
        feeder,
        shooter,
        hood,
        //hanger,
        () -> -driverXbox.getLeftY(),
        () -> -driverXbox.getLeftX()
    );
    
    // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                  () -> driverXbox.getLeftY() * -1,
                                                                  () -> driverXbox.getLeftX() * -1)
                                                              .withControllerRotationAxis(driverXbox::getRightX)
                                                              //.aim(new Pose2d(Landmarks.hubPosition(), new Rotation2d()))                                                           
                                                              .deadband(OperatorConstants.DEADBAND)
                                                              .scaleTranslation(0.8)
                                                              .allianceRelativeControl(true);

    public RobotContainer()
    {
      // Configure the trigger bindings
      configureBindings();
      DriverStation.silenceJoystickConnectionWarning(true);

      //limelight.setDefaultCommand(updateVisionCommand());

      SmartDashboard.putData("Field", field);

      // TODO: Uncomment when subsystem commands are implemented
      //driverXbox.rightBumper().whileTrue(floor.feedCommand());
      //driverXbox.leftTrigger().whileTrue(feeder.reverseCommand());
      //driverXbox.leftBumper().whileTrue(Commands.parallel(Commands.sequence(Commands.waitSeconds(2.0), feeder.feedCommand()), shooter.spinUpCommand()));



        //dis is js fer testing dem ackcheuwators ;)
        //driverXbox.x().whileTrue(hood.positionCommand(1.5));
        //driverXbox.a().whileTrue(hood.positionCommand(0.15));

        //And dis thingy down here is foar testing da chew arm thingys that go up'n down (currently not here)

        driverXbox.leftTrigger().whileTrue(subsystemCommands.shootManually());
        driverXbox.leftBumper().whileTrue(feeder.reverseCommand());

        driverXbox.rightTrigger().whileTrue(intake.intakeCommand());
        driverXbox.rightBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));



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

      // TODO: Uncomment when hanger commands are implemented
       driverXbox.y().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
       driverXbox.b().onTrue(hanger.positionCommand(Hanger.Position.HOMED));

      //Set the default auto (do nothing) 
      // autoChooser.setDefaultOption("Do Nothing", Commands.runOnce(drivebase::zeroGyroWithAlliance)
      //                                                 .andThen(Commands.none()));

      // //Add a simple auto option to have the robot drive forward for 1 second then stop
      // autoChooser.addOption("Drive Forward", Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2)
      //                                             .andThen(drivebase.driveForward().withTimeout(1)));
      // //Put the autoChooser on the SmartDashboard
      // SmartDashboard.putData("Auto Chooser", autoChooser);

      // if (autoChooser.getSelected() == null ) {
      //   RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
      // }
    }

    private void configureBindings()
    {
      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
      // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
      return autoChooser.getSelected();
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

    // private Command updateVisionCommand() {
    //     return limelight.run(() -> {
    //         final Pose2d currentRobotPose = drivebase.getPose();
    //         final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
    //         measurement.ifPresent(m -> {
    //             drivebase.addVisionMeasurement(
    //                 m.poseEstimate.pose, 
    //                 m.poseEstimate.timestampSeconds,
    //                 m.standardDeviations
    //             );
    //         });
    //     })
    //     .ignoringDisable(true);
    // }
}