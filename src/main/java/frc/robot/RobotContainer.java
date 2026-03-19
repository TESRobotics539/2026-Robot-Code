// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.vision.VisionManager;
import frc.robot.subsystems.iodiagnostics.LimelightIOReal;
import frc.robot.subsystems.robot.BlinkinLed;
import frc.robot.subsystems.robot.Feeder;
import frc.robot.subsystems.robot.Floor;
import frc.robot.subsystems.robot.Hanger;
import frc.robot.subsystems.robot.Intake;
import frc.robot.subsystems.robot.Swerve;
import frc.robot.subsystems.robot.UltraShooter;
import frc.robot.subsystems.iodiagnostics.FeederIOReal;
import frc.robot.subsystems.iodiagnostics.FloorIOReal;
import frc.robot.subsystems.iodiagnostics.HangerIOReal;
import frc.robot.subsystems.iodiagnostics.IntakeIOReal;
import frc.robot.subsystems.iodiagnostics.UltraShooterIOReal;
// import frc.robot.subsystems.tuning.BumpTuner;       // Pi-backed live bump tuning (disabled)
// import frc.robot.subsystems.tuning.ShooterAutoTuner; // Pi-backed auto shooter tuning (disabled)
// import frc.robot.subsystems.tuning.ShooterTuner;     // Pi-backed live shooter tuning (disabled)
//import frc.robot.subsystems.tuning.ShooterOrca; // deprecated — replaced by UltraShooter

import java.io.File;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // ── Vision ───────────────────────────────────────────────────────────────
    private final BlinkinLed blinkinLed = new BlinkinLed();

    // ── Drive ────────────────────────────────────────────────────────────────
    private final Field2d field     = new Field2d();
    private final Swerve  drivebase = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"), field);

    // VisionManager must be constructed after drivebase (it holds a reference to it).
    private final VisionManager visionManager = new VisionManager(
        new LimelightIOReal(Ports.kLimelightFront),
        new LimelightIOReal(Ports.kLimelightRear),
        drivebase
    );

    // ── Subsystems ───────────────────────────────────────────────────────────
    private final Intake  intake  = new Intake(new IntakeIOReal());
    private final Floor   floor   = new Floor(new FloorIOReal());
    private final Feeder  feeder  = new Feeder(new FeederIOReal());
    private final Hanger  hanger  = new Hanger(new HangerIOReal());
    // private final BumpTuner bumpTuner = new BumpTuner();  // Pi bump tuning (disabled)

    // ── Shooter ──────────────────────────────────────────────────────────────
    // private final ShooterTuner     shooterTuner     = new ShooterTuner();     // Pi shooter tuning (disabled)
    private final UltraShooter     ultraShooter     = new UltraShooter(new UltraShooterIOReal(), drivebase);
    // private final ShooterAutoTuner shooterAutoTuner = new ShooterAutoTuner(ultraShooter, shooterTuner);

    // Pre-spin during hub-active windows; interrupted automatically by any shoot command
    // TODO: re-enable after PID tuning
    // {
    //     ultraShooter.setDefaultCommand(ultraShooter.preSpinCommand());
    // }

    // ── Controllers ──────────────────────────────────────────────────────────
    final CommandXboxController driverXbox = new CommandXboxController(0);
    private double lastIntakeTriggerPressTime = Double.NEGATIVE_INFINITY;

    // ── Command Factories ────────────────────────────────────────────────────
    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        drivebase,
        floor,
        feeder,
        ultraShooter,
        intake,
        () -> -driverXbox.getLeftY(),
        () -> -driverXbox.getLeftX()
    );

    // ── Auto ─────────────────────────────────────────────────────────────────
    private final SendableChooser<Command> autoChooser;

    // ── Drive Input Stream ───────────────────────────────────────────────────
    // x^1.5 curve softens fine control near center while preserving full speed at edges.
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                  () -> MathUtil.copyDirectionPow(driverXbox.getLeftY() * -1, 1.5),
                                                                  () -> MathUtil.copyDirectionPow(driverXbox.getLeftX() * -1, 1.5))
                                                              .withControllerRotationAxis(() -> MathUtil.copyDirectionPow(driverXbox.getRightX() * -1, 1.5))
                                                              //.aim(new Pose2d(Landmarks.hubPosition(), new Rotation2d()))
                                                              .deadband(OperatorConstants.DEADBAND)
                                                              .scaleTranslation(1.0)
                                                              .allianceRelativeControl(true);

    public RobotContainer()
    {
        // Auto — register named commands before building the chooser
        configureNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();

        // Bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        // Default commands
        blinkinLed.setDefaultCommand(
            Commands.run(blinkinLed::setPhasePattern, blinkinLed)
                .finallyDo(__ -> blinkinLed.setDefaultPattern()));

        // Dashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Field", field);
    }

    private void configureBindings()
    {
        // ── Mode triggers ────────────────────────────────────────────────────
        // Zero gyro and reset fuel detection at the start of every teleop period.
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
        RobotModeTriggers.teleop().onTrue(intake.runOnce(intake::resetFuelDetection));

        // At autonomous start: enforce brake mode and snapshot the current encoder position
        // as the PID target. If kStowIntakeForMatch is enabled, the pivot is locked to that
        // position for the rest of the match.
        RobotModeTriggers.autonomous().onTrue(intake.runOnce(() -> {
            intake.enforceBrakeMode();
            intake.lockCurrentPositionAsStow();
        }));

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

        // ── Drive ────────────────────────────────────────────────────────────
        drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));

        // Manual mid-match gyro reset — press the Back (View) button on the Xbox controller
        driverXbox.back().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));

        // ── Intake ───────────────────────────────────────────────────────────
        // Single pull → deploy/toggle rollers. Double-tap → stow.
        driverXbox.leftTrigger().onTrue(
            Commands.defer(() -> {
                double now = Timer.getFPGATimestamp();
                boolean isDoubleTap = (now - lastIntakeTriggerPressTime)
                    < Constants.IntakeConstants.kDoubleTapWindowSeconds;
                lastIntakeTriggerPressTime = now;
                return isDoubleTap
                    ? intake.stowCommand()
                        .andThen(Commands.runOnce(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, Constants.RumbleConstants.kIntakeStowIntensity)))
                        .andThen(Commands.waitSeconds(Constants.RumbleConstants.kIntakeStowPulseSeconds))
                        .andThen(Commands.runOnce(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.0)))
                    : intake.intakePressCommand();
            }, Set.of(intake)));

        // ── Shooter ──────────────────────────────────────────────────────────
        // Right bumper: spin up → wait for ready → feed
        driverXbox.rightBumper().whileTrue(
            Commands.defer(() -> Commands.parallel(
                ultraShooter.spinUpPhysicsCommand(),
                intake.agitateCommand(),
                // Pulse the entire time the button is held, from first press.
                // Cleared by finallyDo() when the sequence ends or is interrupted.
                Commands.sequence(
                    Commands.runOnce(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, Constants.RumbleConstants.kShootPulseIntensity)),
                    Commands.waitSeconds(Constants.RumbleConstants.kShootPulseOnSeconds),
                    Commands.runOnce(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.0)),
                    Commands.waitSeconds(Constants.RumbleConstants.kShootPulseOffSeconds)
                ).repeatedly(),
                Commands.waitUntil(ultraShooter::isReady)
                    .withTimeout(Constants.ShooterConstants.kShootReadyTimeoutSeconds)
                    .andThen(Commands.waitSeconds(Constants.ShooterConstants.kShootWaitSeconds))
                    .andThen(Commands.parallel(
                        feeder.feedCommand(),
                        Commands.waitSeconds(Constants.ShooterConstants.kFloorFeedDelaySeconds).andThen(floor.feedCommand())
                    ))
            ), Set.of(ultraShooter, intake, feeder, floor))
            .finallyDo(interrupted -> {
                driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                Logger.recordOutput("Commands/SpinUpShoot/Interrupted", interrupted);
            }));

        // ══ PID TUNING — delete this entire block when done ══════════════════
        // A: toggle flywheel on/off at 3,000 RPM
        final double kTuning200RpmInFPS  = 1000.0 * Math.PI * (4.0 / 12.0) / 60.0;
        final double kTuningInitialFPS   = 3000.0 * Math.PI * (4.0 / 12.0) / 60.0;
        driverXbox.a().toggleOnTrue(
            ultraShooter.startEnd(
                () -> ultraShooter.setTarget(kTuningInitialFPS),
                ultraShooter::stop
            ).withName("TuningFlywheelSpin"));
        // X: run feeder + floor at 85% while held (dump shot disabled during tuning)
        driverXbox.x().whileTrue(Commands.parallel(
            feeder.startEnd(() -> feeder.setPercentOutput(0.85), () -> feeder.setPercentOutput(0)),
            floor.startEnd(() -> floor.setPercentOutput(0.85), () -> floor.setPercentOutput(0))
        ).withName("TuningFeed"));
        // B: dump shot sequence while held
        driverXbox.b().whileTrue(
            Commands.defer(() -> Commands.parallel(
                ultraShooter.startEnd(() -> ultraShooter.setTarget(Constants.ShooterConstants.kDumpShotFlywheelSpeed), ultraShooter::stop),
                Commands.waitUntil(ultraShooter::isReady)
                    .withTimeout(Constants.ShooterConstants.kShootReadyTimeoutSeconds)
                    .andThen(Commands.waitSeconds(Constants.ShooterConstants.kShootWaitSeconds))
                    .andThen(Commands.parallel(
                        feeder.feedCommand(),
                        Commands.waitSeconds(Constants.ShooterConstants.kFloorFeedDelaySeconds).andThen(floor.feedCommand())
                    ))
            ), Set.of(ultraShooter, feeder, floor)));
        // ═════════════════════════════════════════════════════════════════════

        // Right trigger: aim + shoot (physics-based, only while hub is active)
        driverXbox.rightTrigger().and(() -> GameData.isHubActiveExpanded(5.0)).whileTrue(
            Commands.parallel(
                // Single announcement pulse the moment the shot sequence begins.
                Commands.sequence(
                    Commands.runOnce(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, Constants.RumbleConstants.kShootPulseIntensity)),
                    Commands.waitSeconds(Constants.RumbleConstants.kShootAnnouncementOnSeconds),
                    Commands.runOnce(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.0))
                ),
                subsystemCommands.shootPhysicsWithFeedExtra(
                    // Supplier so a fresh command is created each execution (avoids composed-command reuse).
                    () -> Commands.sequence(
                        Commands.runOnce(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, Constants.RumbleConstants.kShootPulseIntensity)),
                        Commands.waitSeconds(Constants.RumbleConstants.kShootPulseOnSeconds),
                        Commands.runOnce(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.0)),
                        Commands.waitSeconds(Constants.RumbleConstants.kShootPulseOffSeconds)
                    ).repeatedly()
                ),
                intake.agitateCommand()
            ).finallyDo(interrupted -> {
                driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                Logger.recordOutput("Commands/AimShoot/Interrupted", interrupted);
            }))
            .onFalse(Commands.parallel(
                subsystemCommands.holdAimAndSpeedCommand(1.0),
                intake.runOnce(intake::resetFuelDetection)));

        // ── Feeder ───────────────────────────────────────────────────────────
        driverXbox.leftBumper().whileTrue(feeder.reverseCommand());

        // ── Hanger ───────────────────────────────────────────────────────────
        // D-pad up/down → manual hanger control
        // ══ PID TUNING: hanger bindings temporarily replaced by flywheel speed adjust ══
        // driverXbox.povUp().whileTrue(hanger.run(() -> hanger.setPercentOutput(Constants.HangerConstants.kManualUpPower)))
        //                   .onFalse(hanger.runOnce(() -> hanger.setPercentOutput(0)));
        // driverXbox.povDown().whileTrue(hanger.run(() -> hanger.setPercentOutput(Constants.HangerConstants.kManualDownPower)))
        //                     .onFalse(hanger.runOnce(() -> hanger.setPercentOutput(0)));
        // D-pad up/down → +/- 10 ft/s flywheel speed adjust (delete when done)
        // Uses Commands.runOnce (no subsystem requirement) so it doesn't interrupt TuningFlywheelSpin.
        driverXbox.povUp().onTrue(Commands.runOnce(
            () -> ultraShooter.setTarget(ultraShooter.getTarget() + kTuning200RpmInFPS)));
        driverXbox.povDown().onTrue(Commands.runOnce(
            () -> ultraShooter.setTarget(ultraShooter.getTarget() - kTuning200RpmInFPS)));
        // ═════════════════════════════════════════════════════════════════════

        // Y: auto climb; also spin down shooter in last 30 seconds of teleop
        driverXbox.y().onTrue(hanger.autoClimbCommand()
            .finallyDo(interrupted -> Logger.recordOutput("Commands/AutoClimb/Interrupted", interrupted)));
        driverXbox.y()
            .and(() -> DriverStation.isTeleop() && DriverStation.getMatchTime() <= 30)
            .onTrue(ultraShooter.spinDownCommand());

        // Slow celebratory pulse when the climb sequence completes during teleop.
        // Pattern is distinct from the fast shoot pulse — see RumbleConstants for timings.
        // Runs until kClimbSuccessDurationSeconds have elapsed or the match ends.
        new Trigger(hanger::isAutoClimbCompleted)
            .and(RobotModeTriggers.teleop())
            .onTrue(
                Commands.sequence(
                    Commands.runOnce(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, Constants.RumbleConstants.kClimbSuccessIntensity)),
                    Commands.waitSeconds(Constants.RumbleConstants.kClimbSuccessPulseOnSeconds),
                    Commands.runOnce(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.0)),
                    Commands.waitSeconds(Constants.RumbleConstants.kClimbSuccessPulseOffSeconds)
                ).repeatedly()
                .withTimeout(Constants.RumbleConstants.kClimbSuccessDurationSeconds)
                .finallyDo(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.0))
                .withName("ClimbSuccessRumble"));

        // ── Feedback ─────────────────────────────────────────────────────────
        // Brief double-pulse rumble on each roller current spike so the driver
        // feels every fuel ball enter the robot without looking down.
        new Trigger(intake::consumeRollerSpike)
            .and(RobotModeTriggers.teleop())
            .onTrue(Commands.runOnce(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, Constants.RumbleConstants.kFuelPickupIntensity))
                .andThen(Commands.waitSeconds(Constants.RumbleConstants.kFuelPickupPulseOnSeconds))
                .andThen(Commands.runOnce(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.0)))
                .andThen(Commands.waitSeconds(Constants.RumbleConstants.kFuelPickupPauseBetweenSeconds))
                .andThen(Commands.runOnce(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, Constants.RumbleConstants.kFuelPickupIntensity)))
                .andThen(Commands.waitSeconds(Constants.RumbleConstants.kFuelPickupPulseOnSeconds))
                .andThen(Commands.runOnce(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.0)))
                .withName("FuelPickupRumble"));

        // ── Utility ──────────────────────────────────────────────────────────
        // Emergency stop — cancels every running command immediately.
        // Bind to Start so it's reachable without looking down at the controller.
        // driverXbox.start().onTrue(Commands.runOnce(CommandScheduler.getInstance()::cancelAll)
        //     .ignoringDisable(true).withName("KillAll"));

    }

    private void configureNamedCommands() {
      NamedCommands.registerCommand("Shoot Command", subsystemCommands.shootPhysics().withTimeout(5.0));
      NamedCommands.registerCommand("Auto Shoot", subsystemCommands.autoShoot().andThen(ultraShooter.spinDownCommand()));
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
        // Tracks whether we have already seeded odometry during the current disabled period.
        // Array trick lets us mutate from inside the lambda while satisfying "effectively final".
        final boolean[] disabledSeedDone = {false};
        final boolean[] prevEnabled      = {false};

        return Commands.run(() -> {
            final Pose2d currentRobotPose = drivebase.getPose();
            final boolean isEnabled = DriverStation.isEnabled();

            // Reset seed flag at the start of each enabled period so the next disabled
            // period can seed again with fresh field placement.
            if (isEnabled && !prevEnabled[0]) {
                disabledSeedDone[0] = false;
            }
            prevEnabled[0] = isEnabled;

            // 2-of-3 majority vote across three independent sensors at different robot locations.
            // Requires at least two to agree before inflating stddevs, avoiding false positives
            // from single-sensor vibration or noise.
            // NOTE: getAccelZ() reads the Limelight's physical accelerometer and is independent
            // of IMU mode — the mode-switching in Limelight.periodic() does not affect this vote.
            boolean pigeonOverBump = drivebase.isOverBump();
            boolean frontOverBump  = Math.abs(limelight.getAccelZ() - 1.0)
                > Constants.BumpDetectionConstants.kLimelightAccelZDeviationThreshold;
            boolean rearOverBump   = Math.abs(limelightRear.getAccelZ() - 1.0)
                > Constants.BumpDetectionConstants.kLimelightAccelZDeviationThreshold;
            int bumpVotes = (pigeonOverBump ? 1 : 0) + (frontOverBump ? 1 : 0) + (rearOverBump ? 1 : 0);
            boolean overBump = bumpVotes >= 2;

            // Pigeon 2 full orientation — forwarded to both Limelights so MegaTag2 can compensate
            // for camera tilt during bump traversal and correct for dynamic rotation on all axes.
            double pitchDeg        = drivebase.getPitchDegrees();
            double pitchRateDegPS  = drivebase.getPitchRateDegPerSec();
            double rollDeg         = drivebase.getRollDegrees();
            double rollRateDegPS   = drivebase.getRollRateDegPerSec();
            double yawRateDegPS    = drivebase.getYawRateDegPerSec();

            // ── Disabled-period odometry seeding ─────────────────────────────
            // While disabled, Limelights are in SyncInternalImu mode so MT1 can solve heading
            // independently from tag geometry — no gyro dependency. Seed odometry once per
            // disabled period from the camera with the most tags (≥2 required).
            if (!isEnabled && !disabledSeedDone[0]) {
                var frontMT1 = limelight.getMT1Pose();
                var rearMT1  = limelightRear.getMT1Pose();
                Optional<Pose2d> seedPose = Optional.empty();
                if (frontMT1.isPresent() && rearMT1.isPresent()) {
                    // Both cameras have a fix — prefer the front (arbitrary tiebreak; both are valid).
                    seedPose = frontMT1;
                } else if (frontMT1.isPresent()) {
                    seedPose = frontMT1;
                } else if (rearMT1.isPresent()) {
                    seedPose = rearMT1;
                }
                if (seedPose.isPresent()) {
                    drivebase.resetOdometry(seedPose.get());
                    disabledSeedDone[0] = true;
                    Logger.recordOutput("Vision/DisabledSeed", seedPose.get());
                }
            }

            // Request measurements from both Limelights (each call also sends SetRobotOrientation
            // so MegaTag2 keeps a fresh heading even for the camera that isn't chosen).
            var frontMeasurement = limelight.getMeasurement(
                currentRobotPose, pitchDeg, pitchRateDegPS, rollDeg, rollRateDegPS, yawRateDegPS);
            var rearMeasurement  = limelightRear.getMeasurement(
                currentRobotPose, pitchDeg, pitchRateDegPS, rollDeg, rollRateDegPS, yawRateDegPS);

            // Confidence = avgTagArea × tagCount. Larger tag area means the robot is closer to
            // the tags and the projection error is smaller; more tags further reduce ambiguity.
            double frontConf = frontMeasurement.map(
                m -> m.avgTagArea * m.poseEstimate.tagCount).orElse(0.0);
            double rearConf  = rearMeasurement.map(
                m -> m.avgTagArea * m.poseEstimate.tagCount).orElse(0.0);
            double bestConf  = Math.max(frontConf, rearConf);

            // Wheel-slip detection — both signals must agree to avoid false positives.
            boolean isSlipping = drivebase.isWheelSlipping();
            boolean highConf   = bestConf >= Constants.BumpDetectionConstants.kHighConfidenceThreshold;

            // Publish for Elastic so drivers can verify which camera is active.
            String activeSource;
            var bestMeasurement = frontMeasurement.isEmpty() && rearMeasurement.isEmpty()
                ? java.util.Optional.<frc.robot.subsystems.vision.Limelight.Measurement>empty()
                : (frontConf >= rearConf ? frontMeasurement : rearMeasurement);
            if (frontMeasurement.isEmpty() && rearMeasurement.isEmpty()) {
                activeSource = "none";
            } else if (frontConf >= rearConf && frontMeasurement.isPresent()) {
                activeSource = "front (" + String.format("%.3f", frontConf) + ")";
            } else {
                activeSource = "rear (" + String.format("%.3f", rearConf) + ")";
            }
            Logger.recordOutput("Vision/ActiveSource",   activeSource);
            Logger.recordOutput("Vision/FrontConfidence", frontConf);
            Logger.recordOutput("Vision/RearConfidence",  rearConf);
            Logger.recordOutput("Vision/BestConfidence",  bestConf);
            Logger.recordOutput("Vision/WheelSlipping",   isSlipping);
            Logger.recordOutput("Vision/WheelSlipScore",  drivebase.getWheelSlipScore());
            Logger.recordOutput("Vision/Healthy",         limelight.isVisionHealthy() || limelightRear.isVisionHealthy());
            Logger.recordOutput("Vision/TotalTagCount",   limelight.getLastTagCount() + limelightRear.getLastTagCount());

            if (bestMeasurement.isPresent()) {
                var m = bestMeasurement.get();
                // Confidence-gated slip handling:
                //
                // • Slipping + high confidence → wheels are unreliable, but Limelights have a
                //   solid tag fix. Shrink stddevs to let vision actively correct the drifted pose.
                //
                // • Slipping + low confidence  → neither odometry nor vision is reliable.
                //   Inflate stddevs so the bad camera pose doesn't corrupt the estimate.
                //
                // • Not slipping              → normal stddevs from getMeasurement().
                double stdDevMultiplier;
                if (isSlipping && highConf) {
                    stdDevMultiplier = Constants.BumpDetectionConstants.kSlipHighConfStdDevMultiplier;
                } else if (isSlipping) {
                    stdDevMultiplier = Constants.BumpDetectionConstants.kBumpVisionStdDevMultiplier;
                } else {
                    stdDevMultiplier = 1.0;
                }
                Logger.recordOutput("Vision/StdDevMultiplier", stdDevMultiplier);
                double ts = m.poseEstimate.timestampSeconds;
                // Reject timestamps of 0 or in the future — these indicate a stale or
                // invalid Limelight frame and would corrupt the pose estimator.
                if (ts > 0 && ts <= Timer.getFPGATimestamp()) {
                    var stdDevs = m.standardDeviations.times(stdDevMultiplier);
                    drivebase.addVisionMeasurement(m.poseEstimate.pose, ts, stdDevs);
                }
            }
            // else if (!overBump) {
            //     piAprilTag.getMeasurement().ifPresent(m -> drivebase.addVisionMeasurement(
            //         m.pose, m.timestampSeconds, m.standardDeviations
            //     ));
            // }
        }, limelight, limelightRear).ignoringDisable(true);
    }
}
