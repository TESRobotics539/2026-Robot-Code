package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import frc.robot.Constants;
import frc.robot.Landmarks;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.robot.Feeder;
import frc.robot.subsystems.robot.Floor;
import frc.robot.subsystems.robot.Intake;
import frc.robot.subsystems.robot.Swerve;
import frc.robot.subsystems.robot.UltraShooter;
// import frc.robot.subsystems.tuning.ShooterTuner; // Pi-backed live shooter tuning (disabled)

public final class SubsystemCommands {
    private final Swerve swerve;
    private final Floor floor;
    private final Feeder feeder;
    private final UltraShooter ultraShooter;
    private final Intake intake;
    // private final ShooterTuner shooterTuner; // Pi-backed live shooter tuning (disabled)

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public SubsystemCommands(
        Swerve swerve,
        Floor floor,
        Feeder feeder,
        UltraShooter ultraShooter,
        Intake intake,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.floor = floor;
        this.feeder = feeder;
        this.ultraShooter = ultraShooter;
        this.intake = intake;

        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
        Swerve swerve,
        Floor floor,
        Feeder feeder,
        UltraShooter ultraShooter,
        Intake intake
    ) {
        this(swerve, floor, feeder, ultraShooter, intake, () -> 0, () -> 0);
    }

    public Command shootManually() {
        return ultraShooter.spinUpPhysicsCommand()
            .andThen(feed())
            .andThen(intake.runOnce(() -> intake.setPivotPosition(Intake.Position.DEPLOYED)))
            .handleInterrupt(ultraShooter::stop);
    }

    public Command shootPhysics() {
        return aimAndFire(this::feed);
    }

    /**
     * Physics-based map shot with an extra command run in parallel during the feed phase
     * (after flywheel ready + aim settled). Use this to inject rumble, LEDs, etc.
     *
     * @param feedExtraSupplier called fresh each execution to avoid composed-command reuse
     */
    public Command shootPhysicsWithFeedExtra(Supplier<Command> feedExtraSupplier) {
        return aimAndFire(() -> feedWith(feedExtraSupplier.get()));
    }

    /**
     * Holds flywheel speed and aim for {@code seconds}, then idles the flywheel down to
     * pre-spin speed over 1 second. The aim (and therefore the swerve requirement) is
     * released after {@code seconds} so the driver regains drivetrain control immediately
     * after the hold period ends.
     */
    public Command holdAimAndSpeedCommand(double seconds) {
        return Commands.sequence(
            Commands.parallel(
                ultraShooter.run(() -> {}).withTimeout(seconds),
                new AimAndDriveCommand(swerve, forwardInput, leftInput).withTimeout(seconds)
            ),
            ultraShooter.idleDownCommand()
        );
    }

    public Command autoShoot() {
        return Commands.defer(() -> {
            AimAndDriveCommand aimCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
            return Commands.parallel(
                ultraShooter.spinUpPhysicsCommand(),
                aimCommand,
                Commands.waitUntil(() -> ultraShooter.isReady() && aimCommand.isAimed())
                    .withTimeout(Constants.ShooterConstants.kShootReadyTimeoutSeconds)
                    .andThen(longFeed())
            ).withTimeout(4.0);
        }, Set.of(swerve, ultraShooter, feeder, floor));
    }

    private Command aimAndFire(Supplier<Command> feedCommandSupplier) {
        return Commands.defer(() -> {
            AimAndDriveCommand aimCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
            Command shootSequence = Commands.parallel(
                ultraShooter.spinUpPhysicsCommand(),
                aimCommand,
                Commands.waitUntil(() -> ultraShooter.isReady() && aimCommand.isAimed())
                    .withTimeout(Constants.ShooterConstants.kShootReadyTimeoutSeconds)
                    .andThen(feedCommandSupplier.get())
                    .andThen(intake.runOnce(() -> intake.setPivotPosition(Intake.Position.DEPLOYED)))
            );

            boolean needsBackup = Constants.UltraShooterConstants.kEnableCloseRangeBackup
                && swerve.getDistanceToHub() < Units.inchesToMeters(Constants.UltraShooterConstants.kCloseRangeThresholdInches);

            return needsBackup
                ? Commands.sequence(backupFromHubCommand(), shootSequence)
                : shootSequence;
        }, Set.of(swerve, ultraShooter, feeder, floor, intake));
    }

    /**
     * Drives field-oriented away from the hub until kCloseRangeBackupInches of
     * displacement has been covered. Direction is computed once at schedule time
     * from the current robot pose, so the robot drives in a straight line.
     */
    // Drive-to-ideal-distance tuning constants — sourced from Constants.ShooterConstants.

    /**
     * Drives the robot radially toward or away from the hub until it reaches
     * the ideal shooting distance of {@value #IDEAL_SHOOT_DISTANCE_FT} ft from hub center
     * (sweet spot from physics simulation: 6–10 ft, commanded at midpoint).
     *
     * <p>The radial direction is computed once at schedule time from the robot's
     * current pose and held constant. Speed is proportional to remaining error,
     * capped at {@value #IDEAL_SHOOT_MAX_SPEED_FPS} ft/s. Command ends when the
     * robot is within {@value #IDEAL_SHOOT_TOLERANCE_FT} ft of the target.
     */
    public Command driveToIdealShootingDistanceCommand() {
        return Commands.defer(() -> {
            final Translation2d hubPos   = Landmarks.hubPosition();
            final Translation2d robotPos = swerve.getPose().getTranslation();
            final double currentDist     = robotPos.getDistance(hubPos);

            // Unit vector pointing radially outward (hub → robot), computed once.
            final double safeDist = Math.max(currentDist, 0.01);
            final double radialX  = (robotPos.getX() - hubPos.getX()) / safeDist;
            final double radialY  = (robotPos.getY() - hubPos.getY()) / safeDist;

            return swerve.driveFieldOriented(() -> {
                double distFt  = Units.metersToFeet(swerve.getPose().getTranslation().getDistance(hubPos));
                double errorFt = distFt - Constants.ShooterConstants.kIdealShootDistanceFt;  // positive → too far → drive toward hub
                // Negative speed along radial axis drives toward hub; positive drives away.
                // Convert ft/s → m/s for ChassisSpeeds.
                double speedFps = MathUtil.clamp(-Constants.ShooterConstants.kIdealShootKpFps * errorFt,
                    -Constants.ShooterConstants.kIdealShootMaxSpeedFps, Constants.ShooterConstants.kIdealShootMaxSpeedFps);
                double speedMps = Units.feetToMeters(speedFps);
                return new ChassisSpeeds(radialX * speedMps, radialY * speedMps, 0);
            }).until(() -> {
                double distFt = Units.metersToFeet(swerve.getPose().getTranslation().getDistance(hubPos));
                return Math.abs(distFt - Constants.ShooterConstants.kIdealShootDistanceFt) <= Constants.ShooterConstants.kIdealShootToleranceFt;
            });
        }, Set.of(swerve));
    }

    private Command backupFromHubCommand() {
        final double backupMeters = Units.inchesToMeters(Constants.UltraShooterConstants.kCloseRangeBackupInches);
        return Commands.defer(() -> {
            final Translation2d startPos = swerve.getPose().getTranslation();
            final Translation2d hubPos   = Landmarks.hubPosition();
            final double dist = startPos.getDistance(hubPos);

            // Unit vector pointing from hub toward robot (away from hub).
            // Clamp denominator to avoid divide-by-zero if robot spawns on top of the hub.
            final double safeDist = Math.max(dist, 0.01);
            final double awayX = (startPos.getX() - hubPos.getX()) / safeDist;
            final double awayY = (startPos.getY() - hubPos.getY()) / safeDist;

            return swerve.driveFieldOriented(() -> {
                double traveledFt   = Units.metersToFeet(swerve.getPose().getTranslation().getDistance(startPos));
                double remainingFt  = Units.metersToFeet(backupMeters) - traveledFt;
                double speedFps     = MathUtil.clamp(Constants.ShooterConstants.kIdealShootKpFps * remainingFt,
                    0, Constants.ShooterConstants.kIdealShootMaxSpeedFps);
                double speedMps = Units.feetToMeters(speedFps);
                return new ChassisSpeeds(awayX * speedMps, awayY * speedMps, 0);
            }).until(() -> swerve.getPose().getTranslation().getDistance(startPos) >= backupMeters);
        }, Set.of(swerve));
    }

    private Command feed() {
        return feedWith();
    }

    private Command feedWith(Command... extras) {
        return Commands.parallel(
            feeder.feedCommand(),
            Commands.defer(
                () -> Commands.waitSeconds(Constants.ShooterConstants.kFloorFeedDelaySeconds),
                Set.of()
            ).andThen(floor.feedCommand()),
            Commands.parallel(extras)
        );
    }

    private Command longFeed() {
        return Commands.parallel(
            feeder.feedCommand(),
            floor.feedCommand()
        ).withTimeout(Constants.ShooterConstants.kLongFeedTimeoutSeconds);
    }
}
