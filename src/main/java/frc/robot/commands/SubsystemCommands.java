package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;

import frc.robot.Constants;
import frc.robot.Landmarks;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.robot.Feeder;
import frc.robot.subsystems.robot.Floor;
import frc.robot.subsystems.robot.Swerve;
import frc.robot.subsystems.robot.UltraShooter;
// import frc.robot.subsystems.tuning.ShooterTuner; // Pi-backed live shooter tuning (disabled)

public final class SubsystemCommands {
    private final Swerve swerve;
    private final Floor floor;
    private final Feeder feeder;
    private final UltraShooter ultraShooter;
    // private final ShooterTuner shooterTuner; // Pi-backed live shooter tuning (disabled)

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public SubsystemCommands(
        Swerve swerve,
        Floor floor,
        Feeder feeder,
        UltraShooter ultraShooter,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.floor = floor;
        this.feeder = feeder;
        this.ultraShooter = ultraShooter;

        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
        Swerve swerve,
        Floor floor,
        Feeder feeder,
        UltraShooter ultraShooter
    ) {
        this(swerve, floor, feeder, ultraShooter, () -> 0, () -> 0);
    }

    public Command shootManually() {
        return ultraShooter.spinUpPhysicsCommand()
            .andThen(feed())
            .handleInterrupt(ultraShooter::stop);
    }

    public Command shootMap() {
        return aimAndFire(feed());
    }

    /**
     * Physics-based map shot with an extra command run in parallel during the feed phase
     * (after flywheel ready + aim settled). Use this to inject rumble, LEDs, etc.
     */
    public Command shootMapWithFeedExtra(Command feedExtra) {
        return aimAndFire(feedWith(feedExtra));
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

    private Command aimAndFire(Command feedCommand) {
        return Commands.defer(() -> {
            AimAndDriveCommand aimCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
            Command shootSequence = Commands.parallel(
                ultraShooter.spinUpPhysicsCommand(),
                aimCommand,
                Commands.waitUntil(() -> ultraShooter.isReady() && aimCommand.isAimed())
                    .withTimeout(Constants.ShooterConstants.kShootReadyTimeoutSeconds)
                    .andThen(feedCommand)
            );

            boolean needsBackup = Constants.UltraShooterConstants.kEnableCloseRangeBackup
                && swerve.getDistanceToHub() < Units.inchesToMeters(Constants.UltraShooterConstants.kCloseRangeThresholdInches);

            return needsBackup
                ? Commands.sequence(backupFromHubCommand(), shootSequence)
                : shootSequence;
        }, Set.of(swerve, ultraShooter, feeder, floor));
    }

    /**
     * Drives field-oriented away from the hub until kCloseRangeBackupInches of
     * displacement has been covered. Direction is computed once at schedule time
     * from the current robot pose, so the robot drives in a straight line.
     */
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

            final double speed = Constants.UltraShooterConstants.kCloseRangeBackupSpeedFps * 0.3048;
            return swerve.driveFieldOriented(() -> new ChassisSpeeds(awayX * speed, awayY * speed, 0))
                .until(() -> swerve.getPose().getTranslation().getDistance(startPos) >= backupMeters);
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
        ).withTimeout(7.0);
    }
}
