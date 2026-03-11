package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.ShooterTuner;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UltraShooter;

public final class SubsystemCommands {
    private final Swerve swerve;
    private final Floor floor;
    private final Feeder feeder;
    private final UltraShooter ultraShooter;
    private final ShooterTuner shooterTuner;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public SubsystemCommands(
        Swerve swerve,
        Floor floor,
        Feeder feeder,
        UltraShooter ultraShooter,
        ShooterTuner shooterTuner,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.floor = floor;
        this.feeder = feeder;
        this.ultraShooter = ultraShooter;
        this.shooterTuner = shooterTuner;

        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
        Swerve swerve,
        Floor floor,
        Feeder feeder,
        UltraShooter ultraShooter,
        ShooterTuner shooterTuner
    ) {
        this(swerve, floor, feeder, ultraShooter, shooterTuner, () -> 0, () -> 0);
    }

    public Command shootManually() {
        return ultraShooter.spinUpPhysicsCommand()
            .andThen(feed())
            .handleInterrupt(ultraShooter::stop);
    }

    public Command shootMap() {
        return aimAndFire(feed());
    }

    /** Holds flywheel speed and continues aiming at the hub for the given duration, then stops both. */
    public Command holdAimAndSpeedCommand(double seconds) {
        return Commands.parallel(
            ultraShooter.holdSpeedCommand(seconds),
            new AimAndDriveCommand(swerve, forwardInput, leftInput)
        ).withTimeout(seconds);
    }

    public Command autoShoot() {
        return Commands.defer(() -> {
            AimAndDriveCommand aimCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
            return Commands.parallel(
                ultraShooter.spinUpPhysicsCommand(),
                aimCommand,
                Commands.waitUntil(() -> ultraShooter.isReady() && aimCommand.isAimed())
                    .withTimeout(shooterTuner.getShootReadyTimeoutSeconds())
                    .andThen(longFeed())
            ).withTimeout(4.0);
        }, Set.of(swerve, ultraShooter, feeder, floor));
    }

    private Command aimAndFire(Command feedCommand) {
        return Commands.defer(() -> {
            AimAndDriveCommand aimCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
            return Commands.parallel(
                ultraShooter.spinUpPhysicsCommand(),
                aimCommand,
                Commands.waitUntil(() -> ultraShooter.isReady() && aimCommand.isAimed())
                    .withTimeout(shooterTuner.getShootReadyTimeoutSeconds())
                    .andThen(feedCommand)
            );
        }, Set.of(swerve, ultraShooter, feeder, floor));
    }

    private Command feed() {
        return Commands.parallel(
            feeder.feedCommand(),
            Commands.defer(
                () -> Commands.waitSeconds(shooterTuner.getFloorFeedDelaySeconds()),
                Set.of()
            ).andThen(floor.feedCommand())
        );
    }

    private Command longFeed() {
        return Commands.parallel(
            feeder.feedCommand(),
            floor.feedCommand()
        ).withTimeout(7.0);
    }
}
