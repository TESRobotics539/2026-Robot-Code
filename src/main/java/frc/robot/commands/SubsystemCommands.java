package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
//import frc.robot.subsystems.Hood;
//import frc.robot.subsystems.ShooterOrca; // deprecated — replaced by UltraShooter
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UltraShooter;

public final class SubsystemCommands {
    private final Swerve swerve;
    //private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    //private final ShooterOrca shooter; // deprecated
    private final UltraShooter ultraShooter;
    //private final Hood hood;
    //private final Hanger hanger;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public SubsystemCommands(
        Swerve swerve,
        //Intake intake,
        Floor floor,
        Feeder feeder,
        //ShooterOrca shooter, // deprecated
        UltraShooter ultraShooter,
        //Hood hood,
        //Hanger hanger,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        //this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        //this.shooter = shooter; // deprecated
        this.ultraShooter = ultraShooter;
        //this.hood = hood;
        //this.hanger = hanger;

        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
        Swerve swerve,
        //Intake intake,
        Floor floor,
        Feeder feeder,
        //ShooterOrca shooter, // deprecated
        UltraShooter ultraShooter
        //Hood hood
        //Hanger hanger
    ) {
        this(
            swerve,
            //intake,
            floor,
            feeder,
            //shooter, // deprecated
            ultraShooter,
            //hood,
            //hanger,
            () -> 0,
            () -> 0
        );
    }

    public Command aimAndShoot() {
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, () -> swerve.getPose());
        // deadline() ends the whole group (and cancels aim/prepare) once the feed sequence completes.
        return Commands.deadline(
            Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot())
                .andThen(feed()),
            aimAndDriveCommand,
            Commands.waitSeconds(0.25)
                .andThen(prepareShotCommand)
        );
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
        AimAndDriveCommand aimCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
        return Commands.parallel(
            ultraShooter.spinUpPhysicsCommand(),
            aimCommand,
            Commands.waitUntil(() -> ultraShooter.isReady() && aimCommand.isAimed())
                .withTimeout(Constants.ShooterConstants.kShootReadyTimeoutSeconds)
                .andThen(longFeed())
        ).withTimeout(4.0);
    }

    private Command aimAndFire(Command feedCommand) {
        AimAndDriveCommand aimCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
        return Commands.parallel(
            ultraShooter.spinUpPhysicsCommand(),
            aimCommand,
            Commands.waitUntil(() -> ultraShooter.isReady() && aimCommand.isAimed())
                .withTimeout(Constants.ShooterConstants.kShootReadyTimeoutSeconds)
                .andThen(feedCommand)
        );
    }

    private Command feed() {
        return Commands.parallel(
            feeder.feedCommand(),
            Commands.waitSeconds(Constants.ShooterConstants.kFloorFeedDelaySeconds)
                .andThen(floor.feedCommand())//.alongWith(intake.agitateCommand()))
        );
    }

    private Command longFeed() {
        return Commands.parallel(
            feeder.feedCommand(),
            floor.feedCommand()
        ).withTimeout(7.0);
    }
}
