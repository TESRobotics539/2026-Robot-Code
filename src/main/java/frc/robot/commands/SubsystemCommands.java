package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.ShooterOrca;
import frc.robot.subsystems.Swerve;

public final class SubsystemCommands {
    private final Swerve swerve;
    //private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final ShooterOrca shooter;
    private final Hood hood;
    //private final Hanger hanger;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public SubsystemCommands(
        Swerve swerve,
        //Intake intake,
        Floor floor,
        Feeder feeder,
        ShooterOrca shooter,
        Hood hood,
        //Hanger hanger,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        //this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        //this.hanger = hanger;

        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
        Swerve swerve,
        //Intake intake,
        Floor floor,
        Feeder feeder,
        ShooterOrca shooter,
        Hood hood
        //Hanger hanger
    ) {
        this(
            swerve,
            //intake,
            floor,
            feeder,
            shooter,
            hood,
            //hanger,
            () -> 0,
            () -> 0
        );
    }

    public Command aimAndShoot() {
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getPose());
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
        return shooter.dashboardSpinUpCommand()
            .andThen(feed())
            .handleInterrupt(() -> shooter.stop());
    }

    public Command shootMap() {
        return aimAndFire(feed());
    }

    public Command autoShoot() {
        return aimAndFire(longFeed());
    }

    private Command aimAndFire(Command feedCommand) {
        AimAndDriveCommand aimCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
        return Commands.parallel(
            shooter.spinUpMapCommand(),
            aimCommand,
            Commands.waitUntil(() -> shooter.isShooterReady() && aimCommand.isAimed())
                .withTimeout(Constants.shootReadyTimeoutSeconds)
                .andThen(Commands.waitSeconds(Constants.shootWaitSeconds))
                .andThen(feedCommand)
        );
    }

    private Command feed() {
        return Commands.parallel(
            feeder.feedCommand(),
            Commands.waitSeconds(Constants.floorFeedDelaySeconds)
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
