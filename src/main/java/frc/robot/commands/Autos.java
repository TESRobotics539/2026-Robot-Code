package frc.robot.commands;

/**
 * Auto routines placeholder — add PathPlanner named commands or full autonomous sequences here.
 * See RobotContainer.configureNamedCommands() for currently registered named commands.
 *
 * Archived Choreo-based routines are preserved in comments below for reference.
 */
public final class Autos {
    private Autos() {}
}

// ── Archived: Choreo-based AutoRoutines (pre-PathPlanner) ─────────────────────
// Retained as reference. Requires the Choreo library and generated trajectory classes
// (frc.robot.generated.ChoreoTraj.*) to be re-enabled.
//
// import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$0;
// import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$1;
// import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$2;
// import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$3;
//
// import choreo.auto.AutoChooser;
// import choreo.auto.AutoFactory;
// import choreo.auto.AutoRoutine;
// import choreo.auto.AutoTrajectory;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// import frc.robot.subsystems.robot.Feeder;
// import frc.robot.subsystems.robot.Floor;
// import frc.robot.subsystems.robot.Hanger;
// import frc.robot.subsystems.robot.Intake;
// import frc.robot.subsystems.robot.Swerve;
// import frc.robot.subsystems.vision.Limelight;
//
// public final class Autos {
//     private final Swerve swerve;
//     private final Intake intake;
//     private final Floor floor;
//     private final Feeder feeder;
//     private final Hanger hanger;
//     private final Limelight limelight;
//     private final SubsystemCommands subsystemCommands;
//     private final AutoFactory autoFactory;
//     private final AutoChooser autoChooser;
//
//     public Autos(Swerve swerve, Intake intake, Floor floor, Feeder feeder, Hanger hanger, Limelight limelight) {
//         this.swerve = swerve;
//         this.intake = intake;
//         this.floor = floor;
//         this.feeder = feeder;
//         this.hanger = hanger;
//         this.limelight = limelight;
//         this.subsystemCommands = new SubsystemCommands(swerve, floor, feeder, null);
//         this.autoFactory = swerve.createAutoFactory();
//         this.autoChooser = new AutoChooser();
//     }
//
//     public void configure() {
//         autoChooser.addRoutine("Outpost and Depot", this::outpostAndDepotRoutine);
//         SmartDashboard.putData("Auto Chooser", autoChooser);
//         RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
//     }
//
//     private AutoRoutine outpostAndDepotRoutine() {
//         final AutoRoutine routine = autoFactory.newRoutine("Outpost and Depot");
//         final AutoTrajectory startToOutpost     = OutpostAndDepotTrajectory$0.asAutoTraj(routine);
//         final AutoTrajectory outpostToDepot     = OutpostAndDepotTrajectory$1.asAutoTraj(routine);
//         final AutoTrajectory depotToShootingPose = OutpostAndDepotTrajectory$2.asAutoTraj(routine);
//         final AutoTrajectory shootingPoseToTower = OutpostAndDepotTrajectory$3.asAutoTraj(routine);
//
//         routine.active().onTrue(Commands.sequence(startToOutpost.resetOdometry(), startToOutpost.cmd()));
//         routine.observe(hanger::isHomed).onTrue(
//             Commands.sequence(Commands.waitSeconds(0.5), intake.runOnce(() -> intake.set(Intake.Position.INTAKE)))
//         );
//         startToOutpost.doneDelayed(1).onTrue(outpostToDepot.cmd());
//         outpostToDepot.atTimeBeforeEnd(1).onTrue(intake.intakeCommand());
//         outpostToDepot.doneDelayed(0.1).onTrue(depotToShootingPose.cmd());
//         depotToShootingPose.active().whileTrue(limelight.idle());
//         depotToShootingPose.atTime(0.5).onTrue(Commands.parallel(subsystemCommands.aimAndShoot().withTimeout(5)));
//         depotToShootingPose.done().onTrue(Commands.sequence(subsystemCommands.aimAndShoot().withTimeout(5), shootingPoseToTower.cmd()));
//         shootingPoseToTower.active().whileTrue(limelight.idle());
//         shootingPoseToTower.active().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
//         shootingPoseToTower.done().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
//         return routine;
//     }
// }
