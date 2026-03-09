// package frc.robot.commands;

// import static edu.wpi.first.units.Units.Seconds;

// import java.util.Optional;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.units.measure.Time;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.Driving;
// import frc.robot.subsystems.Swerve;
// import frc.util.DriveInputSmoother;
// import frc.util.ManualDriveInput;
// import frc.util.Stopwatch;

// /**
//  * Teleop manual drive command for the swerve drivetrain.
//  *
//  * Handles field-centric driving with manual rotation input and
//  * heading-hold behavior after a short delay once rotation input
//  * returns to zero.
//  */
// public class ManualDriveCommand extends Command {
//     private enum State {
//         IDLING,
//         DRIVING_WITH_MANUAL_ROTATION,
//         DRIVING_WITH_LOCKED_HEADING
//     }

//     private static final Time kHeadingLockDelay = Seconds.of(0.25); // time to wait before locking heading

//     private final Swerve swerve;
//     private final DriveInputSmoother inputSmoother;

//     private State currentState = State.IDLING;
//     private Optional<Rotation2d> lockedHeading = Optional.empty();
//     private Stopwatch headingLockStopwatch = new Stopwatch();
//     private ManualDriveInput previousInput = new ManualDriveInput();

//     public ManualDriveCommand(
//         Swerve swerve,
//         DoubleSupplier forwardInput,
//         DoubleSupplier leftInput,
//         DoubleSupplier rotationInput
//     ) {
//         this.swerve = swerve;
//         this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput, rotationInput);
//         addRequirements(swerve);
//     }

//     public void seedFieldCentric() {
//         initialize();
//         // YAGSL handles field-centric automatically, no special seeding needed
//     }

//     public void setLockedHeading(Rotation2d heading) {
//         lockedHeading = Optional.of(heading);
//         currentState = State.DRIVING_WITH_LOCKED_HEADING;
//     }

//     private void setLockedHeadingToCurrent() {
//         // Use the current robot heading directly
//         setLockedHeading(swerve.getHeading());
//     }

//     private void lockHeadingIfRotationStopped(ManualDriveInput input) {
//         if (input.hasRotation()) {
//             headingLockStopwatch.reset();
//             lockedHeading = Optional.empty();
//         } else {
//             headingLockStopwatch.startIfNotRunning();
//             if (headingLockStopwatch.elapsedTime().gt(kHeadingLockDelay)) {
//                 setLockedHeadingToCurrent();
//             }
//         }
//     }

//     @Override
//     public void initialize() {
//         currentState = State.IDLING;
//         lockedHeading = Optional.empty();
//         headingLockStopwatch.reset();
//         previousInput = new ManualDriveInput();
//         // Enable heading correction for locked heading mode
//         swerve.getSwerveDrive().setHeadingCorrection(true);
//     }

//     @Override
//     public void execute() {
//         final ManualDriveInput input = inputSmoother.getSmoothedInput();
//         if (input.hasRotation()) {
//             currentState = State.DRIVING_WITH_MANUAL_ROTATION;
//         } else if (input.hasTranslation()) {
//             currentState = lockedHeading.isPresent() ? State.DRIVING_WITH_LOCKED_HEADING : State.DRIVING_WITH_MANUAL_ROTATION;
//         } else if (previousInput.hasRotation() || previousInput.hasTranslation()) {
//             currentState = State.IDLING;
//         }
//         previousInput = input;

//         switch (currentState) {
//             case IDLING:
//                 // Stop the robot
//                 swerve.drive(new Translation2d(0, 0), 0, true);
//                 break;
//             case DRIVING_WITH_MANUAL_ROTATION:
//                 lockHeadingIfRotationStopped(input);
//                 // Use YAGSL's drive method with manual rotation
//                 Translation2d translation = new Translation2d(
//                     input.forward * Driving.kMaxSpeed.in(edu.wpi.first.units.Units.MetersPerSecond),
//                     input.left * Driving.kMaxSpeed.in(edu.wpi.first.units.Units.MetersPerSecond)
//                 );
//                 double rotationRate = input.rotation * Driving.kMaxRotationalRate.in(edu.wpi.first.units.Units.RadiansPerSecond);
//                 swerve.drive(translation, rotationRate, true);
//                 break;
//             case DRIVING_WITH_LOCKED_HEADING:
//                 // Use YAGSL's heading controller
//                 ChassisSpeeds targetSpeeds = swerve.getTargetSpeeds(
//                     input.forward,
//                     input.left,
//                     lockedHeading.get()
//                 );
//                 swerve.driveFieldOriented(targetSpeeds);
//                 break;
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         // Default drive command: runs until interrupted
//         return false;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         // Disable heading correction when command ends
//         swerve.getSwerveDrive().setHeadingCorrection(false);
//     }
// }
