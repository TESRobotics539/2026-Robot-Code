package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.subsystems.robot.Swerve;
import frc.robot.subsystems.robot.UltraShooter;
import frc.util.GeometryUtil;

/**
 * Continuously turns the robot to face the hub while allowing translational driving.
 *
 * <p><b>Never finishes on its own</b> — always wrap with {@code .withTimeout()} or compose
 * inside a {@code Commands.parallel()} that has a terminating deadline. An unguarded
 * {@code AimAndDriveCommand} will stall an autonomous sequence indefinitely.
 */
public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(3);

    private final Swerve swerve;
    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;
    private Rotation2d targetHeading = new Rotation2d();

    public AimAndDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
        addRequirements(swerve);
    }

    public AimAndDriveCommand(Swerve swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        final Rotation2d currentHeading = swerve.getHeading();
        return GeometryUtil.isNear(targetHeading, currentHeading, kAimTolerance);
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getPose().getTranslation();

        // Lead the aim point by the ball's time of flight so the hub moves
        // into the ball's path while the robot is in motion.
        final double tof = UltraShooter.calculateTimeOfFlightSeconds(swerve.getDistanceToHub());
        final ChassisSpeeds fieldVelocity = swerve.getFieldVelocity();
        final Translation2d leadOffset = new Translation2d(
                fieldVelocity.vxMetersPerSecond * tof,
                fieldVelocity.vyMetersPerSecond * tof);
        final Translation2d virtualHub = hubPosition.minus(leadOffset);

        return virtualHub.minus(robotPosition).getAngle();
    }

    @Override
    public void initialize() {
        targetHeading = getDirectionToHub();
        swerve.setHeadingCorrection(true);
    }

    @Override
    public void execute() {
        // Update target heading to point at the hub
        targetHeading = getDirectionToHub();

        // Console log for debugging
        // System.out.println("Target Heading: " + targetHeading.getDegrees() + " degrees");
        // System.out.println("Current Heading (gyro): " + swerve.getHeading().getDegrees() + " degrees");
        // System.out.println("Pose Rotation (theta): " + swerve.getPose().getRotation().getDegrees() + " degrees");
        // System.out.println("Difference: " + (swerve.getPose().getRotation().getDegrees() - swerve.getHeading().getDegrees()) + " degrees");
        // System.out.println("Robot Position: " + swerve.getPose().getTranslation());
        // System.out.println("Hub Position: " + Landmarks.hubPosition());
    
    
        // Use YAGSL's heading controller with raw inputs
        ChassisSpeeds targetSpeeds = swerve.getTargetSpeeds(
            forwardInput.getAsDouble(),
            leftInput.getAsDouble(),
            targetHeading
        );
        swerve.driveFieldOriented(targetSpeeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setHeadingCorrection(false);
    }
}