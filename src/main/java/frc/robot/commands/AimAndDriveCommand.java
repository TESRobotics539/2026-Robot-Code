package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.GeometryUtil;
import frc.util.ManualDriveInput;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;
    private Rotation2d targetHeading = new Rotation2d();

    public AimAndDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
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
        final Rotation2d hubDirection = hubPosition.minus(robotPosition).getAngle();
        return hubDirection;
    }

    @Override
    public void initialize() {
        // Enable heading correction for this command
        swerve.getSwerveDrive().setHeadingCorrection(true);
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        
        // Update target heading to point at the hub
        targetHeading = getDirectionToHub();
        
        // Use YAGSL's heading controller
        ChassisSpeeds targetSpeeds = swerve.getTargetSpeeds(
            input.forward,
            input.left,
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
        // Disable heading correction when command ends
        swerve.getSwerveDrive().setHeadingCorrection(false);
    }
}
