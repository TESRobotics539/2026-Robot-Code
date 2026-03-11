package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.subsystems.ShooterOrca;

public class PrepareShotCommand extends Command {
    private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) ->
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                Interpolator.forDouble()
                    .interpolate(startValue.shooterSpeed, endValue.shooterSpeed, t),
                Interpolator.forDouble()
                    .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
            )
    );

    static {
        // Shooter speeds in ft/s (converted from RPM with 4" wheel)
        distanceToShotMap.put(Inches.of(52.0),  new Shot(49.0, 0.19)); // 2800 RPM
        distanceToShotMap.put(Inches.of(114.4), new Shot(57.0, 0.40)); // 3275 RPM
        distanceToShotMap.put(Inches.of(165.5), new Shot(64.0, 0.48)); // 3650 RPM
    }

    private final ShooterOrca shooter;
    private final Supplier<Pose2d> robotPoseSupplier;

    public PrepareShotCommand(ShooterOrca shooter, Supplier<Pose2d> robotPoseSupplier) {
        this.shooter = shooter;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(shooter);
    }

    public boolean isReadyToShoot() {
        return shooter.isShooterReady();
    }

    private Distance getDistanceToHub() {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
        final Translation2d hubPosition = Landmarks.hubPosition();
        return Meters.of(robotPosition.getDistance(hubPosition));
    }

    @Override
    public void execute() {
        final Distance distanceToHub = getDistanceToHub();
        final Shot shot = distanceToShotMap.get(distanceToHub);
        shooter.setShooterTarget(shot.shooterSpeed);
        SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    public static class Shot {
        public final double shooterSpeed; // m/s
        public final double hoodPosition;

        public Shot(double shooterSpeed, double hoodPosition) {
            this.shooterSpeed = shooterSpeed;
            this.hoodPosition = hoodPosition;
        }
    }
}
