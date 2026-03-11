package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Landmarks {
    /**
     * X-coordinate (meters) where the Blue alliance zone ends and the neutral zone begins.
     * Below this line is the Blue alliance zone; above is neutral or Red territory.
     */
    public static final double kBlueZoneBoundaryX = 5.5;

    /**
     * X-coordinate (meters) where the neutral zone ends and the Red alliance zone begins.
     * Above this line is the Red alliance zone; below is neutral or Blue territory.
     */
    public static final double kRedZoneBoundaryX = 11.0;

    /**
     * Returns true if the robot is in its own alliance zone or the neutral zone —
     * i.e. NOT deep in the opponent's half of the field.
     *
     * <p>Blue pre-spins when x &lt; {@link #kRedZoneBoundaryX} (own zone + neutral).
     * Red pre-spins when x &gt; {@link #kBlueZoneBoundaryX} (own zone + neutral).
     */
    public static boolean isInScoringZone(Pose2d robotPose) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;
        double x = robotPose.getX();
        return switch (alliance.get()) {
            case Blue -> x < kRedZoneBoundaryX;
            case Red  -> x > kBlueZoneBoundaryX;
        };
    }

    public static Translation2d hubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            // Averaged from Landmarks (182.105 in = 4.625 m) and Swerve (4.597 m)
            return new Translation2d(4.611, 4.035);
        }
        // Averaged from Landmarks (469.115 in = 11.916 m) and Swerve (11.938 m)
        return new Translation2d(11.927, 4.035);
    }

    // Add starting positions once you know them from the game manual
    public static Pose2d getStartingPosition(StartingLocation location) {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isBlue = alliance.isPresent() && alliance.get() == Alliance.Blue;
        
        return switch (location) {
            case LEFT -> isBlue 
                ? new Pose2d(Meters.of(1.5), Meters.of(7.0), Rotation2d.fromDegrees(0))
                : new Pose2d(Meters.of(15.0), Meters.of(7.0), Rotation2d.fromDegrees(180));
            
            case CENTER -> isBlue
                ? new Pose2d(Meters.of(1.5), Meters.of(4.1), Rotation2d.fromDegrees(0))
                : new Pose2d(Meters.of(15.0), Meters.of(4.1), Rotation2d.fromDegrees(180));
            
            case RIGHT -> isBlue
                ? new Pose2d(Meters.of(1.5), Meters.of(1.2), Rotation2d.fromDegrees(0))
                : new Pose2d(Meters.of(15.0), Meters.of(1.2), Rotation2d.fromDegrees(180));
        };
    }

    public enum StartingLocation {
        LEFT,
        CENTER,
        RIGHT
    }
}
