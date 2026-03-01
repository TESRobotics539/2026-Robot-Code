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
    public static Translation2d hubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(Inches.of(182.105), Inches.of(158.845));
        }
        return new Translation2d(Inches.of(469.115), Inches.of(158.845));
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
