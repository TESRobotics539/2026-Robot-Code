package frc.robot;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.05;
  }

  public static final double maxSpeed = Units.feetToMeters(8); //was 16 before
  
  // public static class Driving {
  //     public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
  //     public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
  // }

  public static final AngularVelocity intakeMaxSpeed = RPM.of(1);
}
