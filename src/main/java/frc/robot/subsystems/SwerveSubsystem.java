package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.encoders.CANCoderSwerve;
import swervelib.encoders.SwerveAbsoluteEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;

import static edu.wpi.first.units.Units.Meter;


public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive  swerveDrive;
  
  private final SwerveAbsoluteEncoder absoluteEncoder_fl;
  private final CANcoder cancoder_fl;
  
  private final SwerveAbsoluteEncoder absoluteEncoder_fr;
  private final CANcoder cancoder_fr;
  
  private final SwerveAbsoluteEncoder absoluteEncoder_bl;
  private final CANcoder cancoder_bl;
  
  private final SwerveAbsoluteEncoder absoluteEncoder_br;
  private final CANcoder cancoder_br;

  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed, new Pose2d(new Translation2d(Meter.of(1), 
                                                                                                                    Meter.of(4)), 
                                                                                                                    Rotation2d.fromDegrees(0)));
                                                                                                                    
      cancoder_fl = new CANcoder(9);
      absoluteEncoder_fl = new CANCoderSwerve(9);
      cancoder_fr = new CANcoder(12);
      absoluteEncoder_fr = new CANCoderSwerve(12);
      cancoder_bl = new CANcoder(6);
      absoluteEncoder_bl = new CANCoderSwerve(6);
      cancoder_br = new CANcoder(3);
      absoluteEncoder_br = new CANCoderSwerve(3);
      

      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTableInstance.getDefault().getTable("AbsoluteEncoders").getEntry("CANcoder 9 position").setNumber(cancoder_fl.getAbsolutePosition().getValueAsDouble());
    NetworkTableInstance.getDefault().getTable("AbsoluteEncoders").getEntry("CANcoder 9 units").setString(cancoder_fl.getAbsolutePosition().getUnits());
    NetworkTableInstance.getDefault().getTable("AbsoluteEncoders").getEntry("Absolute Encoder 9 position").setNumber(absoluteEncoder_fl.getAbsolutePosition());

    NetworkTableInstance.getDefault().getTable("AbsoluteEncoders").getEntry("CANcoder 12 position").setNumber(cancoder_fr.getAbsolutePosition().getValueAsDouble());
    NetworkTableInstance.getDefault().getTable("AbsoluteEncoders").getEntry("CANcoder 12 units").setString(cancoder_fr.getAbsolutePosition().getUnits());
    NetworkTableInstance.getDefault().getTable("AbsoluteEncoders").getEntry("Absolute Encoder 12 position").setNumber(absoluteEncoder_fr.getAbsolutePosition());

    NetworkTableInstance.getDefault().getTable("AbsoluteEncoders").getEntry("CANcoder 6 position").setNumber(cancoder_bl.getAbsolutePosition().getValueAsDouble());
    NetworkTableInstance.getDefault().getTable("AbsoluteEncoders").getEntry("CANcoder 6 units").setString(cancoder_bl.getAbsolutePosition().getUnits());
    NetworkTableInstance.getDefault().getTable("AbsoluteEncoders").getEntry("Absolute Encoder 6 position").setNumber(absoluteEncoder_bl.getAbsolutePosition());

    NetworkTableInstance.getDefault().getTable("AbsoluteEncoders").getEntry("CANcoder 3 position").setNumber(cancoder_br.getAbsolutePosition().getValueAsDouble());
    NetworkTableInstance.getDefault().getTable("AbsoluteEncoders").getEntry("CANcoder 3 units").setString(cancoder_br.getAbsolutePosition().getUnits());
    NetworkTableInstance.getDefault().getTable("AbsoluteEncoders").getEntry("Absolute Encoder 3 position").setNumber(absoluteEncoder_br.getAbsolutePosition());
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void driveFieldOrientated(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());;
    });
  }
}
