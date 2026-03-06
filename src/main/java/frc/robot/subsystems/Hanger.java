package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Hanger extends SubsystemBase {
    public enum Position {
        HOMED(0),
        EXTEND_HOPPER(2),
        HANGING(6),
        HUNG(0.2);

        private final double inches;

        private Position(double inches) {
            this.inches = inches;
        }

        public Angle motorAngle() {
            final Measure<AngleUnit> angleMeasure = Inches.of(inches).divideRatio(kHangerExtensionPerMotorAngle);
            return Rotations.of(angleMeasure.in(Rotations)); // Promote from Measure<AngleUnit> to Angle
        }
    }

    private static final double kNeoFreeSpeedRPS = 5676.0 / 60.0; // NEO free speed in rotations per second
    private static final Per<DistanceUnit, AngleUnit> kHangerExtensionPerMotorAngle = Inches.of(6).div(Rotations.of(142));
    private static final Distance kExtensionTolerance = Inches.of(1);

    private final SparkMax motor;
    private final SparkClosedLoopController pidController;
    private final RelativeEncoder encoder;

    private boolean isHomed = false;
    private double targetPositionRotations = 0;

    public Hanger() {
        motor = new SparkMax(Ports.kHanger, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(70)  //Was 20
            .secondaryCurrentLimit(120);
        
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.1, 0, 0)
            .velocityFF(0.000175)
            //.velocityFF(1.0 / kNeoFreeSpeedRPS)
            .maxMotion
                .maxVelocity(kNeoFreeSpeedRPS * 60) // RPM
                .maxAcceleration(kNeoFreeSpeedRPS * 60) // RPM/s
                .allowedClosedLoopError(0.1); // Rotations
        
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        encoder = motor.getEncoder();
        pidController = motor.getClosedLoopController();
        
        SmartDashboard.putData(this);
    }

    public void set(Position position) {
        targetPositionRotations = position.motorAngle().in(Rotations);
        pidController.setSetpoint(
            targetPositionRotations,
            ControlType.kMAXMotionPositionControl
        );
    }

    public void setPercentOutput(double percentOutput) {
        motor.set(percentOutput);
    }

    public Command positionCommand(Position position) {
        return runOnce(() -> set(position))
            .andThen(Commands.waitUntil(this::isExtensionWithinTolerance));
    }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPercentOutput(-0.05)),
            Commands.waitUntil(() -> motor.getOutputCurrent() > 7.0),
            runOnce(() -> {
                encoder.setPosition(Position.HOMED.motorAngle().in(Rotations));
                isHomed = true;
                set(Position.EXTEND_HOPPER);
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public boolean isHomed() {
        return isHomed;
    }

    private boolean isExtensionWithinTolerance() {
        final Distance currentExtension = motorAngleToExtension(Rotations.of(encoder.getPosition()));
        final Distance targetExtension = motorAngleToExtension(Rotations.of(targetPositionRotations));
        return currentExtension.isNear(targetExtension, kExtensionTolerance);
    }

    private Distance motorAngleToExtension(Angle motorAngle) {
        final Measure<DistanceUnit> extensionMeasure = motorAngle.timesRatio(kHangerExtensionPerMotorAngle);
        return Inches.of(extensionMeasure.in(Inches)); // Promote from Measure<DistanceUnit> to Distance
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Extension (inches)", () -> motorAngleToExtension(Rotations.of(encoder.getPosition())).in(Inches), null);
        builder.addDoubleProperty("Supply Current", () -> motor.getOutputCurrent(), null);
    }
}
