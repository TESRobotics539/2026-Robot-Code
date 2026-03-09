package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.FeedbackSensor;
// import com.revrobotics.spark.SparkBase.ControlType;

// import edu.wpi.first.units.AngleUnit;
// import edu.wpi.first.units.DistanceUnit;
// import edu.wpi.first.units.Measure;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.units.measure.Per;
// import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Hanger extends SubsystemBase {
    public enum Position {
        HOMED,
        EXTEND_HOPPER,
        HANGING,
        HUNG;

        // PID position control removed — values kept for reference
        // HOMED(0), EXTEND_HOPPER(2), HANGING(6), HUNG(0.2)
        // public Angle motorAngle() {
        //     final Measure<AngleUnit> angleMeasure = Inches.of(inches).divideRatio(kHangerExtensionPerMotorAngle);
        //     return Rotations.of(angleMeasure.in(Rotations));
        // }
    }

    // private static final Per<DistanceUnit, AngleUnit> kHangerExtensionPerMotorAngle = Inches.of(6).div(Rotations.of(142));
    // private static final Distance kExtensionTolerance = Inches.of(1);

    private final SparkMax motor;
    // private final SparkClosedLoopController pidController;
    private final RelativeEncoder encoder;

    // private static final double kToggleDistanceRotations = Constants.HangerConstants.kToggleDistanceRotations;

    private final Debouncer climbCurrentDebouncer = new Debouncer(Constants.HangerConstants.kAutoClimbCurrentDebounceSeconds, DebounceType.kRising);

    private boolean isHomed = false;
    private boolean toggleIsUp = false;

    // Tracks auto climb encoder position for teleop reversal
    // Zero'd at climb start; records ticks at spike detection or autonomous end
    private static final double kDeclimbReturnThresholdRotations = 10.0;
    private double climbEncoderTicks = 0;
    private boolean autoClimbCompleted = false;

    public Hanger() {
        motor = new SparkMax(Ports.kHanger, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.HangerConstants.kSmartCurrentLimit)
            .secondaryCurrentLimit(Constants.HangerConstants.kSecondaryCurrentLimit);

        // config.closedLoop
        //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //     .pid(0.1, 0, 0)
        //     .velocityFF(0.000175);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
        encoder.setPosition(0);

        // pidController = motor.getClosedLoopController();

        SmartDashboard.putData(this);
    }

    // public void set(Position position) {
    //     targetPositionRotations = position.motorAngle().in(Rotations);
    //     pidController.setSetpoint(targetPositionRotations, ControlType.kPosition);
    // }

    public void setPercentOutput(double percentOutput) {
        motor.set(percentOutput);
    }

    // public Command positionCommand(Position position) {
    //     return runOnce(() -> set(position))
    //         .andThen(Commands.waitUntil(this::isExtensionWithinTolerance));
    // }

    public Command toggleCommand() {
        return runOnce(() -> {
            toggleIsUp = !toggleIsUp;
            // pidController.setSetpoint(targetPositionRotations, ControlType.kPosition);
        });
    }

    public Command autoClimbCommand() {
        return Commands.sequence(
            runOnce(() -> {
                climbCurrentDebouncer.calculate(false); // reset debouncer state
                encoder.setPosition(0);                 // zero encoder at climb start
                autoClimbCompleted = false;
            }),
            run(() -> setPercentOutput(Constants.HangerConstants.kAutoClimbFullPower))
                .until(() -> climbCurrentDebouncer.calculate(motor.getOutputCurrent() > Constants.HangerConstants.kAutoClimbCurrentThreshold)),
            runOnce(() -> {
                climbEncoderTicks = encoder.getPosition(); // record ticks at spike
                autoClimbCompleted = true;
                setPercentOutput(Constants.HangerConstants.kAutoClimbReleasePower);
            }),
            Commands.waitSeconds(Constants.HangerConstants.kAutoClimbReleaseSeconds),
            runOnce(() -> setPercentOutput(0))
        ).finallyDo(interrupted -> {
            // If autonomous ended before the spike was detected, record ticks at interruption
            if (interrupted && !autoClimbCompleted) {
                climbEncoderTicks = encoder.getPosition();
                autoClimbCompleted = true;
                setPercentOutput(0);
            }
        });
    }

    public boolean isAutoClimbCompleted() {
        return autoClimbCompleted;
    }

    /**
     * If the auto climb command ran during autonomous, reverses the climber until the
     * encoder returns to within {@link #kDeclimbReturnThresholdRotations} of zero.
     * Works whether the spike was detected or autonomous ended mid-climb.
     * Clears the flag so it only runs once at the start of teleop.
     */
    public Command reverseClimbIfNeededCommand() {
        return Commands.defer(() -> {
            if (!autoClimbCompleted || climbEncoderTicks == 0) return Commands.none();
            double ticks = climbEncoderTicks;
            autoClimbCompleted = false;
            // signum(ticks) tells us which direction the encoder moved;
            // stop when we've returned within threshold of zero
            return run(() -> setPercentOutput(-Constants.HangerConstants.kAutoClimbFullPower))
                .until(() -> Math.signum(ticks) * encoder.getPosition() <= kDeclimbReturnThresholdRotations)
                .andThen(runOnce(() -> setPercentOutput(0)));
        }, java.util.Set.of(this));
    }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPercentOutput(-0.05)),
            Commands.waitUntil(() -> motor.getOutputCurrent() > 7.0),
            runOnce(() -> {
                encoder.setPosition(0);
                isHomed = true;
                // set(Position.EXTEND_HOPPER);
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public boolean isHomed() {
        return isHomed;
    }

    // private boolean isExtensionWithinTolerance() {
    //     final Distance currentExtension = motorAngleToExtension(Rotations.of(encoder.getPosition()));
    //     final Distance targetExtension = motorAngleToExtension(Rotations.of(targetPositionRotations));
    //     return currentExtension.isNear(targetExtension, kExtensionTolerance);
    // }

    // private Distance motorAngleToExtension(Angle motorAngle) {
    //     final Measure<DistanceUnit> extensionMeasure = motorAngle.timesRatio(kHangerExtensionPerMotorAngle);
    //     return Inches.of(extensionMeasure.in(Inches));
    // }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        // builder.addDoubleProperty("Extension (inches)", () -> motorAngleToExtension(Rotations.of(encoder.getPosition())).in(Inches), null);
        builder.addDoubleProperty("Supply Current", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("Encoder Position", () -> encoder.getPosition(), null);
    }
}
