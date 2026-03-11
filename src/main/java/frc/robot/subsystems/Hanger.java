package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
    }

    private final SparkMax motor;
    private final RelativeEncoder encoder;

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

        // Current is needed at 20ms for stall detection; position for encoder tracking.
        // Velocity is never read on the Rio.
        config.signals
            .primaryEncoderVelocityPeriodMs(500);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
        encoder.setPosition(0);

        SmartDashboard.putData(this);
    }

    public void setPercentOutput(double percentOutput) {
        motor.set(percentOutput);
    }

    public Command toggleCommand() {
        return runOnce(() -> toggleIsUp = !toggleIsUp);
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
            runOnce(() -> setPercentOutput(Constants.HangerConstants.kHomingPower)),
            Commands.waitUntil(() -> motor.getOutputCurrent() > Constants.HangerConstants.kHomingCurrentThreshold),
            runOnce(() -> {
                encoder.setPosition(0);
                isHomed = true;
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public boolean isHomed() {
        return isHomed;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Supply Current", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("Encoder Position", () -> encoder.getPosition(), null);
    }
}
