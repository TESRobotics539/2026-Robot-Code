package frc.robot.subsystems.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.iodiagnostics.HangerIO;
import frc.robot.subsystems.iodiagnostics.HangerIOInputsAutoLogged;

public class Hanger extends SubsystemBase {
    public enum Position {
        HOMED,
        EXTEND_HOPPER,
        HANGING,
        HUNG;
    }

    private final HangerIO io;
    private final HangerIOInputsAutoLogged inputs = new HangerIOInputsAutoLogged();

    private final Debouncer climbCurrentDebouncer = new Debouncer(Constants.HangerConstants.kAutoClimbCurrentDebounceSeconds, DebounceType.kRising);

    private boolean isHomed = false;
    private boolean toggleIsUp = false;

    // Tracks auto climb encoder position for teleop reversal
    private static final double kDeclimbReturnThresholdRotations = 10.0;
    private double climbEncoderTicks = 0;
    private boolean autoClimbCompleted = false;

    public Hanger(HangerIO io) {
        this.io = io;
        SmartDashboard.putData(this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hanger", inputs);
    }

    public void setPercentOutput(double percentOutput) {
        io.setPercentOutput(percentOutput);
    }

    public Command toggleCommand() {
        return runOnce(() -> toggleIsUp = !toggleIsUp);
    }

    public Command autoClimbCommand() {
        return Commands.sequence(
            runOnce(() -> {
                climbCurrentDebouncer.calculate(false); // reset debouncer state
                io.resetEncoder();                      // zero encoder at climb start
                autoClimbCompleted = false;
            }),
            run(() -> setPercentOutput(Constants.HangerConstants.kAutoClimbFullPower))
                .until(() -> climbCurrentDebouncer.calculate(inputs.outputCurrentAmps > Constants.HangerConstants.kAutoClimbCurrentThreshold)),
            runOnce(() -> {
                climbEncoderTicks = inputs.encoderPositionRot; // record ticks at spike
                autoClimbCompleted = true;
                setPercentOutput(Constants.HangerConstants.kAutoClimbReleasePower);
            }),
            Commands.waitSeconds(Constants.HangerConstants.kAutoClimbReleaseSeconds),
            runOnce(() -> setPercentOutput(0))
        ).finallyDo(interrupted -> {
            // If autonomous ended before the spike was detected, record ticks at interruption
            if (interrupted && !autoClimbCompleted) {
                climbEncoderTicks = inputs.encoderPositionRot;
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
     */
    public Command reverseClimbIfNeededCommand() {
        return Commands.defer(() -> {
            if (!autoClimbCompleted || climbEncoderTicks == 0) return Commands.none();
            double ticks = climbEncoderTicks;
            autoClimbCompleted = false;
            return run(() -> setPercentOutput(-Constants.HangerConstants.kAutoClimbFullPower))
                .until(() -> Math.signum(ticks) * inputs.encoderPositionRot <= kDeclimbReturnThresholdRotations)
                .andThen(runOnce(() -> setPercentOutput(0)));
        }, java.util.Set.of(this));
    }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPercentOutput(Constants.HangerConstants.kHomingPower)),
            Commands.waitUntil(() -> inputs.outputCurrentAmps > Constants.HangerConstants.kHomingCurrentThreshold)
                .withTimeout(10.0),
            runOnce(() -> {
                io.resetEncoder();
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
        builder.addDoubleProperty("Supply Current", () -> inputs.outputCurrentAmps, null);
        builder.addDoubleProperty("Encoder Position", () -> inputs.encoderPositionRot, null);
    }
}
