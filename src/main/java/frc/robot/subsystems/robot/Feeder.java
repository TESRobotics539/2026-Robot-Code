package frc.robot.subsystems.robot;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.iodiagnostics.FeederIO;
import frc.robot.subsystems.iodiagnostics.FeederIOInputsAutoLogged;
import frc.util.LoggedTracer;

public class Feeder extends SubsystemBase {
    public enum Speed {
        FEED(Constants.FeederConstants.kFeedRPM),
        REVERSE(Constants.FeederConstants.kReverseRPM);

        private final double rpm;

        private Speed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity angularVelocity() {
            return RPM.of(rpm);
        }
    }

    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    public Feeder(FeederIO io) {
        this.io = io;
        SmartDashboard.putData(this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
        LoggedTracer.record("Feeder");
    }

    public void set(Speed speed) {
        io.setVelocityRPM(speed.angularVelocity().in(RPM));
    }

    public void setPercentOutput(double percentOutput) {
        io.setPercentOutput(percentOutput);
    }

    /**
     * Feeds at {@link Speed#FEED} with automatic anti-jam.
     *
     * <p>State machine:
     * <ol>
     *   <li>FEEDING — runs at feed speed; transitions to REVERSING on current spike + low velocity.
     *   <li>REVERSING — runs at {@code kUnjamReverseRPM} for {@code kUnjamReverseSeconds}, then transitions to RECOVERING.
     *   <li>RECOVERING — re-applies feed speed; returns to FEEDING once velocity exceeds {@code kFreeVelocityThresholdRPM}, or repeats the reverse pulse if still jammed.
     * </ol>
     *
     * <p>Never finishes on its own; interrupt to stop.
     */
    public Command feedCommand() {
        final int FEEDING = 0, REVERSING = 1, RECOVERING = 2;
        final int[] state = {FEEDING};
        final Timer unjamTimer = new Timer();

        return runOnce(() -> {
            setPercentOutput(0.9);
            state[0] = FEEDING;
        }).andThen(run(() -> {
            boolean jammed = inputs.outputCurrentAmps > Constants.FeederConstants.kJamCurrentThreshold
                && Math.abs(inputs.velocityRPM) < Constants.FeederConstants.kJamVelocityThresholdRPM;
            boolean free = inputs.velocityRPM > Constants.FeederConstants.kFreeVelocityThresholdRPM;

            Logger.recordOutput("Feeder/UnjamState",
                state[0] == FEEDING ? "FEEDING" : state[0] == REVERSING ? "REVERSING" : "RECOVERING");

            if (state[0] == FEEDING) {
                setPercentOutput(0.9);
                if (jammed) {
                    setPercentOutput(-0.9);
                    unjamTimer.restart();
                    state[0] = REVERSING;
                }
            } else if (state[0] == REVERSING) {
                if (unjamTimer.hasElapsed(Constants.FeederConstants.kUnjamReverseSeconds)) {
                    setPercentOutput(0.9);
                    state[0] = RECOVERING;
                }
            } else { // RECOVERING
                if (free) {
                    state[0] = FEEDING;
                } else if (jammed) {
                    setPercentOutput(-0.9);
                    unjamTimer.restart();
                    state[0] = REVERSING;
                }
            }
        })).finallyDo(interrupted -> setPercentOutput(0));
    }

    public Command reverseCommand() {
        return startEnd(() -> set(Speed.REVERSE), () -> setPercentOutput(0));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("RPM", () -> inputs.velocityRPM, null);
        builder.addDoubleProperty("Output Current", () -> inputs.outputCurrentAmps, null);
    }
}
