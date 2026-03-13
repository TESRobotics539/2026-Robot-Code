package frc.robot.subsystems.robot;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.iodiagnostics.FeederIO;
import frc.robot.subsystems.iodiagnostics.FeederIOInputsAutoLogged;

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
    }

    public void set(Speed speed) {
        io.setVelocityRPM(speed.angularVelocity().in(RPM));
    }

    public void setPercentOutput(double percentOutput) {
        io.setPercentOutput(percentOutput);
    }

    public Command feedCommand() {
        return startEnd(() -> set(Speed.FEED), () -> setPercentOutput(0));
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
