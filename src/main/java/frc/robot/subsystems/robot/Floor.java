package frc.robot.subsystems.robot;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.iodiagnostics.FloorIO;
import frc.robot.subsystems.iodiagnostics.FloorIOInputsAutoLogged;

public class Floor extends SubsystemBase {
    public enum Speed {
        STOP(0),
        FEED(Constants.FloorConstants.kFeedPercentOutput);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    private final FloorIO io;
    private final FloorIOInputsAutoLogged inputs = new FloorIOInputsAutoLogged();

    public Floor(FloorIO io) {
        this.io = io;
        SmartDashboard.putData(this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Floor", inputs);
    }

    public void set(Speed speed) {
        io.setVoltage(speed.voltage().in(Volts));
    }

    public void setPercentOutput(double percentOutput) {
        io.setVoltage(percentOutput * 12.0);
    }

    public Command feedCommand() {
        return startEnd(() -> set(Speed.FEED), () -> set(Speed.STOP));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("RPM", () -> inputs.velocityRPM, null);
        builder.addDoubleProperty("Output Current", () -> inputs.outputCurrentAmps, null);
    }
}
