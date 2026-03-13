package frc.robot.subsystems.iodiagnostics;

import org.littletonrobotics.junction.AutoLog;

public interface FloorIO {
    @AutoLog
    public static class FloorIOInputs {
        public double velocityRPM = 0.0;
        public double outputCurrentAmps = 0.0;
    }

    default void updateInputs(FloorIOInputs inputs) {}
    default void setVoltage(double volts) {}
}
