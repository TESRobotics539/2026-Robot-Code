package frc.robot.subsystems.iodiagnostics;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public double velocityRPM = 0.0;
        public double outputCurrentAmps = 0.0;
    }

    default void updateInputs(FeederIOInputs inputs) {}
    default void setVelocityRPM(double rpm) {}
    default void setPercentOutput(double percent) {}
}
