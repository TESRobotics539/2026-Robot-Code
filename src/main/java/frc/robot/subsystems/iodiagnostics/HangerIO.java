package frc.robot.subsystems.iodiagnostics;

import org.littletonrobotics.junction.AutoLog;

public interface HangerIO {
    @AutoLog
    public static class HangerIOInputs {
        public double encoderPositionRot = 0.0;
        public double outputCurrentAmps = 0.0;
    }

    default void updateInputs(HangerIOInputs inputs) {}
    default void setPercentOutput(double percent) {}
    default void resetEncoder() {}
}
