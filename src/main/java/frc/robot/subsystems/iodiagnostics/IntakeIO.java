package frc.robot.subsystems.iodiagnostics;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double pivotAbsEncoderPosition = 0.0;
        public double pivotRelEncoderPosition = 0.0;
        public double pivotCurrentAmps = 0.0;
        public double rollerVelocityRPM = 0.0;
        public double rollerCurrentAmps = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    /** Set the pivot PID position setpoint (absolute encoder units). */
    default void setPivotSetpoint(double position) {}

    /** Drive pivot open-loop. */
    default void setPivotPercentOutput(double percent) {}

    /** Switch pivot idle mode. {@code coast = true} for deploying, {@code false} (brake) for stowed. */
    default void setPivotCoastMode(boolean coast) {}

    /** Override the pivot closed-loop output range. Call with the normal constants to restore after agitation. */
    default void setPivotOutputRange(double min, double max) {}

    /** Set roller closed-loop velocity target (RPM). */
    default void setRollerRPM(double rpm) {}

    /** Drive roller open-loop. */
    default void setRollerPercentOutput(double percent) {}

    /** Cut roller power immediately. */
    default void stopRoller() {}
}
