package frc.robot.subsystems.iodiagnostics;

import org.littletonrobotics.junction.AutoLog;

public interface UltraShooterIO {

    @AutoLog
    class UltraShooterIOInputs {
        /** Left motor surface velocity (ft/s), sign matches shooter direction. */
        public double leftVelocityFPS   = 0.0;
        /** Middle motor surface velocity (ft/s). */
        public double middleVelocityFPS = 0.0;
        /** Right motor surface velocity (ft/s). */
        public double rightVelocityFPS  = 0.0;
        /** Left motor output current (A). */
        public double leftCurrentAmps   = 0.0;
        /** Middle motor output current (A). */
        public double middleCurrentAmps = 0.0;
        /** Right motor output current (A). */
        public double rightCurrentAmps  = 0.0;
    }

    /** Read hardware state into {@code inputs}. Call once per periodic cycle. */
    default void updateInputs(UltraShooterIOInputs inputs) {}

    /**
     * Command all three motors to the given velocity setpoint with explicit
     * feedforward voltage.  The PID remainder is applied on the controller itself.
     *
     * @param velocityFPS      Flywheel surface speed setpoint (ft/s).
     * @param feedforwardVolts Feedforward voltage (V) added to the PID output.
     */
    default void setVelocity(double velocityFPS, double feedforwardVolts) {}

    /** Cut all motor output to zero immediately. */
    default void stop() {}

    /**
     * Reconfigure kP on all three closed-loop controllers.
     * Only call while the robot is disabled to avoid mid-match configuration writes.
     *
     * @param kP New proportional gain.
     */
    default void setKp(double kP) {}
}
