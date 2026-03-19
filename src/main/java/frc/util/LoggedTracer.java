package frc.util;

import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

/**
 * Lightweight per-subsystem loop-time profiler (adapted from Team 1678).
 *
 * <p>Usage:
 * <ol>
 *   <li>Call {@link #reset()} once at the top of {@code robotPeriodic()},
 *       <em>before</em> {@code CommandScheduler.getInstance().run()}.
 *   <li>Call {@link #record(String)} at the end of each subsystem's
 *       {@code periodic()} method.
 * </ol>
 *
 * <p>Each {@link #record} call logs the elapsed time (ms) since the previous
 * {@link #record} or {@link #reset} call under
 * {@code "LoggedTracer/<subsystemName>_ms"}, giving a per-subsystem view of
 * how much of the 20 ms loop budget each subsystem consumes.
 */
public final class LoggedTracer {

    private static long lastTimestampUs = 0;

    private LoggedTracer() {}

    /** Reset the tracer baseline. Call once at the start of each robot loop. */
    public static void reset() {
        lastTimestampUs = RobotController.getFPGATime();
    }

    /**
     * Record elapsed time since the last {@link #reset()} or {@link #record(String)} call.
     * Publishes to {@code "LoggedTracer/<subsystemName>_ms"}.
     */
    public static void record(String subsystemName) {
        long nowUs = RobotController.getFPGATime();
        Logger.recordOutput("LoggedTracer/" + subsystemName + "_ms", (nowUs - lastTimestampUs) / 1000.0);
        lastTimestampUs = nowUs;
    }
}
