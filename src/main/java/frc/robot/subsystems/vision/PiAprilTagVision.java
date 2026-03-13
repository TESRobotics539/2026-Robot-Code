package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Reads AprilTag-based robot pose estimates published by {@code apriltag_vision.py}
 * running on the Raspberry Pi and exposes them for use as a vision fallback when
 * the Limelight has no confident data.
 *
 * <h2>Priority</h2>
 * <ol>
 *   <li><b>Limelight (primary)</b> — always preferred when tag count ≥ 1.</li>
 *   <li><b>Pi AprilTag (fallback)</b> — used only when {@link #getMeasurement()}
 *       is called and the Limelight returned {@code Optional.empty()}.</li>
 * </ol>
 *
 * <h2>Standard deviations</h2>
 * Pi measurements carry higher uncertainty than a Limelight (un-calibrated
 * consumer USB camera vs. a dedicated vision coprocessor), so the pose
 * estimator weights them accordingly:
 * <ul>
 *   <li>Single tag  — (0.7, 0.7, 25.0)</li>
 *   <li>Multi-tag   — (0.3, 0.3, 15.0)</li>
 * </ul>
 *
 * <h2>Camera stream</h2>
 * An annotated MJPEG feed (tag outlines + IDs) is served on port 1183:
 * {@code http://10.5.39.2:1183/stream.mjpg}
 */
public class PiAprilTagVision extends SubsystemBase {

    // ── Measurement ───────────────────────────────────────────────────────────

    /** Data bundle returned by {@link #getMeasurement()}. */
    public static class Measurement {
        public final Pose2d           pose;
        public final double           timestampSeconds;
        public final Matrix<N3, N1>   standardDeviations;

        public Measurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDeviations) {
            this.pose               = pose;
            this.timestampSeconds   = timestampSeconds;
            this.standardDeviations = standardDeviations;
        }
    }

    // ── Constants ─────────────────────────────────────────────────────────────

    /** Cycles without heartbeat change → Pi considered disconnected (500 ms). */
    private static final int STALE_THRESHOLD = 25;

    /**
     * Approximate camera + processing pipeline latency subtracted from
     * {@link Timer#getFPGATimestamp()} to produce the measurement timestamp.
     * Increase if the robot overshoots during autos (measurement is too fresh);
     * decrease if it undershoots (measurement is too stale).
     */
    private static final double PIPELINE_LATENCY_SECONDS = 0.10;

    /**
     * Minimum decision margin accepted from the Pi for a measurement to be
     * published.  Mirrors {@code MIN_DECISION_MARGIN} in {@code apriltag_vision.py}.
     * The Pi already filters on this; this is a secondary Java-side gate.
     */
    private static final double MIN_DECISION_MARGIN = 35.0;

    // ── NetworkTables ─────────────────────────────────────────────────────────

    private final NetworkTable      nt         = NetworkTableInstance.getDefault().getTable("PiVision");
    private final NetworkTableEntry ntX        = nt.getEntry("RobotX");
    private final NetworkTableEntry ntY        = nt.getEntry("RobotY");
    private final NetworkTableEntry ntYaw      = nt.getEntry("RobotYaw");
    private final NetworkTableEntry ntTagCount = nt.getEntry("TagCount");
    private final NetworkTableEntry ntMargin   = nt.getEntry("AvgDecisionMargin");
    private final NetworkTableEntry ntHB       = nt.getEntry("Heartbeat");

    // ── Staleness tracking ────────────────────────────────────────────────────

    private long lastHeartbeat = Long.MIN_VALUE;
    private int  staleFrames   = STALE_THRESHOLD;

    // ─────────────────────────────────────────────────────────────────────────

    public PiAprilTagVision() {
        SmartDashboard.putData(this);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Returns a pose measurement if the Pi is connected and has a confident
     * AprilTag estimate.  Returns {@code Optional.empty()} otherwise.
     *
     * <p>Call only when the Limelight has already returned empty — this is
     * the fallback, not the primary source.
     */
    public Optional<Measurement> getMeasurement() {
        if (!isConnected()) return Optional.empty();

        int    tagCount = (int) ntTagCount.getInteger(0);
        double margin   = ntMargin.getDouble(0.0);

        if (tagCount == 0 || margin < MIN_DECISION_MARGIN) return Optional.empty();

        Pose2d pose = new Pose2d(
            ntX.getDouble(0.0),
            ntY.getDouble(0.0),
            Rotation2d.fromDegrees(ntYaw.getDouble(0.0))
        );

        // Higher uncertainty than Limelight; improves with more tags.
        Matrix<N3, N1> stdDevs = tagCount >= 2
            ? VecBuilder.fill(0.3, 0.3, 15.0)
            : VecBuilder.fill(0.7, 0.7, 25.0);

        double timestamp = Timer.getFPGATimestamp() - PIPELINE_LATENCY_SECONDS;

        return Optional.of(new Measurement(pose, timestamp, stdDevs));
    }

    /** True when the Pi heartbeat has advanced within the last 500 ms. */
    public boolean isConnected() {
        return staleFrames < STALE_THRESHOLD;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // SubsystemBase overrides
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        long hb = ntHB.getInteger(Long.MIN_VALUE);
        if (hb != lastHeartbeat) {
            lastHeartbeat = hb;
            staleFrames   = 0;
        } else {
            staleFrames++;
        }
        Logger.recordOutput("PiAprilTagVision/Connected", isConnected());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Pi Connected",          this::isConnected,                                   null);
        builder.addIntegerProperty("Tag Count",             () -> ntTagCount.getInteger(0),                      null);
        builder.addDoubleProperty("Avg Decision Margin",    () -> ntMargin.getDouble(0.0),                       null);
        builder.addDoubleProperty("Robot X (m)",            () -> ntX.getDouble(0.0),                            null);
        builder.addDoubleProperty("Robot Y (m)",            () -> ntY.getDouble(0.0),                            null);
        builder.addDoubleProperty("Robot Yaw (deg)",        () -> ntYaw.getDouble(0.0),                          null);
    }
}
