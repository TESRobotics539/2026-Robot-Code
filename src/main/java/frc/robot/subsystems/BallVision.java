package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Reads ball-detection results published by {@code ball_detector.py} running on
 * the Raspberry Pi co-processor and exposes them to the rest of the robot code.
 *
 * <p>The Pi streams an annotated MJPEG feed on port 1182 — add
 * {@code http://10.5.39.2:1182/stream.mjpg} as a camera widget in Shuffleboard
 * or Glass to verify detection quality during testing.
 *
 * <p>All getters return safe defaults (0 count, 0 angle, 0 distance) when the
 * Pi is not connected.
 */
public class BallVision extends SubsystemBase {

    /** Cycles without a heartbeat change before we consider the Pi disconnected. */
    private static final int STALE_THRESHOLD = 25; // 500 ms at 50 Hz

    private final NetworkTable        nt           = NetworkTableInstance.getDefault().getTable("BallDetection");
    private final NetworkTableEntry   ntCount      = nt.getEntry("Count");
    private final NetworkTableEntry   ntAngle      = nt.getEntry("NearestAngle");
    private final NetworkTableEntry   ntDistance   = nt.getEntry("NearestDistance");
    private final NetworkTableEntry   ntHeartbeat  = nt.getEntry("Heartbeat");

    // Telemetry published back to the dashboard
    private final NetworkTableEntry   ntConnected  = nt.getEntry("Pi Connected");

    private long lastHeartbeat  = Long.MIN_VALUE;
    private int  staleFrames    = 0;

    public BallVision() {
        SmartDashboard.putData(this);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API
    // ─────────────────────────────────────────────────────────────────────────

    /** Number of fuel balls currently visible to the Pi camera. */
    public int getBallCount() {
        return isConnected() ? (int) ntCount.getInteger(0) : 0;
    }

    /**
     * Horizontal angle (degrees) from the camera centre to the nearest ball.
     * Negative = ball is to the left, positive = to the right.
     * Returns 0 when no balls are visible or the Pi is disconnected.
     */
    public double getNearestBallAngleDegrees() {
        return isConnected() ? ntAngle.getDouble(0.0) : 0.0;
    }

    /**
     * Estimated distance to the nearest ball (inches), based on apparent size.
     * Returns 0 when no balls are visible or the Pi is disconnected.
     */
    public double getNearestBallDistanceInches() {
        return isConnected() ? ntDistance.getDouble(0.0) : 0.0;
    }

    /** True when at least one ball is visible and the Pi is connected. */
    public boolean hasBallsVisible() {
        return isConnected() && getBallCount() > 0;
    }

    /**
     * True when the Pi heartbeat has advanced within the last 500 ms.
     * Use this to gate any logic that depends on live camera data.
     */
    public boolean isConnected() {
        return staleFrames < STALE_THRESHOLD;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Periodic
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        long hb = ntHeartbeat.getInteger(Long.MIN_VALUE);
        if (hb != lastHeartbeat) {
            lastHeartbeat = hb;
            staleFrames   = 0;
        } else {
            staleFrames++;
        }
        ntConnected.setBoolean(isConnected());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Connected",             this::isConnected,                  null);
        builder.addIntegerProperty("Ball Count",            this::getBallCount,                 null);
        builder.addDoubleProperty("Nearest Angle (deg)",    this::getNearestBallAngleDegrees,   null);
        builder.addDoubleProperty("Nearest Distance (in)",  this::getNearestBallDistanceInches, null);
    }
}
