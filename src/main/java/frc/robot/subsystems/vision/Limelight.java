package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

    // ── Rejection thresholds ──────────────────────────────────────────────────

    /**
     * Measurements older than this threshold are rejected before fusing into the pose estimator.
     * High latency means the image was captured when the robot was in a meaningfully different
     * position, which can corrupt the estimate rather than improve it.
     * (Adapted from frc5687/2025-robot VisionSubsystem: MAX_LATENCY_MS = 100 ms.)
     */
    private static final double MAX_MEASUREMENT_LATENCY_MS = 100.0;

    /**
     * MegaTag2 XY estimates become unreliable above this rotation rate even when heading is
     * provided via {@code SetRobotOrientation}: angular blur during the exposure and the
     * heading-at-capture vs. heading-at-publish gap both corrupt the projection geometry.
     * 270 deg/s is more conservative than Limelight's hard ceiling of 720 deg/s;
     * matches the threshold used by ORCA3136 and provides a wider safety margin.
     */
    private static final double MAX_YAW_RATE_DEG_PER_SEC = 300.0;

    /**
     * Field boundary used to reject geometrically impossible pose estimates (bad solves,
     * tag ambiguity, extreme glare, etc.).
     *
     * <p>Derived from the 2026 REBUILT game manual field drawing (651.2 in × 317.7 in).
     * Verify against the official field CAD or layout drawings before competition.
     * (54 ft 3.2 in × 26 ft 5.7 in)
     *
     * <p>±0.5 m margin is applied so poses near the physical wall boundary are not
     * incorrectly rejected when the robot's center is close to the field edge.
     */
    private static final double FIELD_WIDTH_METERS  = 16.541;
    private static final double FIELD_HEIGHT_METERS =  8.071;
    private static final double FIELD_MARGIN_METERS =  0.5;

    // ── IMU mode constants ────────────────────────────────────────────────────

    /**
     * {@code imumode_set = 1}: Limelight uses ONLY the heading provided by
     * {@code SetRobotOrientation}. Used while enabled so MegaTag2 trusts our Pigeon 2 heading.
     */
    private static final int IMU_MODE_EXTERNAL = 1;

    /**
     * {@code imumode_set = 4}: Limelight syncs its internal IMU to the heading we provide,
     * but keeps an internal IMU fallback. Used while disabled so MT1 can produce a
     * heading-independent pose estimate for odometry seeding without needing us to supply
     * a reliable heading on every cycle.
     */
    private static final int IMU_MODE_SYNC = 4;

    /**
     * Cycles to wait after an IMU mode switch before accepting measurements.
     * ~300 ms at 50 Hz — gives the Limelight time to apply the new mode and stabilise.
     * (Matches ORCA3136/ORCABot2026 VisionSubsystem.kImuSettleCycles.)
     */
    private static final int IMU_SETTLE_CYCLES = 15;

    /** Vision-healthy timeout: if no measurement is accepted for this long, report unhealthy. */
    private static final double VISION_HEALTHY_TIMEOUT_SEC = 0.5;

    // ── Instance state ────────────────────────────────────────────────────────

    private final String name;

    // Cached last IMU reading — updated once per periodic() to avoid redundant NT calls.
    private LimelightHelpers.IMUData lastImu = new LimelightHelpers.IMUData();

    // IMU mode lifecycle tracking
    private boolean wasEnabled         = false;
    private int     imuSettleRemaining = 0;

    // Timestamp of the most recent accepted measurement — used by isVisionHealthy().
    private double lastAcceptedSec = 0.0;

    // Last tag count from MT2 — valid even when getMeasurement() returns empty.
    private int lastTagCount = 0;

    public Limelight(String name) {
        this.name = name;
        // Start in sync mode so the Limelight IMU calibrates from the Pigeon during boot/disabled.
        LimelightHelpers.SetIMUMode(name, IMU_MODE_SYNC);
    }

    @Override
    public void periodic() {
        boolean isEnabled = DriverStation.isEnabled();

        // ── IMU mode transitions ──────────────────────────────────────────────
        if (isEnabled && !wasEnabled) {
            // Disabled → enabled: switch to ExternalImu so MegaTag2 uses only our heading.
            LimelightHelpers.SetIMUMode(name, IMU_MODE_EXTERNAL);
            imuSettleRemaining = IMU_SETTLE_CYCLES;
        } else if (!isEnabled && wasEnabled) {
            // Enabled → disabled: back to SyncInternalImu so MT1 can run heading-independently.
            LimelightHelpers.SetIMUMode(name, IMU_MODE_SYNC);
            imuSettleRemaining = IMU_SETTLE_CYCLES;
        }
        wasEnabled = isEnabled;

        if (imuSettleRemaining > 0) {
            imuSettleRemaining--;
        }

        // ── IMU telemetry ─────────────────────────────────────────────────────
        lastImu = LimelightHelpers.getIMUData(name);
        Logger.recordOutput("Limelight/" + name + "/IMU/Pitch_deg",  lastImu.Pitch);
        Logger.recordOutput("Limelight/" + name + "/IMU/Yaw_deg",    lastImu.Yaw);
        Logger.recordOutput("Limelight/" + name + "/IMU/Roll_deg",   lastImu.Roll);
        Logger.recordOutput("Limelight/" + name + "/IMU/AccelZ_g",   lastImu.accelZ);
        Logger.recordOutput("Limelight/" + name + "/IMU/Settling",   imuSettleRemaining > 0);
        Logger.recordOutput("Limelight/" + name + "/Healthy",        isVisionHealthy());
    }

    /**
     * Fetch a MegaTag2 pose estimate from this Limelight.
     *
     * <p>Pitch/roll angles and all three angular-velocity axes are forwarded to
     * {@code SetRobotOrientation} unconditionally so MegaTag2 always has a fresh heading
     * reference, even when we reject the pose for other reasons (high yaw rate, settling).
     *
     * <p>All angle parameters are in degrees; rate parameters are in degrees per second.
     *
     * @return accepted measurement, or empty if any rejection filter fires
     */
    public Optional<Measurement> getMeasurement(
            Pose2d currentRobotPose,
            double pitchDeg,    double pitchRateDegPerSec,
            double rollDeg,     double rollRateDegPerSec,
            double yawRateDegPerSec) {

        // Always update heading — keeps Limelight's internal state current so the first
        // post-spin (or post-settle) frame has a fresh orientation reference.
        LimelightHelpers.SetRobotOrientation(
            name,
            currentRobotPose.getRotation().getDegrees(), yawRateDegPerSec,
            pitchDeg,  pitchRateDegPerSec,
            rollDeg,   rollRateDegPerSec
        );

        // ── Rejection filter 1: IMU settling ─────────────────────────────────
        if (imuSettleRemaining > 0) {
            Logger.recordOutput("Limelight/" + name + "/RejectReason", "imu_settling");
            return Optional.empty();
        }

        // ── Rejection filter 2: high yaw rate ────────────────────────────────
        if (Math.abs(yawRateDegPerSec) > MAX_YAW_RATE_DEG_PER_SEC) {
            Logger.recordOutput("Limelight/" + name + "/RejectReason", "high_yaw_rate");
            return Optional.empty();
        }

        // ── NT reads (only reached when filters above pass) ───────────────────
        final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        // ── Rejection filter 3: no tags ───────────────────────────────────────
        if (
            poseEstimate_MegaTag1 == null
                || poseEstimate_MegaTag2 == null
                || poseEstimate_MegaTag1.tagCount == 0
                || poseEstimate_MegaTag2.tagCount == 0
        ) {
            lastTagCount = 0;
            Logger.recordOutput("Limelight/" + name + "/RejectReason", "no_tags");
            return Optional.empty();
        }

        lastTagCount = poseEstimate_MegaTag2.tagCount;

        // ── Rejection filter 4: uninitialized NT value ────────────────────────
        // A pose at exactly (0, 0) means the NT entry was never written — no valid solve.
        if (poseEstimate_MegaTag2.pose.getX() == 0.0 && poseEstimate_MegaTag2.pose.getY() == 0.0) {
            Logger.recordOutput("Limelight/" + name + "/RejectReason", "zero_pose");
            return Optional.empty();
        }

        // ── Rejection filter 5: stale frame ───────────────────────────────────
        // A frame captured >100 ms ago depicts the robot at a meaningfully different position.
        // (Adapted from frc5687/2025-robot VisionSubsystem.)
        if (poseEstimate_MegaTag2.latency > MAX_MEASUREMENT_LATENCY_MS) {
            Logger.recordOutput("Limelight/" + name + "/RejectReason", "stale_frame");
            return Optional.empty();
        }

        // ── Hybrid pose: MT2 XY + MT1 rotation ───────────────────────────────
        // MT2 gives stable XY (uses gyro heading to constrain the solve).
        // MT1 gives an independent rotation (no gyro dependency) to counteract gyro drift.
        poseEstimate_MegaTag2.pose = new Pose2d(
            poseEstimate_MegaTag2.pose.getTranslation(),
            poseEstimate_MegaTag1.pose.getRotation()
        );

        // ── Rejection filter 6: field boundary ───────────────────────────────
        // ±FIELD_MARGIN_METERS beyond the true boundary is allowed so poses near the
        // physical wall are not incorrectly rejected when the robot's center is close to the edge.
        double poseX = poseEstimate_MegaTag2.pose.getX();
        double poseY = poseEstimate_MegaTag2.pose.getY();
        if (poseX < -FIELD_MARGIN_METERS || poseX > FIELD_WIDTH_METERS  + FIELD_MARGIN_METERS
         || poseY < -FIELD_MARGIN_METERS || poseY > FIELD_HEIGHT_METERS + FIELD_MARGIN_METERS) {
            Logger.recordOutput("Limelight/" + name + "/RejectReason", "out_of_field");
            return Optional.empty();
        }

        // ── Dynamic XY std devs ───────────────────────────────────────────────
        // Scale trust with how much of the image the tags occupy.
        // avgTagArea is a percentage (0–100); a larger area means the robot is closer to the
        // tags and the projection error is smaller. Rotation stays at 10.0 because MegaTag2
        // relies on the gyro for heading and does not independently constrain rotation.
        // Formula (adapted from frc5687/2025-robot VisionSTDFilter):
        //   xyStdDev = 0.1 / max(0.01, avgTagArea)
        //   → at area 1% (≈10–15 ft): 0.10 m
        //   → at area 5% (≈5–7 ft) : 0.02 m (clamped to 0.03)
        //   → at area 0.1% (≈25 ft): 1.00 m
        // Multi-tag sightings halve the std dev (more geometric constraints).
        double avgTagArea = poseEstimate_MegaTag2.avgTagArea;
        double xyStdDev   = Math.max(0.03, Math.min(3.0, 0.1 / Math.max(0.01, avgTagArea)));
        if (poseEstimate_MegaTag2.tagCount > 1) xyStdDev *= 0.5;
        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(xyStdDev, xyStdDev, 10.0);

        Logger.recordOutput("Limelight/" + name + "/RejectReason",   "");
        Logger.recordOutput("Limelight/" + name + "/EstimatedPose",  poseEstimate_MegaTag2.pose);

        lastAcceptedSec = Timer.getFPGATimestamp();
        return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations, poseEstimate_MegaTag2.avgTagArea));
    }

    /**
     * Returns the raw MegaTag1 pose if ≥2 tags are visible and the pose is within field bounds.
     *
     * <p>MT1 solves heading independently from tag geometry — it does not use the heading
     * supplied via {@code SetRobotOrientation}. This makes it suitable for odometry seeding
     * while disabled, where we want a heading-independent ground-truth pose rather than a
     * Kalman-filter update.
     *
     * <p>Requires tagCount ≥ 2 because a single-tag MT1 solve has high heading ambiguity.
     */
    public Optional<Pose2d> getMT1Pose() {
        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (mt1 == null || mt1.tagCount < 2) return Optional.empty();
        if (mt1.pose.getX() == 0.0 && mt1.pose.getY() == 0.0) return Optional.empty();

        double x = mt1.pose.getX();
        double y = mt1.pose.getY();
        if (x < -FIELD_MARGIN_METERS || x > FIELD_WIDTH_METERS  + FIELD_MARGIN_METERS
         || y < -FIELD_MARGIN_METERS || y > FIELD_HEIGHT_METERS + FIELD_MARGIN_METERS) {
            return Optional.empty();
        }
        return Optional.of(mt1.pose);
    }

    // ── Public accessors ──────────────────────────────────────────────────────

    /**
     * True if at least one measurement was accepted within the last
     * {@value #VISION_HEALTHY_TIMEOUT_SEC} seconds.
     */
    public boolean isVisionHealthy() {
        return (Timer.getFPGATimestamp() - lastAcceptedSec) < VISION_HEALTHY_TIMEOUT_SEC;
    }

    /**
     * Tag count from the most recent MT2 estimate.
     * Returns 0 if no tags were visible or if the last getMeasurement() was rejected before
     * the NT read (e.g., during settle or high yaw rate).
     */
    public int getLastTagCount() {
        return lastTagCount;
    }

    public double getYawStdDev() {
        double[] stddevs = NetworkTableInstance.getDefault().getTable(name)
                          .getEntry("stddevs").getDoubleArray(new double[12]);
        return stddevs[5];
    }

    /** Returns the cached accelerometer Z-axis reading (g) from the last periodic() call.
     *  This value is independent of IMU mode — it reads the physical sensor directly. */
    public double getAccelZ() {
        return lastImu.accelZ;
    }

    public boolean getTV() {
        return LimelightHelpers.getTV(name);
    }

    public double getTX() {
        return LimelightHelpers.getTX(name);
    }

    // ── Inner types ───────────────────────────────────────────────────────────

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;
        public final double avgTagArea;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations, double avgTagArea) {
            this.poseEstimate       = poseEstimate;
            this.standardDeviations = standardDeviations;
            this.avgTagArea         = avgTagArea;
        }
    }
}
