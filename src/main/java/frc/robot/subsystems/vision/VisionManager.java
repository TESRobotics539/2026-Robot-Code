package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.iodiagnostics.LimelightIO;
import frc.robot.subsystems.iodiagnostics.LimelightIOInputsAutoLogged;
import frc.robot.subsystems.robot.Swerve;
import frc.util.LoggedTracer;

/**
 * Dual-Limelight vision subsystem modeled after ORCA3136/ORCABot2026.
 *
 * <p>All vision processing happens in {@link #periodic()} — no command should ever
 * {@code addRequirements()} on this subsystem, because vision fusion must never be
 * interrupted.
 *
 * <h2>Pipeline per cycle</h2>
 * <ol>
 *   <li>Feed robot heading + angular velocities to both Limelights</li>
 *   <li>Get MegaTag2 pose estimates from both cameras</li>
 *   <li>Run 4-stage rejection filter (stale, field boundary, yaw rate, zero pose)</li>
 *   <li>Calculate dynamic standard deviations based on tag distance and count</li>
 *   <li>Fuse accepted measurements into the swerve drive's Kalman filter</li>
 * </ol>
 *
 * <h2>IMU mode lifecycle</h2>
 * <ul>
 *   <li><b>Disabled:</b> SyncInternalImu — Limelight syncs its IMU to our gyro.
 *       MT1 used to seed odometry with heading-independent pose.</li>
 *   <li><b>Enabled:</b> ExternalImu — Limelight uses ONLY the heading we provide via
 *       SetRobotOrientation. MT2 used for Kalman filter fusion.</li>
 * </ul>
 */
public class VisionManager extends SubsystemBase {

    // Limelight IMU mode values
    private static final int IMU_MODE_EXTERNAL = 1;
    private static final int IMU_MODE_SYNC     = 4;

    private final LimelightIO frontIO;
    private final LimelightIO rearIO;
    private final Swerve drivebase;

    private final LimelightIOInputsAutoLogged frontInputs = new LimelightIOInputsAutoLogged();
    private final LimelightIOInputsAutoLogged rearInputs  = new LimelightIOInputsAutoLogged();

    // IMU mode lifecycle
    private boolean wasEnabled         = false;
    private int     imuSettleRemaining = 0;
    private boolean imuSettled         = true;

    // Vision health tracking
    private double lastAcceptedSec = 0.0;

    // Disabled-period seeding
    private boolean hasEverHadFix      = false;
    private boolean disabledSeedDone   = false;

    // Tag count tracking for telemetry
    private int lastFrontTagCount = 0;
    private int lastRearTagCount  = 0;

    /**
     * @param frontIO   IO implementation for the front-facing Limelight
     * @param rearIO    IO implementation for the rear-facing Limelight
     * @param drivebase drivetrain subsystem (provides current pose and orientation)
     */
    public VisionManager(LimelightIO frontIO, LimelightIO rearIO, Swerve drivebase) {
        this.frontIO   = frontIO;
        this.rearIO    = rearIO;
        this.drivebase = drivebase;
    }

    @Override
    public void periodic() {
        boolean isEnabled = DriverStation.isEnabled();

        // ── IMU mode transitions ────────────────────────────────────────────
        if (isEnabled && !wasEnabled) {
            enterActiveMode();
        } else if (!isEnabled && wasEnabled) {
            enterSeedMode();
        }
        wasEnabled = isEnabled;

        // Count down settle timer
        if (imuSettleRemaining > 0) {
            imuSettleRemaining--;
            if (imuSettleRemaining == 0) {
                imuSettled = true;
            }
        }

        // ── Feed heading to both cameras every frame ────────────────────────
        Pose2d currentPose   = drivebase.getPose();
        double headingDeg    = currentPose.getRotation().getDegrees();
        double yawRateDegPS  = drivebase.getYawRateDegPerSec();
        double pitchDeg      = drivebase.getPitchDegrees();
        double pitchRateDPS  = drivebase.getPitchRateDegPerSec();
        double rollDeg       = drivebase.getRollDegrees();
        double rollRateDPS   = drivebase.getRollRateDegPerSec();

        frontIO.updateInputs(frontInputs, headingDeg, yawRateDegPS, pitchDeg, pitchRateDPS, rollDeg, rollRateDPS);
        Logger.processInputs("Vision/FrontLimelight", frontInputs);

        rearIO.updateInputs(rearInputs, headingDeg, yawRateDegPS, pitchDeg, pitchRateDPS, rollDeg, rollRateDPS);
        Logger.processInputs("Vision/RearLimelight", rearInputs);

        // ── Skip processing while IMU is settling ───────────────────────────
        Logger.recordOutput("Vision/ImuSettled", imuSettled);
        if (!imuSettled) {
            Logger.recordOutput("Vision/ActiveSource", "imu_settling");
            publishHealthTelemetry();
            LoggedTracer.record("VisionManager");
            return;
        }

        // ── Process both cameras ────────────────────────────────────────────
        processCamera(frontInputs, true);
        processCamera(rearInputs, false);

        publishHealthTelemetry();
        LoggedTracer.record("VisionManager");
    }

    /**
     * Core vision pipeline for a single camera.
     * Adapted from ORCA3136/ORCABot2026 VisionSubsystem.processCamera().
     */
    private void processCamera(LimelightIOInputsAutoLogged inputs, boolean isFront) {
        String label = isFront ? "Front" : "Rear";

        // Update tag count tracking
        if (isFront) {
            lastFrontTagCount = inputs.megaTag2TagCount;
        } else {
            lastRearTagCount = inputs.megaTag2TagCount;
        }

        // ── Rejection: no data ──────────────────────────────────────────────
        if (inputs.megaTag2TagCount == 0) {
            Logger.recordOutput("Vision/" + label + "/Accepted", false);
            Logger.recordOutput("Vision/" + label + "/RejectReason", "no_tags");
            return;
        }

        Pose2d visionPose = inputs.megaTag2Pose;
        double now = Timer.getFPGATimestamp();

        // ── 4-stage rejection filter ────────────────────────────────────────
        String rejectReason = getRejectReason(inputs, visionPose, now);

        if (!rejectReason.isEmpty()) {
            Logger.recordOutput("Vision/" + label + "/Accepted", false);
            Logger.recordOutput("Vision/" + label + "/RejectReason", rejectReason);
            Logger.recordOutput("Vision/" + label + "/Pose", visionPose);
            return;
        }

        // ── Calculate dynamic XY std devs (ORCA3136 formula) ────────────────
        // xyStdDev = kXYStdDevBase * distance^2 * (1/tagCount) * singleTagPenalty
        // Theta is 9999 — MegaTag2 borrows heading from the gyro, so feeding
        // theta back into the Kalman filter would create a circular dependency.
        double distanceFactor   = inputs.megaTag2AvgTagDist * inputs.megaTag2AvgTagDist;
        double tagFactor        = 1.0 / Math.max(1, inputs.megaTag2TagCount);
        double singleTagPenalty = (inputs.megaTag2TagCount == 1)
            ? Constants.VisionConstants.kSingleTagPenalty : 1.0;
        double xyStdDev = Math.max(Constants.VisionConstants.kMinXYStdDev,
            Constants.VisionConstants.kXYStdDevBase * distanceFactor * tagFactor * singleTagPenalty);
        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, 9999.0);

        // ── Disabled-period seeding or enabled fusion ───────────────────────
        boolean highConfidence = inputs.megaTag2TagCount >= Constants.VisionConstants.kSeedMinTagCount
            && inputs.megaTag2AvgTagDist < Constants.VisionConstants.kSeedMaxDistM;

        if (!hasEverHadFix || (!DriverStation.isEnabled() && highConfidence && !disabledSeedDone)) {
            // Seed odometry — prefer MT1 (heading-independent) if available
            Pose2d seedPose;
            if (inputs.megaTag1TagCount >= 2
                    && !(inputs.megaTag1Pose.getX() == 0.0 && inputs.megaTag1Pose.getY() == 0.0)) {
                seedPose = inputs.megaTag1Pose;
            } else {
                seedPose = visionPose;
            }

            drivebase.resetOdometry(seedPose);

            if (!hasEverHadFix) {
                Logger.recordOutput("Vision/FirstFix", seedPose);
                hasEverHadFix = true;
            } else {
                Logger.recordOutput("Vision/DisabledSeed", seedPose);
                disabledSeedDone = true;
            }

            // Re-settle IMU after odometry reset
            imuSettleRemaining = Constants.VisionConstants.kImuSettleCycles;
            imuSettled = false;
        } else {
            // Normal enabled fusion: Kalman filter with dynamic std devs
            double ts = inputs.megaTag2TimestampSeconds;
            if (ts > 0 && ts <= Timer.getFPGATimestamp()) {
                drivebase.addVisionMeasurement(visionPose, ts, stdDevs);
            }
        }

        lastAcceptedSec = now;

        // ── Telemetry ───────────────────────────────────────────────────────
        Logger.recordOutput("Vision/" + label + "/Accepted", true);
        Logger.recordOutput("Vision/" + label + "/RejectReason", "");
        Logger.recordOutput("Vision/" + label + "/Pose", visionPose);
        Logger.recordOutput("Vision/" + label + "/XYStdDev", xyStdDev);
        Logger.recordOutput("Vision/" + label + "/TagCount", inputs.megaTag2TagCount);
        Logger.recordOutput("Vision/" + label + "/AvgTagDist", inputs.megaTag2AvgTagDist);
    }

    /**
     * 4-stage rejection filter. Returns empty string if accepted.
     * Adapted from ORCA3136/ORCABot2026 VisionSubsystem.getRejectReason().
     */
    private String getRejectReason(LimelightIOInputsAutoLogged inputs, Pose2d visionPose, double now) {
        // 1. Zero pose — uninitialized NT value, no valid solve
        if (visionPose.getX() == 0.0 && visionPose.getY() == 0.0) {
            return "zero_pose";
        }

        // 2. Stale timestamp
        double age = now - inputs.megaTag2TimestampSeconds;
        if (age > Constants.VisionConstants.kMaxTimestampAgeSec || age < 0) {
            return "stale_timestamp";
        }

        // 3. Field boundary
        double x = visionPose.getX();
        double y = visionPose.getY();
        if (x < Constants.VisionConstants.kFieldMinX || x > Constants.VisionConstants.kFieldMaxX
                || y < Constants.VisionConstants.kFieldMinY || y > Constants.VisionConstants.kFieldMaxY) {
            return "out_of_field";
        }

        // 4. High yaw rate — MegaTag2 is unreliable during fast rotation
        double yawRate = Math.abs(drivebase.getYawRateDegPerSec());
        if (yawRate > Constants.VisionConstants.kMaxYawRateDegPerSec) {
            return "high_yaw_rate";
        }

        return "";
    }

    // ── IMU mode transitions ────────────────────────────────────────────────

    private void enterActiveMode() {
        frontIO.setImuMode(IMU_MODE_EXTERNAL);
        rearIO.setImuMode(IMU_MODE_EXTERNAL);
        imuSettleRemaining = Constants.VisionConstants.kImuSettleCycles;
        imuSettled = false;
    }

    private void enterSeedMode() {
        frontIO.setImuMode(IMU_MODE_SYNC);
        rearIO.setImuMode(IMU_MODE_SYNC);
        imuSettleRemaining = Constants.VisionConstants.kImuSettleCycles;
        imuSettled = false;
        disabledSeedDone = false;
    }

    // ── Public accessors ────────────────────────────────────────────────────

    /** True if at least one camera accepted a measurement recently. */
    public boolean isVisionHealthy() {
        return (Timer.getFPGATimestamp() - lastAcceptedSec) < Constants.VisionConstants.kVisionHealthyTimeoutSec;
    }

    /** Combined tag count from both cameras (most recent cycle). */
    public int getTotalTagCount() {
        return lastFrontTagCount + lastRearTagCount;
    }

    /** True if vision has ever successfully seeded or fused a measurement. */
    public boolean hasEverHadFix() {
        return hasEverHadFix;
    }

    // ── Telemetry helpers ───────────────────────────────────────────────────

    private void publishHealthTelemetry() {
        Logger.recordOutput("Vision/Healthy", isVisionHealthy());
        Logger.recordOutput("Vision/TotalTagCount", getTotalTagCount());
        Logger.recordOutput("Vision/HasEverHadFix", hasEverHadFix);
    }
}
