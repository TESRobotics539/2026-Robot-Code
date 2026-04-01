package frc.robot.subsystems.vision;

import java.util.Optional;

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
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.iodiagnostics.LimelightIO;
import frc.robot.subsystems.iodiagnostics.LimelightIOInputsAutoLogged;
import frc.robot.subsystems.robot.Swerve;
import frc.util.LoggedTracer;

/**
 * Owns both Limelights, fuses their pose estimates, and feeds the best
 * measurement into the drivetrain pose estimator each loop.
 *
 * <p>Rejection pipeline (adapted from ORCA3136/ORCABot2026 VisionSubsystem):
 * <ol>
 *   <li>Stale timestamp — reject if {@code now - timestamp > 0.5 s}
 *   <li>Field boundary — reject if pose falls outside field + 0.5 m margin
 *   <li>High yaw rate — reject if |yawRate| > 270 deg/s (MT2 heading lag)
 *   <li>Tag count — reject if MT2 sees zero tags
 * </ol>
 *
 * <p>Standard deviations use a distance-squared / tag-count formula (ORCA3136)
 * rather than tag-area, which is more physically meaningful. Rotation stddev is
 * fixed at 9999 so vision never overrides the gyro heading.
 *
 * <p>Odometry seeding: on the first valid MT2 fix with ≥ 2 tags within
 * {@link VisionConstants#kSeedMaxDistM}, the pose estimator is reset to the
 * vision estimate. This only runs once and only during the disabled period to
 * avoid mid-match jumps.
 *
 * <p>All hardware reads go through {@link LimelightIO#updateInputs} and are
 * logged via {@code Logger.processInputs()} so AdvantageKit can replay every
 * fusion decision exactly as it happened on the robot.
 */
public class VisionManager extends SubsystemBase {

    private final LimelightIO frontIO;
    private final LimelightIO rearIO;
    private final Swerve drivebase;

    private final LimelightIOInputsAutoLogged frontInputs = new LimelightIOInputsAutoLogged();
    private final LimelightIOInputsAutoLogged rearInputs  = new LimelightIOInputsAutoLogged();

    /** True once we have successfully seeded odometry from a high-quality vision fix. */
    private boolean hasAppliedFirstFix = false;

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

    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Current robot orientation forwarded to both Limelights so MegaTag2 can
        // compensate for camera tilt and dynamic yaw rotation.
        Pose2d currentPose  = drivebase.getPose();
        double headingDeg   = currentPose.getRotation().getDegrees();
        double yawRateDegPS = drivebase.getYawRateDegPerSec();
        double pitchDeg     = drivebase.getPitchDegrees();
        double rollDeg      = drivebase.getRollDegrees();

        // Read all hardware data (SetRobotOrientation + NT reads) for both cameras.
        frontIO.updateInputs(frontInputs, headingDeg, yawRateDegPS, pitchDeg, rollDeg);
        Logger.processInputs("Vision/FrontLimelight", frontInputs);

        rearIO.updateInputs(rearInputs, headingDeg, yawRateDegPS, pitchDeg, rollDeg);
        Logger.processInputs("Vision/RearLimelight", rearInputs);

        // ── Bump detection ────────────────────────────────────────────────────
        // Pigeon 2 only — Limelight IMU data is unreliable in mode 0 (IMU disabled).
        // TODO: re-add Limelight accelerometer voting once IMU mode is tuned.
        boolean overBump  = drivebase.isOverBump();
        int     bumpVotes = overBump ? 1 : 0;

        // ── Pose estimate extraction ──────────────────────────────────────────
        Optional<MeasurementResult> frontMeasurement = extractMeasurement(frontInputs, yawRateDegPS);
        Optional<MeasurementResult> rearMeasurement  = extractMeasurement(rearInputs, yawRateDegPS);

        // Confidence = avgTagArea × tagCount. Larger area = closer robot = less
        // projection error; more tags = less geometric ambiguity.
        double frontConf = frontMeasurement.map(m -> m.avgTagArea * m.tagCount).orElse(0.0);
        double rearConf  = rearMeasurement.map(m -> m.avgTagArea  * m.tagCount).orElse(0.0);
        double bestConf  = Math.max(frontConf, rearConf);

        // ── Camera selection ──────────────────────────────────────────────────
        Optional<MeasurementResult> bestMeasurement;
        String activeSource;
        if (frontMeasurement.isEmpty() && rearMeasurement.isEmpty()) {
            bestMeasurement = Optional.empty();
            activeSource    = "none";
        } else if (frontConf >= rearConf && frontMeasurement.isPresent()) {
            bestMeasurement = frontMeasurement;
            activeSource    = "front (" + String.format("%.3f", frontConf) + ")";
        } else {
            bestMeasurement = rearMeasurement;
            activeSource    = "rear ("  + String.format("%.3f", rearConf)  + ")";
        }

        // ── Wheel-slip detection ──────────────────────────────────────────────
        boolean isSlipping = drivebase.isWheelSlipping();
        boolean highConf   = bestConf >= Constants.BumpDetectionConstants.kHighConfidenceThreshold;

        // ── Odometry seeding (disabled period only, one-shot) ─────────────────
        // Try front camera first, then rear. Seeds when we have a high-quality fix
        // (≥2 tags, close range) but have not yet corrected the initial odometry pose.
        if (!hasAppliedFirstFix && DriverStation.isDisabled()) {
            maybeSeedOdometry(frontInputs);
            if (!hasAppliedFirstFix) {
                maybeSeedOdometry(rearInputs);
            }
        }

        // ── Logging ───────────────────────────────────────────────────────────
        Logger.recordOutput("Vision/ActiveSource",      activeSource);
        Logger.recordOutput("Vision/FrontConfidence",   frontConf);
        Logger.recordOutput("Vision/RearConfidence",    rearConf);
        Logger.recordOutput("Vision/BestConfidence",    bestConf);
        Logger.recordOutput("Vision/OverBump",          overBump);
        Logger.recordOutput("Vision/BumpVotes",         bumpVotes);
        Logger.recordOutput("Vision/WheelSlipping",     isSlipping);
        Logger.recordOutput("Vision/WheelSlipScore",    drivebase.getWheelSlipScore());
        Logger.recordOutput("Vision/HasFirstFix",       hasAppliedFirstFix);
        Logger.recordOutput("Vision/YawRateDegPerSec",  yawRateDegPS);

        // ── Pose estimator update ─────────────────────────────────────────────
        if (bestMeasurement.isPresent()) {
            var m = bestMeasurement.get();

            // Stddev multiplier priority (highest wins):
            //   Bump + high conf  → camera shaking but solid tag fix; mild inflation
            //   Bump + low conf   → shaking + no reliable fix; heavy inflation
            //   Slip + low conf   → odometry and vision both unreliable; heavy inflation
            //   Otherwise         → pass stddevs unchanged
            double stdDevMultiplier;
            if (overBump && highConf) {
                stdDevMultiplier = Constants.BumpDetectionConstants.kBumpHighConfStdDevMultiplier;
            } else if (overBump) {
                stdDevMultiplier = Constants.BumpDetectionConstants.kBumpVisionStdDevMultiplier;
            } else if (isSlipping && !highConf) {
                stdDevMultiplier = Constants.BumpDetectionConstants.kBumpVisionStdDevMultiplier;
            } else {
                stdDevMultiplier = 1.0;
            }
            Logger.recordOutput("Vision/StdDevMultiplier",   stdDevMultiplier);
            Logger.recordOutput("Vision/AcceptedPose",        m.pose);
            Logger.recordOutput("Vision/AcceptedXYStdDev",    m.standardDeviations.get(0, 0));
            Logger.recordOutput("Vision/AcceptedTagCount",    m.tagCount);
            Logger.recordOutput("Vision/AcceptedTagDist_ft",  m.avgTagDist * 3.28084);

            drivebase.addVisionMeasurement(m.pose, m.timestampSeconds,
                m.standardDeviations.times(stdDevMultiplier));
        }
        LoggedTracer.record("VisionManager");
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Private helpers
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Applies a 4-stage rejection filter and computes distance-based stddevs for
     * an MT2 pose estimate. Returns {@link Optional#empty()} if any stage rejects.
     *
     * <p>Uses MT2 pose directly (translation + MT2 heading) with rotation stddev
     * fixed at {@link VisionConstants#kRotationStdDev} so the gyro always owns
     * heading. This avoids injecting ambiguous MT1 single-solve rotation into the
     * estimator.
     *
     * <p>Stddev formula (ORCA3136): {@code xyStd = kXYStdDevBase * dist² / tagCount},
     * with a 2× single-tag penalty and a minimum floor.
     *
     * @param inputs      populated Limelight inputs for one camera
     * @param yawRateDegPS current robot yaw rate (deg/s) from the drive gyro
     */
    private Optional<MeasurementResult> extractMeasurement(
            LimelightIOInputsAutoLogged inputs, double yawRateDegPS) {

        // Stage 1 — Tag count: MT2 must see at least one tag.
        if (inputs.megaTag2TagCount == 0) {
            return Optional.empty();
        }

        // Stage 2 — Stale timestamp: reject frames older than kMaxTimestampAgeSec.
        double ts  = inputs.megaTag2TimestampSeconds;
        double now = Timer.getFPGATimestamp();
        if (ts <= 0 || (now - ts) > VisionConstants.kMaxTimestampAgeSec) {
            return Optional.empty();
        }

        // Stage 3 — High yaw rate: MT2 heading compensation can't keep up with fast spins.
        if (Math.abs(yawRateDegPS) > VisionConstants.kMaxYawRateDegPerSec) {
            return Optional.empty();
        }

        // Stage 4 — Field boundary: discard poses that land outside the field + margin.
        Pose2d pose = inputs.megaTag2Pose;
        double x    = pose.getX();
        double y    = pose.getY();
        if (x < VisionConstants.kFieldMinX || x > VisionConstants.kFieldMaxX
                || y < VisionConstants.kFieldMinY || y > VisionConstants.kFieldMaxY) {
            return Optional.empty();
        }

        // Distance-based XY stddevs (ORCA3136 formula):
        //   xyStd = base × dist² / tagCount, clamped to [kMinXYStdDev, ∞)
        // Single-tag ambiguity gets an additional 2× penalty.
        double dist     = inputs.megaTag2AvgTagDist;
        double tagCount = inputs.megaTag2TagCount;
        double xyStdDev = Math.max(
            VisionConstants.kMinXYStdDev,
            VisionConstants.kXYStdDevBase * (dist * dist) / tagCount
        );
        if (tagCount == 1) {
            xyStdDev *= VisionConstants.kSingleTagPenalty;
        }

        // Rotation stddev is 9999 — vision never corrects heading; gyro owns it.
        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, VisionConstants.kRotationStdDev);

        return Optional.of(new MeasurementResult(
            pose,
            ts,
            stdDevs,
            inputs.megaTag2AvgTagArea,
            inputs.megaTag2TagCount,
            dist
        ));
    }

    /**
     * Resets odometry to the MT2 pose if this is the first good fix (≥2 tags,
     * average tag distance ≤ {@link VisionConstants#kSeedMaxDistM}).
     *
     * <p>Only runs during the disabled period and at most once per power-cycle
     * to prevent mid-match jumps.
     */
    private void maybeSeedOdometry(LimelightIOInputsAutoLogged inputs) {
        if (inputs.megaTag2TagCount < VisionConstants.kSeedMinTagCount) return;
        if (inputs.megaTag2AvgTagDist > VisionConstants.kSeedMaxDistM) return;

        double ts  = inputs.megaTag2TimestampSeconds;
        double now = Timer.getFPGATimestamp();
        if (ts <= 0 || (now - ts) > VisionConstants.kMaxTimestampAgeSec) return;

        Pose2d seedPose = inputs.megaTag2Pose;
        double x = seedPose.getX(), y = seedPose.getY();
        if (x < VisionConstants.kFieldMinX || x > VisionConstants.kFieldMaxX
                || y < VisionConstants.kFieldMinY || y > VisionConstants.kFieldMaxY) return;

        drivebase.resetOdometry(seedPose);
        hasAppliedFirstFix = true;
        Logger.recordOutput("Vision/OdometrySeedPose", seedPose);
        Logger.recordOutput("Vision/OdometrySeedTagCount", inputs.megaTag2TagCount);
    }

    /** Immutable bundle of a processed Limelight pose estimate. */
    private static final class MeasurementResult {
        final Pose2d           pose;
        final double           timestampSeconds;
        final Matrix<N3, N1>   standardDeviations;
        final double           avgTagArea;
        final int              tagCount;
        final double           avgTagDist;

        MeasurementResult(Pose2d pose, double timestampSeconds,
                          Matrix<N3, N1> standardDeviations,
                          double avgTagArea, int tagCount, double avgTagDist) {
            this.pose               = pose;
            this.timestampSeconds   = timestampSeconds;
            this.standardDeviations = standardDeviations;
            this.avgTagArea         = avgTagArea;
            this.tagCount           = tagCount;
            this.avgTagDist         = avgTagDist;
        }
    }
}
