package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.iodiagnostics.LimelightIO;
import frc.robot.subsystems.iodiagnostics.LimelightIOInputsAutoLogged;
import frc.robot.subsystems.robot.Swerve;
import frc.util.LoggedTracer;

/**
 * Owns both Limelights, fuses their pose estimates, and feeds the best
 * measurement into the drivetrain pose estimator each loop.
 *
 * <p>All hardware reads go through {@link LimelightIO#updateInputs} and are
 * logged via {@code Logger.processInputs()} so AdvantageKit can replay every
 * fusion decision exactly as it happened on the robot.
 *
 * <p>This class replaces the old {@code Limelight} SubsystemBase and the
 * {@code updateVisionCommand()} default-command pattern in RobotContainer.
 */
public class VisionManager extends SubsystemBase {

    // Rejection threshold sourced from Constants — see VisionConstants.kMaxMeasurementLatencyMs.

    private final LimelightIO frontIO;
    private final LimelightIO rearIO;
    private final Swerve drivebase;

    private final LimelightIOInputsAutoLogged frontInputs = new LimelightIOInputsAutoLogged();
    private final LimelightIOInputsAutoLogged rearInputs  = new LimelightIOInputsAutoLogged();

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
        Pose2d currentPose    = drivebase.getPose();
        double headingDeg     = currentPose.getRotation().getDegrees();
        double yawRateDegPS   = drivebase.getYawRateDegPerSec();
        double pitchDeg       = drivebase.getPitchDegrees();
        double rollDeg        = drivebase.getRollDegrees();

        // Read all hardware data (SetRobotOrientation + NT reads) for both cameras.
        frontIO.updateInputs(frontInputs, headingDeg, yawRateDegPS, pitchDeg, rollDeg);
        Logger.processInputs("Vision/FrontLimelight", frontInputs);

        rearIO.updateInputs(rearInputs, headingDeg, yawRateDegPS, pitchDeg, rollDeg);
        Logger.processInputs("Vision/RearLimelight", rearInputs);

        // ── Bump detection ────────────────────────────────────────────────────
        // 2-of-3 majority vote: Pigeon 2 + both Limelight accelerometers.
        // Requires two sensors to agree to avoid false positives from single-sensor
        // noise. Camera shake during bump traversal makes tag estimates unreliable.
        boolean pigeonOverBump = drivebase.isOverBump();
        boolean frontOverBump  = Math.abs(frontInputs.imuAccelZG - 1.0)
            > Constants.BumpDetectionConstants.kLimelightAccelZDeviationThreshold;
        boolean rearOverBump   = Math.abs(rearInputs.imuAccelZG - 1.0)
            > Constants.BumpDetectionConstants.kLimelightAccelZDeviationThreshold;
        int     bumpVotes      = (pigeonOverBump ? 1 : 0) + (frontOverBump ? 1 : 0) + (rearOverBump ? 1 : 0);
        boolean overBump       = bumpVotes >= 2;

        // ── Pose estimate extraction ──────────────────────────────────────────
        Optional<MeasurementResult> frontMeasurement = extractMeasurement(frontInputs);
        Optional<MeasurementResult> rearMeasurement  = extractMeasurement(rearInputs);

        // Confidence = avgTagArea × tagCount. Larger area = robot is closer = less
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

        // ── Logging ───────────────────────────────────────────────────────────
        Logger.recordOutput("Vision/ActiveSource",   activeSource);
        Logger.recordOutput("Vision/FrontConfidence", frontConf);
        Logger.recordOutput("Vision/RearConfidence",  rearConf);
        Logger.recordOutput("Vision/BestConfidence",  bestConf);
        Logger.recordOutput("Vision/OverBump",        overBump);
        Logger.recordOutput("Vision/BumpVotes",       bumpVotes);
        Logger.recordOutput("Vision/WheelSlipping",   isSlipping);
        Logger.recordOutput("Vision/WheelSlipScore",  drivebase.getWheelSlipScore());

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
            Logger.recordOutput("Vision/StdDevMultiplier", stdDevMultiplier);

            // Reject invalid timestamps — 0 or future values indicate a stale frame.
            double ts = m.timestampSeconds;
            if (ts > 0 && ts <= Timer.getFPGATimestamp()) {
                drivebase.addVisionMeasurement(m.pose, ts, m.standardDeviations.times(stdDevMultiplier));
            }
        }
        LoggedTracer.record("VisionManager");
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Private helpers
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Produces a fused pose estimate from a set of Limelight inputs, or
     * {@link Optional#empty()} if the inputs are invalid or too stale.
     *
     * <p>Strategy: use the MT2 translation (gyro-aided, more stable) combined
     * with the MT1 rotation (independent tag geometry, counteracts gyro drift).
     */
    private Optional<MeasurementResult> extractMeasurement(LimelightIOInputsAutoLogged inputs) {
        if (inputs.megaTag1TagCount == 0 || inputs.megaTag2TagCount == 0) {
            return Optional.empty();
        }
        if (inputs.megaTag2LatencyMs > Constants.VisionConstants.kMaxMeasurementLatencyMs) {
            return Optional.empty();
        }

        // Combine MT2 translation with MT1 rotation.
        Pose2d combinedPose = new Pose2d(
            inputs.megaTag2Pose.getTranslation(),
            inputs.megaTag1Pose.getRotation()
        );

        // Dynamic XY stddevs — larger tag area means the robot is closer to the
        // tags and projection error is smaller. Multi-tag sightings halve the stddev.
        // Formula (adapted from frc5687/2025-robot VisionSTDFilter):
        //   xyStdDev = 0.1 / max(0.01, avgTagArea), clamped [0.03, 3.0]
        double avgTagArea = inputs.megaTag2AvgTagArea;
        double xyStdDev   = Math.max(0.03, Math.min(3.0, 0.1 / Math.max(0.01, avgTagArea)));
        if (inputs.megaTag2TagCount > 1) xyStdDev *= 0.5;
        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, 10.0);

        return Optional.of(new MeasurementResult(
            combinedPose,
            inputs.megaTag2TimestampSeconds,
            stdDevs,
            avgTagArea,
            inputs.megaTag2TagCount
        ));
    }

    /** Immutable bundle of a processed Limelight pose estimate. */
    private static final class MeasurementResult {
        final Pose2d           pose;
        final double           timestampSeconds;
        final Matrix<N3, N1>   standardDeviations;
        final double           avgTagArea;
        final int              tagCount;

        MeasurementResult(Pose2d pose, double timestampSeconds,
                          Matrix<N3, N1> standardDeviations,
                          double avgTagArea, int tagCount) {
            this.pose               = pose;
            this.timestampSeconds   = timestampSeconds;
            this.standardDeviations = standardDeviations;
            this.avgTagArea         = avgTagArea;
            this.tagCount           = tagCount;
        }
    }
}
