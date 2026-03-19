package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

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
     * Threshold matches the Limelight team's documented recommendation.
     */
    private static final double MAX_YAW_RATE_DEG_PER_SEC = 720.0;

    private final String name;

    // Cached last IMU reading — updated once per periodic() to avoid redundant NT calls.
    private LimelightHelpers.IMUData lastImu = new LimelightHelpers.IMUData();

    public Limelight(String name) {
        this.name = name;
        LimelightHelpers.SetIMUMode(name, 4);
    }

    @Override
    public void periodic() {
        lastImu = LimelightHelpers.getIMUData(name);
        Logger.recordOutput("Limelight/" + name + "/IMU/Pitch_deg",  lastImu.Pitch);
        Logger.recordOutput("Limelight/" + name + "/IMU/Yaw_deg",    lastImu.Yaw);
        Logger.recordOutput("Limelight/" + name + "/IMU/Roll_deg",   lastImu.Roll);
        Logger.recordOutput("Limelight/" + name + "/IMU/AccelZ_g",   lastImu.accelZ);
    }

    /**
     * Fetch a pose estimate from this Limelight.
     *
     * <p>Pitch, roll, and yaw rate are forwarded to {@code SetRobotOrientation} so MegaTag2 can
     * compensate for camera tilt (e.g. when traversing the bump) and dynamic rotation.
     * All angle parameters are in degrees; yawRate is degrees per second.
     */
    public Optional<Measurement> getMeasurement(
            Pose2d currentRobotPose, double pitchDeg, double rollDeg, double yawRateDegPerSec) {
        // Always update heading — keeps Limelight's internal state current so the first
        // post-spin frame has a fresh orientation reference even if we skipped reads.
        LimelightHelpers.SetRobotOrientation(
            name,
            currentRobotPose.getRotation().getDegrees(), yawRateDegPerSec,
            pitchDeg, 0,
            rollDeg, 0
        );

        // Skip the expensive NT reads and reject the pose entirely while spinning fast.
        // MegaTag2 XY is unreliable above MAX_YAW_RATE_DEG_PER_SEC regardless of heading input.
        if (Math.abs(yawRateDegPerSec) > MAX_YAW_RATE_DEG_PER_SEC) {
            Logger.recordOutput("Limelight/" + name + "/RejectedHighYawRate", true);
            return Optional.empty();
        }
        Logger.recordOutput("Limelight/" + name + "/RejectedHighYawRate", false);

        final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (
            poseEstimate_MegaTag1 == null
                || poseEstimate_MegaTag2 == null
                || poseEstimate_MegaTag1.tagCount == 0
                || poseEstimate_MegaTag2.tagCount == 0
        ) {
            return Optional.empty();
        }

        // Reject stale frames. A frame captured >100 ms ago depicts the robot at a position
        // that may be significantly different from its current location. Fusing such a
        // measurement actively harms pose estimate accuracy.
        // (Adapted from frc5687/2025-robot VisionSubsystem.)
        if (poseEstimate_MegaTag2.latency > MAX_MEASUREMENT_LATENCY_MS) {
            return Optional.empty();
        }

        // Combine the readings from MegaTag1 and MegaTag2:
        // 1. Use the more stable position from MegaTag2
        // 2. Use the rotation from MegaTag1 (with low confidence) to counteract gyro drift
        poseEstimate_MegaTag2.pose = new Pose2d(
            poseEstimate_MegaTag2.pose.getTranslation(),
            poseEstimate_MegaTag1.pose.getRotation()
        );

        // Dynamic XY std devs — scale trust with how much of the image the tags occupy.
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

        Logger.recordOutput("Limelight/" + name + "/EstimatedPose", poseEstimate_MegaTag2.pose);

        return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations, poseEstimate_MegaTag2.avgTagArea));
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;
        public final double avgTagArea;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations, double avgTagArea) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
            this.avgTagArea = avgTagArea;
        }
    }    public double getYawStdDev() {
        double[] stddevs = NetworkTableInstance.getDefault().getTable(name)
                          .getEntry("stddevs").getDoubleArray(new double[12]);
        return stddevs[5];
    }


    /** Returns the cached accelerometer Z-axis reading (g) from the last periodic() call. */
    public double getAccelZ() {
        return lastImu.accelZ;
    }

    public boolean getTV() {
        return LimelightHelpers.getTV(name);
    }

    public double getTX() {
        return LimelightHelpers.getTX(name);
    }
}
