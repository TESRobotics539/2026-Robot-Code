package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;

    // IMU telemetry publishers — visible in Elastic under SmartDashboard/<name>/
    private final DoublePublisher imuPitchPub;
    private final DoublePublisher imuYawPub;
    private final DoublePublisher imuRollPub;
    private final DoublePublisher imuAccelZPub;

    // Cached last IMU reading — updated once per periodic() to avoid redundant NT calls.
    private LimelightHelpers.IMUData lastImu = new LimelightHelpers.IMUData();

    public Limelight(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();

        imuPitchPub  = telemetryTable.getDoubleTopic("IMU Pitch (deg)").publish();
        imuYawPub    = telemetryTable.getDoubleTopic("IMU Yaw (deg)").publish();
        imuRollPub   = telemetryTable.getDoubleTopic("IMU Roll (deg)").publish();
        imuAccelZPub = telemetryTable.getDoubleTopic("IMU Accel Z (g)").publish();

        LimelightHelpers.SetIMUMode(name, 4);
    }

    @Override
    public void periodic() {
        lastImu = LimelightHelpers.getIMUData(name);
        imuPitchPub.set(lastImu.Pitch);
        imuYawPub.set(lastImu.Yaw);
        imuRollPub.set(lastImu.Roll);
        imuAccelZPub.set(lastImu.accelZ);
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
        LimelightHelpers.SetRobotOrientation(
            name,
            currentRobotPose.getRotation().getDegrees(), yawRateDegPerSec,
            pitchDeg, 0,
            rollDeg, 0
        );

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

        // Combine the readings from MegaTag1 and MegaTag2:
        // 1. Use the more stable position from MegaTag2
        // 2. Use the rotation from MegaTag1 (with low confidence) to counteract gyro drift
        poseEstimate_MegaTag2.pose = new Pose2d(
            poseEstimate_MegaTag2.pose.getTranslation(),
            poseEstimate_MegaTag1.pose.getRotation()
        );
        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.1, 0.1, 10.0);

        posePublisher.set(poseEstimate_MegaTag2.pose);

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
    }




    public Optional<Pose2d> getPoseEstimateMT2() {
        LimelightHelpers.PoseEstimate megaTag2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        if (megaTag2Pose != null && megaTag2Pose.tagCount > 0)
            return Optional.of(megaTag2Pose.pose);

        return Optional.empty();
    }

    public Optional<Pose2d> getPoseEstimateMT1() {
        LimelightHelpers.PoseEstimate megaTag1Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (megaTag1Pose != null && megaTag1Pose.tagCount > 0)
            return Optional.of(megaTag1Pose.pose);

        return Optional.empty();
    }

    public double getYawStdDev() {
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
