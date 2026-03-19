package frc.robot.subsystems.iodiagnostics;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class LimelightIOReal implements LimelightIO {

    private final String name;

    public LimelightIOReal(String name) {
        this.name = name;
        // Start in SyncInternalImu mode so the Limelight IMU calibrates from the
        // Pigeon during boot/disabled. VisionManager will switch to ExternalImu
        // when the robot is enabled.
        LimelightHelpers.SetIMUMode(name, 4);
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs,
                             double robotHeadingDeg,
                             double yawRateDegPS,
                             double pitchDeg,
                             double pitchRateDegPS,
                             double rollDeg,
                             double rollRateDegPS) {
        // Send orientation before reading pose estimates so MegaTag2 uses the
        // current heading in its calculation.
        LimelightHelpers.SetRobotOrientation(
            name,
            robotHeadingDeg, yawRateDegPS,
            pitchDeg, pitchRateDegPS,
            rollDeg, rollRateDegPS
        );

        // IMU
        LimelightHelpers.IMUData imu = LimelightHelpers.getIMUData(name);
        inputs.imuPitchDeg = imu.Pitch;
        inputs.imuRollDeg  = imu.Roll;
        inputs.imuYawDeg   = imu.Yaw;
        inputs.imuAccelZG  = imu.accelZ;

        // MegaTag1
        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (mt1 != null) {
            inputs.megaTag1Pose             = mt1.pose;
            inputs.megaTag1TagCount         = mt1.tagCount;
            inputs.megaTag1LatencyMs        = mt1.latency;
            inputs.megaTag1AvgTagArea       = mt1.avgTagArea;
            inputs.megaTag1TimestampSeconds = mt1.timestampSeconds;
            inputs.megaTag1AvgTagDist       = mt1.avgTagDist;
        } else {
            inputs.megaTag1Pose             = new Pose2d();
            inputs.megaTag1TagCount         = 0;
            inputs.megaTag1LatencyMs        = 0.0;
            inputs.megaTag1AvgTagArea       = 0.0;
            inputs.megaTag1TimestampSeconds = 0.0;
            inputs.megaTag1AvgTagDist       = 0.0;
        }

        // MegaTag2
        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (mt2 != null) {
            inputs.megaTag2Pose             = mt2.pose;
            inputs.megaTag2TagCount         = mt2.tagCount;
            inputs.megaTag2LatencyMs        = mt2.latency;
            inputs.megaTag2AvgTagArea       = mt2.avgTagArea;
            inputs.megaTag2TimestampSeconds = mt2.timestampSeconds;
            inputs.megaTag2AvgTagDist       = mt2.avgTagDist;
        } else {
            inputs.megaTag2Pose             = new Pose2d();
            inputs.megaTag2TagCount         = 0;
            inputs.megaTag2LatencyMs        = 0.0;
            inputs.megaTag2AvgTagArea       = 0.0;
            inputs.megaTag2TimestampSeconds = 0.0;
            inputs.megaTag2AvgTagDist       = 0.0;
        }
    }

    @Override
    public void setImuMode(int mode) {
        LimelightHelpers.SetIMUMode(name, mode);
    }
}
