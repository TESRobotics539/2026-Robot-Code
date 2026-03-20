package frc.robot.subsystems.iodiagnostics;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class LimelightIOReal implements LimelightIO {

    private final String name;

    public LimelightIOReal(String name) {
        this.name = name;
        // Mode 0: disable Limelight's internal IMU — SetRobotOrientation (Pigeon2) is the
        // sole orientation source for MegaTag2. Mode 4 (full fusion) makes the Limelight's
        // onboard IMU the primary heading source, reducing SetRobotOrientation to a correction
        // hint and causing MegaTag2 to diverge from the Pigeon2 heading.
        LimelightHelpers.SetIMUMode(name, 0);
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs,
                             double robotHeadingDeg,
                             double yawRateDegPS,
                             double pitchDeg,
                             double rollDeg) {
        // Send orientation before reading pose estimates so MegaTag2 uses the
        // current heading in its calculation.
        LimelightHelpers.SetRobotOrientation(
            name,
            robotHeadingDeg, yawRateDegPS,
            pitchDeg, 0,
            rollDeg, 0
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
        } else {
            inputs.megaTag1Pose             = new Pose2d();
            inputs.megaTag1TagCount         = 0;
            inputs.megaTag1LatencyMs        = 0.0;
            inputs.megaTag1AvgTagArea       = 0.0;
            inputs.megaTag1TimestampSeconds = 0.0;
        }

        // MegaTag2
        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (mt2 != null) {
            inputs.megaTag2Pose             = mt2.pose;
            inputs.megaTag2TagCount         = mt2.tagCount;
            inputs.megaTag2LatencyMs        = mt2.latency;
            inputs.megaTag2AvgTagArea       = mt2.avgTagArea;
            inputs.megaTag2AvgTagDist       = mt2.avgTagDist;
            inputs.megaTag2TimestampSeconds = mt2.timestampSeconds;
        } else {
            inputs.megaTag2Pose             = new Pose2d();
            inputs.megaTag2TagCount         = 0;
            inputs.megaTag2LatencyMs        = 0.0;
            inputs.megaTag2AvgTagArea       = 0.0;
            inputs.megaTag2AvgTagDist       = 0.0;
            inputs.megaTag2TimestampSeconds = 0.0;
        }
    }
}
