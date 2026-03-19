package frc.robot.subsystems.iodiagnostics;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {

    @AutoLog
    class LimelightIOInputs {
        // IMU data fused by the Limelight's onboard IMU (mode 4).
        public double imuPitchDeg = 0.0;
        public double imuRollDeg  = 0.0;
        public double imuYawDeg   = 0.0;
        public double imuAccelZG  = 0.0;

        // MegaTag1 — provides an independent rotation estimate from tag geometry.
        public Pose2d megaTag1Pose             = new Pose2d();
        public int    megaTag1TagCount         = 0;
        public double megaTag1LatencyMs        = 0.0;
        public double megaTag1AvgTagArea       = 0.0;
        public double megaTag1TimestampSeconds = 0.0;

        // MegaTag2 — provides a stable translation estimate; uses gyro for heading.
        public Pose2d megaTag2Pose             = new Pose2d();
        public int    megaTag2TagCount         = 0;
        public double megaTag2LatencyMs        = 0.0;
        public double megaTag2AvgTagArea       = 0.0;
        public double megaTag2TimestampSeconds = 0.0;
    }

    /**
     * Sends the current robot orientation to the Limelight (for MegaTag2 heading
     * compensation) and reads all sensor data — IMU, MegaTag1, and MegaTag2 — into
     * {@code inputs}.
     *
     * <p>Orientation must be supplied on every call so MegaTag2 always uses a fresh
     * heading. Both operations are combined here to guarantee SetRobotOrientation is
     * called immediately before the pose estimates are read.
     *
     * @param inputs            struct to populate
     * @param robotHeadingDeg   current robot heading from the gyro (degrees, blue-origin)
     * @param yawRateDegPS      current yaw rate (degrees per second)
     * @param pitchDeg          current pitch (degrees)
     * @param rollDeg           current roll (degrees)
     */
    default void updateInputs(LimelightIOInputs inputs,
                              double robotHeadingDeg,
                              double yawRateDegPS,
                              double pitchDeg,
                              double rollDeg) {}
}
