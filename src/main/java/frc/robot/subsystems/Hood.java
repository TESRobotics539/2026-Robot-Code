package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.config.ServoHubConfig;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Landmarks;
import frc.robot.Constants;
import frc.robot.Ports;

public class Hood extends SubsystemBase {
    // ServoHub channel assignments — update to match physical wiring
    private static final ChannelId kLeftChannel  = ChannelId.kChannelId0;
    private static final ChannelId kRightChannel = ChannelId.kChannelId5;

    // Pulse width bounds in microseconds — preserves the previous setBoundsMicroseconds(2000,1800,1500,1200,1000) values
    private static final int kMinPulseUs    = 1075;
    private static final int kCenterPulseUs = 1575;
    private static final int kMaxPulseUs    = 2075;

    private static final Distance kServoLength = Millimeters.of(100);
    private static final LinearVelocity kMaxServoSpeed = Millimeters.of(20).per(Second);
    private static final double kMinPosition = Constants.HoodConstants.kMinPosition;
    private static final double kMaxPosition = Constants.HoodConstants.kMaxPosition;
    private static final double kPositionTolerance = 0.01;

    private final ServoHub servoHub;
    private final ServoChannel leftChannel;
    private final ServoChannel rightChannel;

    private double currentPosition = 0.5;
    private double targetPosition  = 0.5;
    private Time lastUpdateTime = Seconds.of(0);

    public Hood() {
        servoHub = new ServoHub(Ports.kServoHub);

        // Configure pulse range to match the servo's mechanical travel
        ServoHubConfig config = new ServoHubConfig();
        config.channel0.pulseRange(kMinPulseUs, kCenterPulseUs, kMaxPulseUs);
        config.channel5.pulseRange(kMinPulseUs, kCenterPulseUs, kMaxPulseUs);
        servoHub.configure(config, ResetMode.kNoResetSafeParameters);

        leftChannel  = servoHub.getServoChannel(kLeftChannel);
        rightChannel = servoHub.getServoChannel(kRightChannel);

        // Enable both channels so the servo hub will drive them
        leftChannel.setPowered(true);
        leftChannel.setEnabled(true);
        rightChannel.setPowered(true);
        rightChannel.setEnabled(true);

        setPosition(currentPosition);
        SmartDashboard.putData(this);
    }

    private int positionToPulseWidth(double position) {
        return kMinPulseUs + (int)(position * (kMaxPulseUs - kMinPulseUs));
    }

    /** Expects a position between 0.0 and 1.0 */
    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
        leftChannel.setPulseWidth(positionToPulseWidth(clampedPosition));
        rightChannel.setPulseWidth(positionToPulseWidth(clampedPosition));
        targetPosition = clampedPosition;
    }

    private static final double kTrackingMinDistanceMeters = Constants.HoodConstants.kTrackingMinDistanceMeters;
    private static final double kTrackingMaxDistanceMeters = Constants.HoodConstants.kTrackingMaxDistanceMeters;
    private static final double kTrackingMinPosition = Constants.HoodConstants.kTrackingMinPosition;
    private static final double kTrackingMaxPosition = Constants.HoodConstants.kTrackingMaxPosition;

    /** Continuously adjusts hood position based on distance to the hub. */
    public Command trackHubCommand(Supplier<Pose2d> poseSupplier) {
        return run(() -> {
            Translation2d hubPos = Landmarks.hubPosition();
            double distanceMeters = poseSupplier.get().getTranslation().getDistance(hubPos);
            double t = MathUtil.clamp(
                (distanceMeters - kTrackingMinDistanceMeters) / (kTrackingMaxDistanceMeters - kTrackingMinDistanceMeters),
                0.0, 1.0);
            setPosition(MathUtil.interpolate(kTrackingMinPosition, kTrackingMaxPosition, t));
        });
    }

    /** Holds a fixed position indefinitely until interrupted. */
    public Command holdPositionCommand(double position) {
        return run(() -> setPosition(position));
    }

    /** Expects a position between 0.0 and 1.0 */
    public Command positionCommand(double position) {
        return runOnce(() -> setPosition(position))
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
    }

    private void updateCurrentPosition() {
        final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        if (isPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }

        final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
        final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
        currentPosition = targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
    }

    @Override
    public void periodic() {
        updateCurrentPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Current Position", () -> currentPosition, null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, value -> setPosition(value));
    }
}
