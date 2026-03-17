package frc.robot.subsystems.iodiagnostics;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants;
import frc.robot.Ports;

/**
 * Real UltraShooterIO — three SparkFlex (Neo Vortex) motors in a fixed-hood flywheel.
 *
 * <p>Hardware layout:
 * <ul>
 *   <li>Left  (CAN {kShooterLeft})   — not inverted</li>
 *   <li>Middle (CAN {kShooterMiddle}) — not inverted</li>
 *   <li>Right  (CAN {kShooterRight})  — inverted</li>
 * </ul>
 *
 * <p>Velocity units throughout are ft/s (surface speed), using the
 * {@code velocityConversionFactor} baked into the SparkFlex encoder config.
 */
public class UltraShooterIOReal implements UltraShooterIO {

    private static final double WHEEL_DIAMETER_FEET      = 4.0 / 12.0;
    private static final double WHEEL_CIRCUMFERENCE_FEET = Math.PI * WHEEL_DIAMETER_FEET;
    /** RPM → ft/s surface velocity conversion factor. */
    private static final double VELOCITY_CONVERSION      = WHEEL_CIRCUMFERENCE_FEET / 60.0;
    /** Neo Vortex free-speed surface velocity (ft/s) — used for KV feedforward. */
    private static final double FREE_SPEED_FPS           = 6784.0 * VELOCITY_CONVERSION;
    /** KV in V/(ft/s): 12 V at free speed. */
    private static final double KV                       = 12.0 / FREE_SPEED_FPS;

    private final SparkFlex left;
    private final SparkFlex middle;
    private final SparkFlex right;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder middleEncoder;
    private final RelativeEncoder rightEncoder;

    private final SparkClosedLoopController leftPID;
    private final SparkClosedLoopController middlePID;
    private final SparkClosedLoopController rightPID;

    public UltraShooterIOReal() {
        left   = new SparkFlex(Ports.kShooterLeft,   MotorType.kBrushless);
        middle = new SparkFlex(Ports.kShooterMiddle, MotorType.kBrushless);
        right  = new SparkFlex(Ports.kShooterRight,  MotorType.kBrushless);

        leftEncoder   = left.getEncoder();
        middleEncoder = middle.getEncoder();
        rightEncoder  = right.getEncoder();

        leftPID   = left.getClosedLoopController();
        middlePID = middle.getClosedLoopController();
        rightPID  = right.getClosedLoopController();

        configureMotor(left,   /* inverted */ false);
        configureMotor(middle, /* inverted */ false);
        configureMotor(right,  /* inverted */ true);
    }

    private void configureMotor(SparkFlex motor, boolean inverted) {
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.inverted(inverted)
           .idleMode(IdleMode.kCoast)
           .smartCurrentLimit(
               Constants.UltraShooterConstants.kSmartCurrentLimit,
               Constants.UltraShooterConstants.kFreeCurrentLimit)
           .secondaryCurrentLimit(Constants.UltraShooterConstants.kStatorCurrentLimit);
        cfg.encoder
           .velocityConversionFactor(VELOCITY_CONVERSION);
        cfg.closedLoop
           .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
           .pid(Constants.UltraShooterConstants.kP,
                Constants.UltraShooterConstants.kI,
                Constants.UltraShooterConstants.kD);
        // Position is never used — slow the signal down to reduce CAN traffic.
        cfg.signals
           .primaryEncoderPositionPeriodMs(500);
        motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(UltraShooterIOInputs inputs) {
        inputs.leftVelocityFPS   = leftEncoder.getVelocity();
        inputs.middleVelocityFPS = middleEncoder.getVelocity();
        inputs.rightVelocityFPS  = rightEncoder.getVelocity();
        inputs.leftCurrentAmps   = left.getOutputCurrent();
        inputs.middleCurrentAmps = middle.getOutputCurrent();
        inputs.rightCurrentAmps  = right.getOutputCurrent();
    }

    @Override
    public void setVelocity(double velocityFPS, double feedforwardVolts) {
        leftPID  .setSetpoint(velocityFPS, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforwardVolts);
        middlePID.setSetpoint(velocityFPS, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforwardVolts);
        rightPID .setSetpoint(velocityFPS, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforwardVolts);
    }

    @Override
    public void stop() {
        left  .set(0);
        middle.set(0);
        right .set(0);
    }

    @Override
    public void setKp(double kP) {
        // configure() is an expensive CAN write — only safe to call while disabled.
        if (!edu.wpi.first.wpilibj.DriverStation.isDisabled()) return;
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.closedLoop
           .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
           .pid(kP, Constants.UltraShooterConstants.kI, Constants.UltraShooterConstants.kD)
           .velocityFF(KV);
        left  .configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        middle.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        right .configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
