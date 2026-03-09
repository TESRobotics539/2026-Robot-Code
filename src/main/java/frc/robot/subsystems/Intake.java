package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Intake extends SubsystemBase {

    public enum Position {
        STOWED(Constants.IntakeConstants.kStowedPosition),
        DEPLOYED(Constants.IntakeConstants.kDeployedPosition); // TODO: tune absolute encoder value

        public final double value;

        Position(double value) {
            this.value = value;
        }
    }

    private static final double kMinPosition = Constants.IntakeConstants.kMinPosition;
    private static final double kMaxPosition = Constants.IntakeConstants.kMaxPosition;

    private final SparkMax pivotMotor;
    private final SparkClosedLoopController pivotController;
    private final AbsoluteEncoder absEncoder;
    private final RelativeEncoder encoder;

    private final SparkFlex rollerMotor;

    private double targetPivotPosition = 0.0;
    private boolean usePercentOutput = false;
    private boolean rollerRunning = false;
    private boolean matchStowLocked = false;
    private boolean deployedPositionCalibrated = false;
    private boolean initialDeployEnabled = false;   // true only when the match-start deploy fires
    private double  deployStartPosition   = 0.0;    // encoder position when deploy began
    private double  calibratedDeployedPosition = Double.NaN; // persists across deploys once set
    private final Debouncer deployCurrentDebouncer = new Debouncer(Constants.IntakeConstants.kPivotDeployedCurrentDebounceSeconds, DebounceType.kRising);

    public Intake() {
        pivotMotor = new SparkMax(Ports.kIntakePivot, MotorType.kBrushless);
        pivotController = pivotMotor.getClosedLoopController();
        absEncoder = pivotMotor.getAbsoluteEncoder();
        encoder = pivotMotor.getEncoder();
        configurePivotMotor();

        rollerMotor = new SparkFlex(Ports.kIntakeRollers, MotorType.kBrushless);
        configureRollerMotor();

        SmartDashboard.putData(this);
    }

    private void configurePivotMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(true)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12);

        config.smartCurrentLimit(80)
            .secondaryCurrentLimit(120);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(1.5, 0, 5)
            .outputRange(-0.15, 0.4);

        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureRollerMotor() {
        SparkFlexConfig config = new SparkFlexConfig();

        config.inverted(true);
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(80);
        config.secondaryCurrentLimit(120);

        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        if (matchStowLocked) {
            usePercentOutput = false;
        }
        // Snap deployed position to the hard stop once the intake has traveled >= 0.2 encoder
        // rotations from where it started deploying. Only runs on the initial match deploy.
        if (!usePercentOutput && !deployedPositionCalibrated && initialDeployEnabled && isDeployed()) {
            boolean travelMet = Math.abs(absEncoder.getPosition() - deployStartPosition) >= Constants.IntakeConstants.kPivotDeployedTravelThreshold;
            boolean spiked = deployCurrentDebouncer.calculate(
                travelMet && pivotMotor.getOutputCurrent() > Constants.IntakeConstants.kPivotDeployedCurrentThreshold);
            if (spiked) {
                calibratedDeployedPosition = Math.max(kMinPosition, Math.min(kMaxPosition, absEncoder.getPosition()));
                targetPivotPosition = calibratedDeployedPosition;
                deployedPositionCalibrated = true;
                initialDeployEnabled = false;
            }
        }
        if (!usePercentOutput && targetPivotPosition != 0.0) {
            pivotController.setSetpoint(targetPivotPosition, ControlType.kPosition);
        }
    }

    private void setPivotIdleMode(IdleMode mode) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(mode);
        pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setPivotPosition(Position position) {
        if (matchStowLocked) return;
        usePercentOutput = false;
        if (position == Position.DEPLOYED) {
            setPivotIdleMode(IdleMode.kCoast);
            deployedPositionCalibrated = false;
            deployStartPosition = absEncoder.getPosition();
            deployCurrentDebouncer.calculate(false); // reset debouncer state
            // Reuse the calibrated position if already established, otherwise use the constant
            targetPivotPosition = !Double.isNaN(calibratedDeployedPosition)
                ? calibratedDeployedPosition
                : Math.max(kMinPosition, Math.min(kMaxPosition, position.value));
        } else if (position == Position.STOWED) {
            setPivotIdleMode(IdleMode.kBrake);
            targetPivotPosition = Math.max(kMinPosition, Math.min(kMaxPosition, position.value));
        }
    }

    /**
     * Deploys the intake and enables current-spike calibration for this deploy.
     * Call only from the automatic match-start deploy trigger.
     */
    public void setInitialDeployPosition() {
        initialDeployEnabled = true;
        setPivotPosition(Position.DEPLOYED);
    }

    /** Forces brake mode regardless of current position — call at autonomous start. */
    public void enforceBrakeMode() {
        setPivotIdleMode(IdleMode.kBrake);
    }

    /**
     * Reads the current absolute encoder position and sets it as the PID target.
     * If {@link Constants#kStowIntakeForMatch} is enabled, also locks the pivot for
     * the rest of the match so any subsequent calls to {@link #setPivotPosition} are ignored.
     */
    public void lockCurrentPositionAsStow() {
        double currentPos = absEncoder.getPosition();
        targetPivotPosition = Math.max(kMinPosition, Math.min(kMaxPosition, currentPos));
        usePercentOutput = false;
        matchStowLocked = Constants.IntakeConstants.kStowIntakeForMatch;
    }

    public void setPivotPercentOutput(double percentOutput) {
        usePercentOutput = true;
        pivotMotor.set(percentOutput);
    }

    public void setRollerSpeed(double percentOutput) {
        rollerMotor.set(percentOutput);
    }

    public void stopRoller() {
        rollerMotor.set(0);
    }

    private boolean isDeployed() {
        return targetPivotPosition != 0.0 && targetPivotPosition < Position.STOWED.value - 0.05;
    }

    /**
     * Holds the subsystem for up to 0.5 seconds to distinguish short vs long press.
     * Short press (released before 0.5s):
     *   - If stowed: deploy + start rollers
     *   - If deployed: toggle rollers on/off
     * Long press (held >= 0.5s): stow intake and stop rollers.
     *
     * Bind onTrue to this command and onFalse to {@link #cancelPressCommand()}.
     */
    public Command intakePressCommand() {
        return run(() -> {})
            .withTimeout(Constants.IntakeConstants.kLongPressThresholdSeconds)
            .finallyDo(interrupted -> {
                if (interrupted) {
                    // Short press
                    if (!isDeployed()) {
                        setPivotPosition(Position.DEPLOYED);
                        setRollerSpeed(Constants.IntakeConstants.kRollerSpeed);
                        rollerRunning = true;
                    } else if (rollerRunning) {
                        stopRoller();
                        rollerRunning = false;
                    } else {
                        setRollerSpeed(Constants.IntakeConstants.kRollerSpeed);
                        rollerRunning = true;
                    }
                } else {
                    // Long press — stow
                    setPivotPosition(Position.STOWED);
                    stopRoller();
                    rollerRunning = false;
                }
            });
    }

    /** Schedule this onFalse to cancel intakePressCommand and trigger short-press logic. */
    public Command cancelPressCommand() {
        return runOnce(() -> {});
    }

    /**
     * Repeatedly pulses the intake pivot to agitate fuel during shooting.
     * Pattern: 25% up for 0.33s, then 5% down for 0.2s, repeat.
     * Restores position control when interrupted.
     */
    public Command agitateCommand() {
        return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(Constants.IntakeConstants.kAgitateUpPower)),
            Commands.waitSeconds(Constants.IntakeConstants.kAgitateUpSeconds),
            runOnce(() -> setPivotPercentOutput(Constants.IntakeConstants.kAgitateDownPower)),
            Commands.waitSeconds(Constants.IntakeConstants.kAgitateDownSeconds)
        ).repeatedly()
        .finallyDo(() -> usePercentOutput = false);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Abs Encoder Position", () -> absEncoder.getPosition(), null);
        builder.addDoubleProperty("Rel Encoder Position", () -> encoder.getPosition(), null);
        builder.addDoubleProperty("Target Position", () -> targetPivotPosition, null);
        builder.addDoubleProperty("Pivot Current (A)", () -> pivotMotor.getOutputCurrent(), null);
        builder.addDoubleProperty("Roller RPM", () -> rollerMotor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("Roller Current (A)", () -> rollerMotor.getOutputCurrent(), null);
    }
}
