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
import edu.wpi.first.wpilibj.Timer;
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
    private final SparkClosedLoopController rollerController;

    private double targetPivotPosition = 0.0;
    private boolean usePercentOutput = false;
    private boolean rollerRunning = false;
    private int rollerSpikeCount = 0;
    private boolean lastRollerAboveThreshold = false;
    private final Timer rollerNoLoadTimer = new Timer();
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
        rollerController = rollerMotor.getClosedLoopController();
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
            .pid(Constants.IntakeConstants.kPivotP, Constants.IntakeConstants.kPivotI, Constants.IntakeConstants.kPivotD)
            .outputRange(Constants.IntakeConstants.kPivotOutputRangeMin, Constants.IntakeConstants.kPivotOutputRangeMax);

        // PID uses the absolute encoder; the relative encoder position and velocity are never read.
        config.signals
            .primaryEncoderPositionPeriodMs(500)
            .primaryEncoderVelocityPeriodMs(500);

        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureRollerMotor() {
        SparkFlexConfig config = new SparkFlexConfig();

        config.inverted(true);
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(80);
        config.secondaryCurrentLimit(120);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.IntakeConstants.kRollerP, Constants.IntakeConstants.kRollerI, Constants.IntakeConstants.kRollerD)
            .velocityFF(12.0 / Constants.IntakeConstants.kRollerFreeSpeedRPM);

        // Position is never read on the Rio; velocity and current are needed for ball detection.
        config.signals
            .primaryEncoderPositionPeriodMs(500);

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

        // Detect rising edges of roller current to count fuel pickups
        boolean aboveThreshold = rollerRunning &&
            rollerMotor.getOutputCurrent() > Constants.IntakeConstants.kRollerLoadCurrentThreshold;
        if (aboveThreshold && !lastRollerAboveThreshold) {
            rollerSpikeCount++;
            rollerNoLoadTimer.restart(); // Reset window after each pickup
        }
        lastRollerAboveThreshold = aboveThreshold;

        // Cut off rollers if no load spike seen within the timeout window
        if (rollerRunning && rollerNoLoadTimer.hasElapsed(Constants.IntakeConstants.kRollerNoLoadTimeoutSeconds)) {
            stopRoller();
            rollerRunning = false;
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

    public void setRollerSpeed(double rpm) {
        rollerController.setSetpoint(rpm, ControlType.kVelocity);
        rollerNoLoadTimer.restart();
    }

    public void stopRoller() {
        rollerMotor.set(0);
        rollerNoLoadTimer.stop();
        rollerNoLoadTimer.reset();
    }

    /** Returns true once the roller has seen enough current spikes to confirm fuel pickup. */
    public boolean hasPickedUpFuel() {
        return rollerSpikeCount >= Constants.IntakeConstants.kRollerFuelSpikeCount;
    }

    /** Resets the fuel pickup spike counter. Call at autonomous start. */
    public void resetFuelDetection() {
        rollerSpikeCount = 0;
        lastRollerAboveThreshold = false;
    }

    private boolean isDeployed() {
        return targetPivotPosition != 0.0 && targetPivotPosition < Position.STOWED.value - 0.05;
    }

    /**
     * Single trigger pull: if stowed → deploy + start rollers.
     * If deployed and rollers running → stop rollers.
     * If deployed and rollers stopped → start rollers.
     */
    public Command intakePressCommand() {
        return runOnce(() -> {
            if (!isDeployed()) {
                setPivotPosition(Position.DEPLOYED);
                setRollerSpeed(Constants.IntakeConstants.kRollerRPM);
                rollerRunning = true;
            } else if (rollerRunning) {
                stopRoller();
                rollerRunning = false;
            } else {
                setRollerSpeed(Constants.IntakeConstants.kRollerRPM);
                rollerRunning = true;
            }
        });
    }

    /** Double-tap: stow the intake and stop rollers. */
    public Command stowCommand() {
        return runOnce(() -> {
            setPivotPosition(Position.STOWED);
            stopRoller();
            rollerRunning = false;
        });
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
