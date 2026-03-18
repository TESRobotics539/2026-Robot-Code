package frc.robot.subsystems.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.iodiagnostics.IntakeIO;
import frc.robot.subsystems.iodiagnostics.IntakeIOInputsAutoLogged;
import frc.util.LoggedTracer;

public class Intake extends SubsystemBase {

    public enum Position {
        STOWED(Constants.IntakeConstants.kStowedPosition),
        DEPLOYED(Constants.IntakeConstants.kDeployedPosition);

        public final double value;

        Position(double value) {
            this.value = value;
        }
    }

    private static final double kMinPosition = Constants.IntakeConstants.kMinPosition;
    private static final double kMaxPosition = Constants.IntakeConstants.kMaxPosition;
    /** Maximum deviation from kStowedPosition at which a measured position is accepted as a valid stow calibration. */
    private static final double kStowCalibrationTolerance = 0.05;

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private double targetPivotPosition = 0.0;
    private boolean hasSetpoint = false;
    private boolean usePercentOutput = false;
    private boolean rollerRunning = false;
    private int rollerSpikeCount = 0;
    private boolean lastRollerAboveThreshold = false;
    private boolean rollerJustSpiked = false;
    private final Timer rollerNoLoadTimer = new Timer();

    // Roller anti-jam state machine
    private static final int ROLLER_RUNNING    = 0;
    private static final int ROLLER_REVERSING  = 1;
    private static final int ROLLER_RECOVERING = 2;
    private int rollerUnjamState = ROLLER_RUNNING;
    private final Timer rollerUnjamTimer = new Timer();
    private boolean hasZeroed = false;
    private boolean matchStowLocked = false;
    private boolean deployedPositionCalibrated = false;
    private boolean initialDeployEnabled = false;
    private double  deployStartPosition   = 0.0;
    private double  calibratedDeployedPosition = Double.NaN;
    private double  calibratedStowedPosition   = Double.NaN;
    private final Debouncer deployCurrentDebouncer = new Debouncer(Constants.IntakeConstants.kPivotDeployedCurrentDebounceSeconds, DebounceType.kRising);

    public Intake(IntakeIO io) {
        this.io = io;
        SmartDashboard.putData(this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // On first cycle, snapshot the current encoder position so the PID has a valid
        // target before any match period begins (e.g., during testing or teleop-only runs).
        if (!hasZeroed) {
            double currentPos = inputs.pivotAbsEncoderPosition;
            targetPivotPosition = Math.max(kMinPosition, Math.min(kMaxPosition, currentPos));
            if (Math.abs(currentPos - Constants.IntakeConstants.kStowedPosition) <= kStowCalibrationTolerance) {
                calibratedStowedPosition = currentPos;
            }
            hasSetpoint = true;
            hasZeroed = true;
        }

        if (matchStowLocked) {
            usePercentOutput = false;
        }

        // Snap deployed position to the hard stop once the intake has traveled far enough.
        if (!usePercentOutput && !deployedPositionCalibrated && initialDeployEnabled && isDeployed()) {
            boolean travelMet = Math.abs(inputs.pivotAbsEncoderPosition - deployStartPosition) >= Constants.IntakeConstants.kPivotDeployedTravelThreshold;
            boolean spiked = deployCurrentDebouncer.calculate(
                travelMet && inputs.pivotCurrentAmps > Constants.IntakeConstants.kPivotDeployedCurrentThreshold);
            if (spiked) {
                calibratedDeployedPosition = Math.max(kMinPosition, Math.min(kMaxPosition, inputs.pivotAbsEncoderPosition));
                targetPivotPosition = calibratedDeployedPosition;
                hasSetpoint = true;
                deployedPositionCalibrated = true;
                initialDeployEnabled = false;
            }
        }

        if (!usePercentOutput && hasSetpoint) {
            io.setPivotSetpoint(targetPivotPosition);
        }

        // Roller anti-jam state machine — runs before spike detection so that reverse-phase
        // current cannot trigger false fuel-pickup events.
        if (rollerRunning) {
            boolean jammed = inputs.rollerCurrentAmps > Constants.IntakeConstants.kRollerJamCurrentThreshold
                && Math.abs(inputs.rollerVelocityRPM) < Constants.IntakeConstants.kRollerJamVelocityThresholdRPM;
            boolean free = inputs.rollerVelocityRPM > Constants.IntakeConstants.kRollerFreeVelocityThresholdRPM;

            Logger.recordOutput("Intake/RollerUnjamState",
                rollerUnjamState == ROLLER_RUNNING ? "RUNNING"
                    : rollerUnjamState == ROLLER_REVERSING ? "REVERSING" : "RECOVERING");

            if (rollerUnjamState == ROLLER_RUNNING) {
                io.setRollerPercentOutput(0.9);
                if (jammed) {
                    io.setRollerPercentOutput(-0.9);
                    rollerUnjamTimer.restart();
                    rollerUnjamState = ROLLER_REVERSING;
                }
            } else if (rollerUnjamState == ROLLER_REVERSING) {
                if (rollerUnjamTimer.hasElapsed(Constants.IntakeConstants.kRollerUnjamReverseSeconds)) {
                    io.setRollerPercentOutput(0.9);
                    rollerUnjamState = ROLLER_RECOVERING;
                }
            } else { // RECOVERING
                if (free) {
                    rollerUnjamState = ROLLER_RUNNING;
                } else if (jammed) {
                    io.setRollerPercentOutput(-0.9);
                    rollerUnjamTimer.restart();
                    rollerUnjamState = ROLLER_REVERSING;
                }
            }
        } else {
            rollerUnjamState = ROLLER_RUNNING; // reset on stop so next run starts clean
        }

        // Detect rising edges of roller current to count fuel pickups.
        // Gated on ROLLER_RUNNING so reverse-phase current cannot trigger false pickup events.
        boolean aboveThreshold = rollerRunning && rollerUnjamState == ROLLER_RUNNING
            && inputs.rollerCurrentAmps > Constants.IntakeConstants.kRollerLoadCurrentThreshold;
        if (aboveThreshold && !lastRollerAboveThreshold) {
            rollerSpikeCount++;
            rollerJustSpiked = true;
            rollerNoLoadTimer.restart();
        }
        lastRollerAboveThreshold = aboveThreshold;

        // Cut off rollers if no load spike seen within the timeout window
        if (rollerRunning && rollerNoLoadTimer.hasElapsed(Constants.IntakeConstants.kRollerNoLoadTimeoutSeconds)) {
            stopRoller();
            rollerRunning = false;
        }
        LoggedTracer.record("Intake");
    }

    public void setPivotPosition(Position position) {
        if (matchStowLocked) return;
        usePercentOutput = false;
        if (position == Position.DEPLOYED) {
            io.setPivotCoastMode(true);
            deployedPositionCalibrated = false;
            deployStartPosition = inputs.pivotAbsEncoderPosition;
            deployCurrentDebouncer.calculate(false); // reset debouncer state
            // Always run current-spike calibration on the very first deploy (before any
            // match-start trigger fires), so the hard-stop zero is captured regardless of
            // which code path initiates the deploy.
            if (Double.isNaN(calibratedDeployedPosition)) {
                initialDeployEnabled = true;
            }
            targetPivotPosition = !Double.isNaN(calibratedDeployedPosition)
                ? calibratedDeployedPosition
                : Math.max(kMinPosition, Math.min(kMaxPosition, position.value));
            hasSetpoint = true;
        } else if (position == Position.STOWED) {
            io.setPivotCoastMode(false);
            targetPivotPosition = Math.max(kMinPosition, Math.min(kMaxPosition, position.value));
            hasSetpoint = true;
        }
    }

    /**
     * Deploys the intake and enables current-spike calibration for this deploy.
     * Called from the automatic match-start deploy trigger. Note: calibration also
     * runs automatically on the very first deploy after code startup.
     */
    public void setInitialDeployPosition() {
        initialDeployEnabled = true;
        setPivotPosition(Position.DEPLOYED);
    }

    /** Forces brake mode regardless of current position — call at autonomous start. */
    public void enforceBrakeMode() {
        io.setPivotCoastMode(false);
    }

    /**
     * Reads the current absolute encoder position and sets it as the PID target.
     * If {@link Constants#kStowIntakeForMatch} is enabled, also locks the pivot.
     */
    public void lockCurrentPositionAsStow() {
        double currentPos = inputs.pivotAbsEncoderPosition;
        targetPivotPosition = Math.max(kMinPosition, Math.min(kMaxPosition, currentPos));
        if (Math.abs(currentPos - Constants.IntakeConstants.kStowedPosition) <= kStowCalibrationTolerance) {
            calibratedStowedPosition = currentPos;
        }
        hasSetpoint = true;
        usePercentOutput = false;
        matchStowLocked = Constants.IntakeConstants.kStowIntakeForMatch;
    }

    public void setPivotPercentOutput(double percentOutput) {
        usePercentOutput = true;
        io.setPivotPercentOutput(percentOutput);
    }

    public void setRollerSpeed(double rpm) {
        io.setRollerRPM(rpm);
        rollerNoLoadTimer.restart();
    }

    public void stopRoller() {
        io.stopRoller();
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
        rollerJustSpiked = false;
    }

    /**
     * Returns {@code true} once per roller current spike (rising edge of the load
     * threshold crossing) and immediately clears the flag.  Intended to be polled
     * by a {@link edu.wpi.first.wpilibj2.command.button.Trigger} in
     * {@code RobotContainer} so each fuel pickup produces exactly one rumble event.
     */
    public boolean consumeRollerSpike() {
        boolean spiked = rollerJustSpiked;
        rollerJustSpiked = false;
        return spiked;
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
     * Linearly interpolates between the calibrated deployed and stowed positions.
     * {@code percent = 0} → deployed hard-stop, {@code percent = 1} → stowed.
     * Falls back to the nominal constants if either endpoint hasn't been calibrated yet.
     */
    private double agitatePosition(double percent) {
        double deployed = Double.isNaN(calibratedDeployedPosition)
            ? Position.DEPLOYED.value : calibratedDeployedPosition;
        double stowed = Double.isNaN(calibratedStowedPosition)
            ? Position.STOWED.value : calibratedStowedPosition;
        return deployed + percent * (stowed - deployed);
    }

    /**
     * Sinusoidally oscillates the intake pivot between the low and high agitation positions.
     * Reduces the pivot output range to {@code kAgitateOutputRangeMax} for the duration and
     * restores the normal range when interrupted.
     */
    public Command agitateCommand() {
        return runOnce(() -> io.setPivotOutputRange(
                -Constants.IntakeConstants.kAgitateOutputRangeMax,
                 Constants.IntakeConstants.kAgitateOutputRangeMax))
            .andThen(run(() -> {
                double phase = (Timer.getFPGATimestamp() % Constants.IntakeConstants.kAgitatePeriodSeconds)
                    / Constants.IntakeConstants.kAgitatePeriodSeconds;
                double alpha = (1.0 - Math.cos(2.0 * Math.PI * phase)) / 2.0;
                usePercentOutput = false;
                targetPivotPosition = agitatePosition(
                    Constants.IntakeConstants.kAgitateLowPercent
                    + alpha * (Constants.IntakeConstants.kAgitateHighPercent - Constants.IntakeConstants.kAgitateLowPercent));
                hasSetpoint = true;
            }))
            .finallyDo(() -> io.setPivotOutputRange(
                Constants.IntakeConstants.kPivotOutputRangeMin,
                Constants.IntakeConstants.kPivotOutputRangeMax));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Abs Encoder Position", () -> inputs.pivotAbsEncoderPosition, null);
        builder.addDoubleProperty("Rel Encoder Position", () -> inputs.pivotRelEncoderPosition, null);
        builder.addDoubleProperty("Target Position", () -> targetPivotPosition, null);
        builder.addDoubleProperty("Pivot Current (A)", () -> inputs.pivotCurrentAmps, null);
        builder.addDoubleProperty("Roller RPM", () -> inputs.rollerVelocityRPM, null);
        builder.addDoubleProperty("Roller Current (A)", () -> inputs.rollerCurrentAmps, null);
    }
}
