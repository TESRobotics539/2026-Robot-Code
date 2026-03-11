package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Automatic kP tuner for the flywheel shooter.
 *
 * <h2>What it does</h2>
 * <p>Follows the WPILib flywheel tuning guide:
 * kV (feedforward) handles steady-state; kP (proportional feedback) corrects
 * disturbances.  This subsystem monitors each shot's velocity dip and
 * recovery, then nudges kP up or down one small step:
 * <ul>
 *   <li>Recovery too slow (&gt; {@value #TARGET_RECOVERY_SECS} s) → increase kP</li>
 *   <li>Overshoot detected → decrease kP</li>
 *   <li>Recovery fast with no overshoot → keep kP</li>
 * </ul>
 *
 * <h2>Safety</h2>
 * The updated kP is written to the {@code ShooterTuner/Params/Kp} NT entry as
 * a <em>Live</em> value.  The frozen-cache pattern in {@link ShooterTuner}
 * ensures it only becomes active on the next robot disable — never mid-match.
 * {@link UltraShooter} detects the change during disable and re-configures the
 * SparkFlex controllers before the next enable.
 *
 * <h2>Persistence</h2>
 * Press "Save Shooter Config to Pi" in the SmartDashboard to write the new kP
 * to {@code /home/pi/shooter_tuner.json} so it survives Pi reboots.
 */
public class ShooterAutoTuner extends SubsystemBase {

    // ── Tuning knobs ──────────────────────────────────────────────────────────

    /** Velocity drop below target (ft/s) that identifies a ball passing through. */
    private static final double DIP_THRESHOLD_FPS     = 3.0;

    /** Velocity above target (ft/s) during recovery that counts as overshoot. */
    private static final double OVERSHOOT_FPS         = 1.0;

    /** Desired recovery time (s) from dip detection back to within tolerance. */
    private static final double TARGET_RECOVERY_SECS  = 0.30;

    /** kP is nudged by this amount per shot. */
    private static final double KP_STEP               = 0.0002;

    /** Lower bound — never let kP go negative. */
    private static final double KP_MIN                = 0.0;

    /** Upper bound — prevents runaway integral of proportional gain. */
    private static final double KP_MAX                = 0.020;

    /** Give up on a shot if recovery takes longer than this (s). */
    private static final double RECOVERY_TIMEOUT_SECS = 2.5;

    // ── State machine ─────────────────────────────────────────────────────────

    private enum State {
        /** Flywheel idle or not yet ready — not tracking. */
        IDLE,
        /** Flywheel is at target speed — watching for a velocity dip. */
        WATCHING,
        /** Velocity has dipped below threshold — tracking the trough. */
        DIP_ACTIVE,
        /** Velocity is climbing back — timing to full recovery. */
        RECOVERING
    }

    private State  state           = State.IDLE;
    private double dipDetectedTime = 0.0;
    private double lastTarget      = 0.0;
    private boolean overshootDetected = false;

    // ── Subsystem references ──────────────────────────────────────────────────

    private final UltraShooter ultraShooter;
    private final ShooterTuner shooterTuner;

    // ── Telemetry ─────────────────────────────────────────────────────────────

    private final NetworkTable  nt             = NetworkTableInstance.getDefault().getTable("ShooterAutoTuner");
    private final StringEntry   ntState        = nt.getStringTopic("State").getEntry("IDLE");
    private final IntegerEntry  ntShotsTotal   = nt.getIntegerTopic("Shots Analyzed").getEntry(0);
    private final DoubleEntry   ntLastRecovery = nt.getDoubleTopic("Last Recovery (ms)").getEntry(0.0);
    private final StringEntry   ntLastAction   = nt.getStringTopic("Last Action").getEntry("(no shots yet)");
    private final DoubleEntry   ntCurrentKp    = nt.getDoubleTopic("Current kP").getEntry(0.0);
    private final BooleanEntry  ntEnabled      = nt.getBooleanTopic("Autotuner Enabled").getEntry(true);

    private int shotsAnalyzed = 0;

    // ─────────────────────────────────────────────────────────────────────────

    public ShooterAutoTuner(UltraShooter ultraShooter, ShooterTuner shooterTuner) {
        this.ultraShooter = ultraShooter;
        this.shooterTuner = shooterTuner;

        ntEnabled.set(true);
        ntLastAction.set("(no shots yet)");
        ntLastRecovery.set(0.0);
        ntShotsTotal.set(0);

        SmartDashboard.putData(this);
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        boolean enabled = ntEnabled.get();

        double avgVelocity = ultraShooter.getAverageVelocity();
        double target      = ultraShooter.getTarget();
        double now         = Timer.getFPGATimestamp();
        double tolerance   = shooterTuner.getReadyTolerance();

        // Publish current kP regardless of state
        ntCurrentKp.set(shooterTuner.getLiveKp());
        ntState.set(state.name());

        // Reset if disabled, autotuner turned off, or target cleared
        if (!DriverStation.isEnabled() || !enabled || target <= 0) {
            state = State.IDLE;
            return;
        }

        // If target changed significantly mid-sequence, reset to IDLE to avoid
        // measuring a cross-target transition as if it were a shot dip.
        if (Math.abs(target - lastTarget) > DIP_THRESHOLD_FPS && state != State.IDLE) {
            state = State.IDLE;
        }
        lastTarget = target;

        switch (state) {

            case IDLE:
                if (ultraShooter.isReady()) {
                    state = State.WATCHING;
                }
                break;

            case WATCHING:
                if (!ultraShooter.isReady() && avgVelocity < target - DIP_THRESHOLD_FPS) {
                    // Dip detected — a ball is passing through
                    dipDetectedTime   = now;
                    overshootDetected = false;
                    state = State.DIP_ACTIVE;
                }
                break;

            case DIP_ACTIVE:
                if (now - dipDetectedTime > RECOVERY_TIMEOUT_SECS) {
                    // Took too long — skip this shot and reset
                    ntLastAction.set("Timeout in dip — skipped");
                    state = State.IDLE;
                    break;
                }
                // Transition to RECOVERING once velocity climbs back above the dip threshold
                if (avgVelocity >= target - DIP_THRESHOLD_FPS) {
                    state = State.RECOVERING;
                }
                break;

            case RECOVERING:
                // Track overshoot
                if (avgVelocity > target + OVERSHOOT_FPS) {
                    overshootDetected = true;
                }

                if (now - dipDetectedTime > RECOVERY_TIMEOUT_SECS) {
                    // Still not recovered — kP is far too low; bump it more aggressively
                    double kp = Math.min(KP_MAX, shooterTuner.getLiveKp() + KP_STEP * 3);
                    shooterTuner.setLiveKp(kp);
                    ntLastAction.set("kP increased ×3 (recovery timeout)");
                    ntShotsTotal.set(++shotsAnalyzed);
                    state = State.IDLE;
                    break;
                }

                if (Math.abs(avgVelocity - target) < tolerance) {
                    // Fully recovered — analyze and adjust
                    double recoveryMs = (now - dipDetectedTime) * 1000.0;
                    ntLastRecovery.set(recoveryMs);
                    ntShotsTotal.set(++shotsAnalyzed);
                    adjustKp(now - dipDetectedTime);
                    state = State.WATCHING; // watch for next shot immediately
                }
                break;
        }
    }

    // ── kP adjustment ─────────────────────────────────────────────────────────

    private void adjustKp(double recoveryTimeSecs) {
        double currentKp = shooterTuner.getLiveKp();
        double newKp     = currentKp;
        String action;

        if (overshootDetected) {
            // Oscillation → kP too high
            newKp  = Math.max(KP_MIN, currentKp - KP_STEP);
            action = String.format("kP %.5f→%.5f (overshoot)", currentKp, newKp);
        } else if (recoveryTimeSecs > TARGET_RECOVERY_SECS) {
            // Recovery too slow → kP too low
            newKp  = Math.min(KP_MAX, currentKp + KP_STEP);
            action = String.format("kP %.5f→%.5f (slow: %.0fms)", currentKp, newKp, recoveryTimeSecs * 1000);
        } else {
            action = String.format("kP OK %.5f (%.0fms)", currentKp, recoveryTimeSecs * 1000);
        }

        ntLastAction.set(action);
        if (newKp != currentKp) {
            shooterTuner.setLiveKp(newKp);
        }
    }

    // ── Sendable ──────────────────────────────────────────────────────────────

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("State",            ntState::get,            null);
        builder.addBooleanProperty("Enabled",         ntEnabled::get,          ntEnabled::set);
        builder.addDoubleProperty("Current kP",       shooterTuner::getLiveKp, null);
        builder.addDoubleProperty("Active kP",        shooterTuner::getKp,     null);
        builder.addDoubleProperty("Last Recovery(ms)",ntLastRecovery::get,     null);
        builder.addStringProperty("Last Action",      ntLastAction::get,       null);
        builder.addIntegerProperty("Shots Analyzed",  () -> shotsAnalyzed,     null);
    }
}
