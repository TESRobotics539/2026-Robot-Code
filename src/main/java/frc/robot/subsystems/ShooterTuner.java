package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Live-tunable shooter parameters backed by permanent storage on the Pi.
 *
 * <h2>How it works</h2>
 * <ol>
 *   <li>On every boot the Pi runs {@code shooter_tuner.py}, which loads
 *       {@code /home/pi/shooter_tuner.json} (creating it with defaults on first
 *       run) and publishes all values to the {@code ShooterTuner/Params/} NT table.
 *   </li>
 *   <li>This Java subsystem reads those entries with each {@link Constants}
 *       value as a fallback — so the robot works correctly even when the Pi is
 *       not connected.
 *   </li>
 *   <li>Engineers adjust values live from Elastic or Shuffleboard, then call
 *       {@link #saveToPi()} (or press the SmartDashboard "Save Shooter Config to Pi"
 *       button) to persist the new values to the Pi's JSON file.
 *   </li>
 * </ol>
 *
 * <h2>Usage</h2>
 * Construct and pass the instance wherever shooter parameters are consumed.
 * Replace every {@code Constants.UltraShooterConstants.kX} or
 * {@code Constants.ShooterConstants.kX} reference with the equivalent getter:
 *
 * <pre>
 *   shooterTuner.getNearShotOffsetPercent()
 *   shooterTuner.getReadyTolerance()
 *   // etc.
 * </pre>
 */
public class ShooterTuner extends SubsystemBase {

    // ── Staleness threshold ────────────────────────────────────────────────────
    /** Cycles without a heartbeat change before the Pi is considered offline (500 ms). */
    private static final int STALE_THRESHOLD = 25;

    // ── NetworkTables ──────────────────────────────────────────────────────────
    private final NetworkTable nt;

    // Parameter entries — each is a read/write entry so Elastic can modify them
    // and the Java code reads the current (possibly Pi-updated) value each cycle.
    private final DoubleEntry eNearShotOffsetPercent;
    private final DoubleEntry eFarShotOffsetPercent;
    private final DoubleEntry eReadyTolerance;
    private final DoubleEntry ePreSpinFraction;
    private final DoubleEntry eShootWaitSeconds;
    private final DoubleEntry eShootReadyTimeoutSeconds;
    private final DoubleEntry eFloorFeedDelaySeconds;
    private final DoubleEntry eDumpShotFlywheelSpeed;

    // Save handshake
    private final BooleanEntry eSaveCmd;
    private final BooleanSubscriber eSavedOk;

    // Staleness tracking
    private final IntegerSubscriber eHeartbeat;
    private final StringSubscriber  eConfigFile;
    private long lastHeartbeat = Long.MIN_VALUE;
    private int  staleFrames   = 0;

    // Frozen parameter cache — snapshotted on every robot disable so that
    // accidental dashboard edits during a match never affect in-flight behavior.
    // All getters read from this cache, not directly from NT.
    private double cachedNearShotOffsetPercent;
    private double cachedFarShotOffsetPercent;
    private double cachedReadyTolerance;
    private double cachedPreSpinFraction;
    private double cachedShootWaitSeconds;
    private double cachedShootReadyTimeoutSeconds;
    private double cachedFloorFeedDelaySeconds;
    private double cachedDumpShotFlywheelSpeed;

    // ─────────────────────────────────────────────────────────────────────────

    public ShooterTuner() {
        nt = NetworkTableInstance.getDefault().getTable("ShooterTuner");

        NetworkTable params  = nt.getSubTable("Params");
        NetworkTable cmd     = nt.getSubTable("Cmd");
        NetworkTable status  = nt.getSubTable("Status");

        // Parameter entries — defaults come from Constants so the robot works
        // correctly if the Pi hasn't published yet or is not connected.
        eNearShotOffsetPercent    = params.getDoubleTopic("NearShotOffsetPercent")
                                          .getEntry(Constants.UltraShooterConstants.kNearShotOffsetPercent);
        eFarShotOffsetPercent     = params.getDoubleTopic("FarShotOffsetPercent")
                                          .getEntry(Constants.UltraShooterConstants.kFarShotOffsetPercent);
        eReadyTolerance           = params.getDoubleTopic("ReadyTolerance")
                                          .getEntry(Constants.UltraShooterConstants.kReadyTolerance);
        ePreSpinFraction          = params.getDoubleTopic("PreSpinFraction")
                                          .getEntry(Constants.UltraShooterConstants.kPreSpinFraction);
        eShootWaitSeconds         = params.getDoubleTopic("ShootWaitSeconds")
                                          .getEntry(Constants.ShooterConstants.kShootWaitSeconds);
        eShootReadyTimeoutSeconds = params.getDoubleTopic("ShootReadyTimeoutSeconds")
                                          .getEntry(Constants.ShooterConstants.kShootReadyTimeoutSeconds);
        eFloorFeedDelaySeconds    = params.getDoubleTopic("FloorFeedDelaySeconds")
                                          .getEntry(Constants.ShooterConstants.kFloorFeedDelaySeconds);
        eDumpShotFlywheelSpeed    = params.getDoubleTopic("DumpShotFlywheelSpeed")
                                          .getEntry(Constants.ShooterConstants.kDumpShotFlywheelSpeed);

        eSaveCmd    = cmd.getBooleanTopic("Save").getEntry(false);
        eSavedOk    = status.getBooleanTopic("SavedOk").subscribe(false);
        eHeartbeat  = status.getIntegerTopic("Heartbeat").subscribe(Long.MIN_VALUE);
        eConfigFile = status.getStringTopic("ConfigFile").subscribe("(not connected)");

        // Seed the frozen cache with defaults so the robot is safe before the
        // first disable snapshot (e.g., during the first-boot boot sequence).
        snapshotCache();

        SmartDashboard.putData(this);

        // SmartDashboard button — press to persist the current NT values to the Pi.
        SmartDashboard.putData("Save Shooter Config to Pi",
            runOnce(this::saveToPi).withName("SaveShooterConfig").ignoringDisable(true));
    }

    /** Copies all live NT values into the frozen cache. Called on every disable. */
    private void snapshotCache() {
        cachedNearShotOffsetPercent    = eNearShotOffsetPercent.get();
        cachedFarShotOffsetPercent     = eFarShotOffsetPercent.get();
        cachedReadyTolerance           = eReadyTolerance.get();
        cachedPreSpinFraction          = ePreSpinFraction.get();
        cachedShootWaitSeconds         = eShootWaitSeconds.get();
        cachedShootReadyTimeoutSeconds = eShootReadyTimeoutSeconds.get();
        cachedFloorFeedDelaySeconds    = eFloorFeedDelaySeconds.get();
        cachedDumpShotFlywheelSpeed    = eDumpShotFlywheelSpeed.get();
    }

    // ── Parameter getters ──────────────────────────────────────────────────────
    // All getters read from the frozen cache, which is snapshotted on every
    // robot disable.  This means dashboard edits take effect at the next disable,
    // never mid-match.

    /** Percent offset applied to flywheel speed for near shots. */
    public double getNearShotOffsetPercent()    { return cachedNearShotOffsetPercent; }

    /** Percent offset applied to flywheel speed for far shots. */
    public double getFarShotOffsetPercent()     { return cachedFarShotOffsetPercent; }

    /** ft/s tolerance band around target speed to consider the shooter ready. */
    public double getReadyTolerance()           { return cachedReadyTolerance; }

    /** Fraction of target speed to spin up to while waiting for a shot command. */
    public double getPreSpinFraction()          { return cachedPreSpinFraction; }

    /** Seconds to wait after shoot command before advancing the feeder. */
    public double getShootWaitSeconds()         { return cachedShootWaitSeconds; }

    /** Seconds to wait for the shooter to reach ready speed before giving up. */
    public double getShootReadyTimeoutSeconds() { return cachedShootReadyTimeoutSeconds; }

    /** Seconds to delay floor roller engagement after the feeder starts. */
    public double getFloorFeedDelaySeconds()    { return cachedFloorFeedDelaySeconds; }

    /** ft/s flywheel target speed for dump shots. */
    public double getDumpShotFlywheelSpeed()    { return cachedDumpShotFlywheelSpeed; }

    // ── Save command ──────────────────────────────────────────────────────────

    /**
     * Signals the Pi to persist the current NetworkTable parameter values to
     * {@code /home/pi/shooter_tuner.json}.
     *
     * <p>The Pi resets the flag and sets {@code Status/SavedOk} once the file
     * write completes.  Check {@link #isSavedOk()} to confirm success.
     */
    public void saveToPi() {
        eSaveCmd.set(true);
    }

    /** Returns {@code true} if the Pi acknowledged the most recent save as successful. */
    public boolean isSavedOk() {
        return eSavedOk.get();
    }

    // ── Connection status ─────────────────────────────────────────────────────

    /** Returns {@code true} when the Pi heartbeat is fresh (within the last 500 ms). */
    public boolean isPiConnected() {
        return staleFrames < STALE_THRESHOLD;
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        long hb = eHeartbeat.get();
        if (hb != lastHeartbeat) {
            lastHeartbeat = hb;
            staleFrames   = 0;
        } else {
            staleFrames++;
        }

        // Snapshot live NT values into the frozen cache whenever the robot is
        // disabled.  Changes made in Elastic take effect at the next disable,
        // not immediately, so accidental mid-match edits are harmless.
        if (DriverStation.isDisabled()) {
            snapshotCache();
        }
    }

    // ── Sendable ──────────────────────────────────────────────────────────────

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Pi Connected",  this::isPiConnected, null);
        builder.addBooleanProperty("Saved OK",      this::isSavedOk,     null);
        builder.addStringProperty("Config File",    eConfigFile::get,    null);

        // "Live" entries reflect the current NT value — edit these in Elastic.
        // "Active" entries show what the robot is actually using (frozen at last disable).
        builder.addDoubleProperty("Live/Near Shot Offset (%)",      eNearShotOffsetPercent::get,    null);
        builder.addDoubleProperty("Live/Far Shot Offset (%)",       eFarShotOffsetPercent::get,     null);
        builder.addDoubleProperty("Live/Ready Tolerance (ft/s)",    eReadyTolerance::get,           null);
        builder.addDoubleProperty("Live/Pre-Spin Fraction",         ePreSpinFraction::get,          null);
        builder.addDoubleProperty("Live/Shoot Wait (s)",            eShootWaitSeconds::get,         null);
        builder.addDoubleProperty("Live/Shoot Ready Timeout (s)",   eShootReadyTimeoutSeconds::get, null);
        builder.addDoubleProperty("Live/Floor Feed Delay (s)",      eFloorFeedDelaySeconds::get,    null);
        builder.addDoubleProperty("Live/Dump Shot Speed (ft/s)",    eDumpShotFlywheelSpeed::get,    null);

        builder.addDoubleProperty("Active/Near Shot Offset (%)",    this::getNearShotOffsetPercent,    null);
        builder.addDoubleProperty("Active/Far Shot Offset (%)",     this::getFarShotOffsetPercent,     null);
        builder.addDoubleProperty("Active/Ready Tolerance (ft/s)",  this::getReadyTolerance,           null);
        builder.addDoubleProperty("Active/Pre-Spin Fraction",       this::getPreSpinFraction,          null);
        builder.addDoubleProperty("Active/Shoot Wait (s)",          this::getShootWaitSeconds,         null);
        builder.addDoubleProperty("Active/Shoot Ready Timeout (s)", this::getShootReadyTimeoutSeconds, null);
        builder.addDoubleProperty("Active/Floor Feed Delay (s)",    this::getFloorFeedDelaySeconds,    null);
        builder.addDoubleProperty("Active/Dump Shot Speed (ft/s)",  this::getDumpShotFlywheelSpeed,    null);
    }
}
