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
 *   shooterTuner.getCloseShotOffsetPercent()
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
    private final DoubleEntry eCloseShotOffsetPercent;
    private final DoubleEntry eMidShotOffsetPercent;
    private final DoubleEntry eFarShotOffsetPercent;
    private final DoubleEntry eReadyTolerance;
    private final DoubleEntry eShootReadyTimeoutSeconds;
    private final DoubleEntry eShootWaitSeconds;
    private final DoubleEntry eFloorFeedDelaySeconds;
    private final DoubleEntry eKp;
    private final DoubleEntry eFlywheelEfficiency;
    private final DoubleEntry eDragCoefficient;
    private final DoubleEntry eBallMassLbs;

    // Live-tuning toggle — when true the frozen cache is refreshed every cycle so
    // dashboard edits take effect immediately during a match.
    // This entry is intentionally NOT subject to the frozen cache itself.
    private final BooleanEntry eLiveTuningEnabled;

    // Save handshake
    private final BooleanEntry eSaveCmd;
    private final BooleanSubscriber eSavedOk;

    // Staleness tracking
    private final IntegerSubscriber eHeartbeat;
    private final StringSubscriber  eConfigFile;
    private long lastHeartbeat = Long.MIN_VALUE;
    private int  staleFrames   = STALE_THRESHOLD;

    // Frozen parameter cache — snapshotted on every robot disable so that
    // accidental dashboard edits during a match never affect in-flight behavior.
    // All getters read from this cache, not directly from NT.
    private double cachedCloseShotOffsetPercent;
    private double cachedMidShotOffsetPercent;
    private double cachedFarShotOffsetPercent;
    private double cachedReadyTolerance;
    private double cachedShootReadyTimeoutSeconds;
    private double cachedShootWaitSeconds;
    private double cachedFloorFeedDelaySeconds;
    private double cachedKp;
    private double cachedFlywheelEfficiency;
    private double cachedDragCoefficient;
    private double cachedBallMassLbs;

    // ─────────────────────────────────────────────────────────────────────────

    public ShooterTuner() {
        nt = NetworkTableInstance.getDefault().getTable("ShooterTuner");

        NetworkTable params  = nt.getSubTable("Params");
        NetworkTable cmd     = nt.getSubTable("Cmd");
        NetworkTable status  = nt.getSubTable("Status");

        // Parameter entries — defaults come from Constants so the robot works
        // correctly if the Pi hasn't published yet or is not connected.
        eCloseShotOffsetPercent   = params.getDoubleTopic("CloseShotOffsetPercent")
                                          .getEntry(Constants.UltraShooterConstants.kCloseShotOffsetPercent);
        eMidShotOffsetPercent     = params.getDoubleTopic("MidShotOffsetPercent")
                                          .getEntry(Constants.UltraShooterConstants.kMidShotOffsetPercent);
        eFarShotOffsetPercent     = params.getDoubleTopic("FarShotOffsetPercent")
                                          .getEntry(Constants.UltraShooterConstants.kFarShotOffsetPercent);
        eReadyTolerance           = params.getDoubleTopic("ReadyTolerance")
                                          .getEntry(Constants.UltraShooterConstants.kReadyTolerance);
        eShootReadyTimeoutSeconds = params.getDoubleTopic("ShootReadyTimeoutSeconds")
                                          .getEntry(Constants.ShooterConstants.kShootReadyTimeoutSeconds);
        eShootWaitSeconds         = params.getDoubleTopic("ShootWaitSeconds")
                                          .getEntry(Constants.ShooterConstants.kShootWaitSeconds);
        eFloorFeedDelaySeconds    = params.getDoubleTopic("FloorFeedDelaySeconds")
                                          .getEntry(Constants.ShooterConstants.kFloorFeedDelaySeconds);
        eKp                       = params.getDoubleTopic("Kp")
                                          .getEntry(Constants.UltraShooterConstants.kP);
        eFlywheelEfficiency       = params.getDoubleTopic("FlywheelEfficiency")
                                          .getEntry(Constants.UltraShooterConstants.kFlywheelEfficiency);
        eDragCoefficient          = params.getDoubleTopic("DragCoefficient")
                                          .getEntry(Constants.UltraShooterConstants.kDragCoefficient);
        eBallMassLbs              = params.getDoubleTopic("BallMassLbs")
                                          .getEntry(Constants.UltraShooterConstants.kBallMassLbs);
        eLiveTuningEnabled        = params.getBooleanTopic("LiveTuningEnabled")
                                          .getEntry(false);

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

    /** Copies all live NT values into the frozen cache. Called on every disable (and every
     *  cycle when {@code LiveTuningEnabled} is set in the dashboard). */
    private void snapshotCache() {
        cachedCloseShotOffsetPercent   = eCloseShotOffsetPercent.get();
        cachedMidShotOffsetPercent     = eMidShotOffsetPercent.get();
        cachedFarShotOffsetPercent     = eFarShotOffsetPercent.get();
        cachedReadyTolerance           = eReadyTolerance.get();
        cachedShootReadyTimeoutSeconds = eShootReadyTimeoutSeconds.get();
        cachedShootWaitSeconds         = eShootWaitSeconds.get();
        cachedFloorFeedDelaySeconds    = eFloorFeedDelaySeconds.get();
        cachedKp                       = eKp.get();
        cachedFlywheelEfficiency       = eFlywheelEfficiency.get();
        cachedDragCoefficient          = eDragCoefficient.get();
        cachedBallMassLbs              = eBallMassLbs.get();
    }

    // ── Parameter getters ──────────────────────────────────────────────────────
    // All getters read from the frozen cache, which is snapshotted on every
    // robot disable.  This means dashboard edits take effect at the next disable,
    // never mid-match.

    /** Percent offset at the close anchor (~3 ft). */
    public double getCloseShotOffsetPercent()   { return cachedCloseShotOffsetPercent; }

    /** Percent offset at the mid anchor (~13 ft). */
    public double getMidShotOffsetPercent()     { return cachedMidShotOffsetPercent; }

    /** Percent offset at the far anchor (~23 ft). */
    public double getFarShotOffsetPercent()     { return cachedFarShotOffsetPercent; }

    /** ft/s tolerance band around target speed to consider the shooter ready. */
    public double getReadyTolerance()           { return cachedReadyTolerance; }

    /** Seconds to wait for the shooter to reach ready speed before giving up. */
    public double getShootReadyTimeoutSeconds() { return cachedShootReadyTimeoutSeconds; }

    /** Seconds to wait after reaching ready speed before releasing the feeder. */
    public double getShootWaitSeconds()         { return cachedShootWaitSeconds; }

    /** Seconds to delay floor roller engagement after the feeder starts. */
    public double getFloorFeedDelaySeconds()    { return cachedFloorFeedDelaySeconds; }

    /**
     * Active (frozen) kP — what the robot is currently using.
     * Snapshotted from NT on every disable; applied to motors by {@link UltraShooter}.
     */
    public double getKp()                       { return cachedKp; }

    /**
     * Live NT kP — reflects any autotuner updates made during this session.
     * Use this to read the autotuner's current recommendation before the next disable.
     */
    public double getLiveKp()                   { return eKp.get(); }

    /** Active flywheel efficiency fraction (0–1). Frozen at last disable (or live if enabled). */
    public double getFlywheelEfficiency()        { return cachedFlywheelEfficiency; }

    /** Active aerodynamic drag constant B (kg/m). Set to 0 to disable drag compensation. */
    public double getDragCoefficient()           { return cachedDragCoefficient; }

    /** Active ball mass (lbs). Used in drag deceleration term B/m (converted to kg internally). */
    public double getBallMassLbs()               { return cachedBallMassLbs; }

    /** Returns {@code true} when live-tuning mode is active (dashboard edits take effect immediately). */
    public boolean isLiveTuningEnabled()         { return eLiveTuningEnabled.get(); }

    /**
     * Writes a new kP to the live NT entry.  Called by {@link ShooterAutoTuner}
     * after each shot analysis.  The value becomes active at the next disable.
     */
    public void setLiveKp(double kp)            { eKp.set(kp); }

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
        // disabled OR when LiveTuningEnabled is set from the dashboard.
        // Live-tuning mode lets engineers adjust shot parameters mid-match in Elastic;
        // leave it off in competition to prevent accidental edits.
        if (DriverStation.isDisabled() || eLiveTuningEnabled.get()) {
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
        builder.addBooleanProperty("Live Tuning Enabled",              eLiveTuningEnabled::get,
                                                                       eLiveTuningEnabled::set);

        builder.addDoubleProperty("Live/Close Shot Offset (%)",     eCloseShotOffsetPercent::get,   null);
        builder.addDoubleProperty("Live/Mid Shot Offset (%)",       eMidShotOffsetPercent::get,     null);
        builder.addDoubleProperty("Live/Far Shot Offset (%)",       eFarShotOffsetPercent::get,     null);
        builder.addDoubleProperty("Live/Ready Tolerance (ft/s)",    eReadyTolerance::get,           null);
        builder.addDoubleProperty("Live/Shoot Ready Timeout (s)",   eShootReadyTimeoutSeconds::get, null);
        builder.addDoubleProperty("Live/Shoot Wait (s)",            eShootWaitSeconds::get,         null);
        builder.addDoubleProperty("Live/Floor Feed Delay (s)",      eFloorFeedDelaySeconds::get,    null);
        builder.addDoubleProperty("Live/kP",                        eKp::get,                       null);
        builder.addDoubleProperty("Live/Flywheel Efficiency",       eFlywheelEfficiency::get,       null);
        builder.addDoubleProperty("Live/Drag Coefficient (kg/m)",   eDragCoefficient::get,          null);
        builder.addDoubleProperty("Live/Ball Mass (lbs)",           eBallMassLbs::get,              null);

        builder.addDoubleProperty("Active/Close Shot Offset (%)",   this::getCloseShotOffsetPercent,   null);
        builder.addDoubleProperty("Active/Mid Shot Offset (%)",     this::getMidShotOffsetPercent,     null);
        builder.addDoubleProperty("Active/Far Shot Offset (%)",     this::getFarShotOffsetPercent,     null);
        builder.addDoubleProperty("Active/Ready Tolerance (ft/s)",  this::getReadyTolerance,           null);
        builder.addDoubleProperty("Active/Shoot Ready Timeout (s)", this::getShootReadyTimeoutSeconds, null);
        builder.addDoubleProperty("Active/Shoot Wait (s)",          this::getShootWaitSeconds,         null);
        builder.addDoubleProperty("Active/Floor Feed Delay (s)",    this::getFloorFeedDelaySeconds,    null);
        builder.addDoubleProperty("Active/kP",                      this::getKp,                       null);
        builder.addDoubleProperty("Active/Flywheel Efficiency",     this::getFlywheelEfficiency,       null);
        builder.addDoubleProperty("Active/Drag Coefficient (kg/m)", this::getDragCoefficient,          null);
        builder.addDoubleProperty("Active/Ball Mass (lbs)",         this::getBallMassLbs,              null);
    }
}
