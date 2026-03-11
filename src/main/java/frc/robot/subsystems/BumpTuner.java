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
 * Live-tunable bump detection parameters backed by permanent storage on the Pi.
 *
 * <h2>How it works</h2>
 * <ol>
 *   <li>On every boot the Pi runs {@code bump_tuner.py}, which loads
 *       {@code /home/pi/bump_tuner.json} (creating it with defaults on first
 *       run) and publishes all values to the {@code BumpTuner/Params/} NT table.
 *   </li>
 *   <li>This Java subsystem reads those entries with each {@link Constants}
 *       value as a fallback — so the robot works correctly even when the Pi is
 *       not connected.
 *   </li>
 *   <li>Engineers adjust values live from Elastic or Shuffleboard, then call
 *       {@link #saveToPi()} (or press the SmartDashboard "Save Bump Config"
 *       button) to persist the new values to the Pi's JSON file.
 *   </li>
 * </ol>
 *
 * <h2>Usage</h2>
 * Construct before {@link Swerve}, then pass the instance to the Swerve
 * constructor.  Replace every {@code Constants.BumpDetectionConstants.kX}
 * reference with the equivalent getter:
 *
 * <pre>
 *   bumpTuner.getBumpPitchThresholdDeg()
 *   bumpTuner.getWheelSlipThresholdMps2()
 *   // etc.
 * </pre>
 */
public class BumpTuner extends SubsystemBase {

    // ── Staleness threshold ────────────────────────────────────────────────────
    /** Cycles without a heartbeat change before the Pi is considered offline (500 ms). */
    private static final int STALE_THRESHOLD = 25;

    // ── NetworkTables ──────────────────────────────────────────────────────────
    private final NetworkTable nt;

    // Parameter entries — each is a read/write entry so Elastic can modify them
    // and the Java code reads the current (possibly Pi-updated) value each cycle.
    private final DoubleEntry eBumpPitch;
    private final DoubleEntry eBumpAccelZ;
    private final DoubleEntry eLLAccelZ;
    private final DoubleEntry eSlipThreshold;
    private final DoubleEntry eHighConf;
    private final DoubleEntry eSlipHighMult;
    private final DoubleEntry eBumpLowMult;

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
    private double cachedBumpPitch;
    private double cachedBumpAccelZ;
    private double cachedLLAccelZ;
    private double cachedSlipThreshold;
    private double cachedHighConf;
    private double cachedSlipHighMult;
    private double cachedBumpLowMult;

    // ─────────────────────────────────────────────────────────────────────────

    public BumpTuner() {
        nt = NetworkTableInstance.getDefault().getTable("BumpTuner");

        NetworkTable params  = nt.getSubTable("Params");
        NetworkTable cmd     = nt.getSubTable("Cmd");
        NetworkTable status  = nt.getSubTable("Status");

        // Parameter entries — defaults come from Constants so the robot works
        // correctly if the Pi hasn't published yet or is not connected.
        eBumpPitch     = params.getDoubleTopic("BumpPitchThresholdDeg")
                               .getEntry(Constants.BumpDetectionConstants.kBumpPitchThresholdDegrees);
        eBumpAccelZ    = params.getDoubleTopic("BumpAccelZDeviation")
                               .getEntry(Constants.BumpDetectionConstants.kBumpAccelZDeviationThreshold);
        eLLAccelZ      = params.getDoubleTopic("LimelightAccelZDeviation")
                               .getEntry(Constants.BumpDetectionConstants.kLimelightAccelZDeviationThreshold);
        eSlipThreshold = params.getDoubleTopic("WheelSlipThresholdMps2")
                               .getEntry(Constants.BumpDetectionConstants.kWheelSlipDetectionThresholdMps2);
        eHighConf      = params.getDoubleTopic("HighConfidenceThreshold")
                               .getEntry(Constants.BumpDetectionConstants.kHighConfidenceThreshold);
        eSlipHighMult  = params.getDoubleTopic("SlipHighConfMultiplier")
                               .getEntry(Constants.BumpDetectionConstants.kSlipHighConfStdDevMultiplier);
        eBumpLowMult   = params.getDoubleTopic("BumpVisionMultiplier")
                               .getEntry(Constants.BumpDetectionConstants.kBumpVisionStdDevMultiplier);

        eSaveCmd   = cmd.getBooleanTopic("Save").getEntry(false);
        eSavedOk   = status.getBooleanTopic("SavedOk").subscribe(false);
        eHeartbeat = status.getIntegerTopic("Heartbeat").subscribe(Long.MIN_VALUE);
        eConfigFile = status.getStringTopic("ConfigFile").subscribe("(not connected)");

        // Seed the frozen cache with defaults so the robot is safe before the
        // first disable snapshot (e.g., during the first-boot boot sequence).
        snapshotCache();

        SmartDashboard.putData(this);

        // SmartDashboard button — press to persist the current NT values to the Pi.
        SmartDashboard.putData("Save Bump Config to Pi",
            runOnce(this::saveToPi).withName("SaveBumpConfig").ignoringDisable(true));
    }

    /** Copies all live NT values into the frozen cache. Called on every disable. */
    private void snapshotCache() {
        cachedBumpPitch     = eBumpPitch.get();
        cachedBumpAccelZ    = eBumpAccelZ.get();
        cachedLLAccelZ      = eLLAccelZ.get();
        cachedSlipThreshold = eSlipThreshold.get();
        cachedHighConf      = eHighConf.get();
        cachedSlipHighMult  = eSlipHighMult.get();
        cachedBumpLowMult   = eBumpLowMult.get();
    }

    // ── Parameter getters ──────────────────────────────────────────────────────
    // Each getter returns the live NT value.  When the Pi is connected this
    // reflects any changes made in Elastic; when the Pi is offline it returns
    // the default supplied in the constructor (mirrors Constants).

    // ── Parameter getters ──────────────────────────────────────────────────────
    // All getters read from the frozen cache, which is snapshotted on every
    // robot disable.  This means dashboard edits take effect at the next disable,
    // never mid-match.

    /** Robot pitch threshold (degrees) above which a bump is considered active. */
    public double getBumpPitchThresholdDeg()    { return cachedBumpPitch; }

    /** Pigeon 2 Z-axis deviation (g) threshold for bump detection. */
    public double getBumpAccelZDeviation()       { return cachedBumpAccelZ; }

    /** Limelight Z-axis deviation (g) threshold for independent bump confirmation. */
    public double getLimelightAccelZDeviation()  { return cachedLLAccelZ; }

    /** Encoder-vs-IMU acceleration mismatch (m/s²) threshold to flag wheel slip. */
    public double getWheelSlipThresholdMps2()    { return cachedSlipThreshold; }

    /** Minimum vision confidence (avgTagArea × tagCount) to trust Limelight during slip. */
    public double getHighConfidenceThreshold()   { return cachedHighConf; }

    /** stdDev multiplier applied when slipping AND confidence is HIGH (&lt; 1.0 = trust more). */
    public double getSlipHighConfMultiplier()     { return cachedSlipHighMult; }

    /** stdDev multiplier applied when slipping AND confidence is LOW (&gt; 1.0 = trust less). */
    public double getBumpVisionMultiplier()       { return cachedBumpLowMult; }

    // ── Save command ──────────────────────────────────────────────────────────

    /**
     * Signals the Pi to persist the current NetworkTable parameter values to
     * {@code /home/pi/bump_tuner.json}.
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
        builder.addBooleanProperty("Pi Connected",                    this::isPiConnected,              null);
        builder.addBooleanProperty("Saved OK",                        this::isSavedOk,                  null);
        builder.addStringProperty("Config File",                      eConfigFile::get,                 null);

        // "Live" entries reflect the current NT value — edit these in Elastic.
        // "Active" entries show what the robot is actually using (frozen at last disable).
        builder.addDoubleProperty("Live/Bump Pitch Threshold (deg)",  eBumpPitch::get,                  null);
        builder.addDoubleProperty("Live/Bump AccelZ Deviation (g)",   eBumpAccelZ::get,                 null);
        builder.addDoubleProperty("Live/LL AccelZ Deviation (g)",     eLLAccelZ::get,                   null);
        builder.addDoubleProperty("Live/Wheel Slip Threshold (m/s²)", eSlipThreshold::get,              null);
        builder.addDoubleProperty("Live/High Conf Threshold",         eHighConf::get,                   null);
        builder.addDoubleProperty("Live/Slip High Conf Multiplier",   eSlipHighMult::get,               null);
        builder.addDoubleProperty("Live/Bump Vision Multiplier",      eBumpLowMult::get,                null);

        builder.addDoubleProperty("Active/Bump Pitch Threshold (deg)",  this::getBumpPitchThresholdDeg,  null);
        builder.addDoubleProperty("Active/Bump AccelZ Deviation (g)",   this::getBumpAccelZDeviation,    null);
        builder.addDoubleProperty("Active/LL AccelZ Deviation (g)",     this::getLimelightAccelZDeviation, null);
        builder.addDoubleProperty("Active/Wheel Slip Threshold (m/s²)", this::getWheelSlipThresholdMps2, null);
        builder.addDoubleProperty("Active/High Conf Threshold",         this::getHighConfidenceThreshold, null);
        builder.addDoubleProperty("Active/Slip High Conf Multiplier",   this::getSlipHighConfMultiplier,  null);
        builder.addDoubleProperty("Active/Bump Vision Multiplier",      this::getBumpVisionMultiplier,    null);
    }
}
