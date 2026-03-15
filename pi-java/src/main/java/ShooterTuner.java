/**
 * Shooter Parameter Tuner — Team 539
 *
 * <p>Mirrors pi/shooter_tuner.py exactly. Loads saved parameters from
 * {@code /home/pi/shooter_tuner.json} on startup (writing defaults if absent),
 * publishes them to the ShooterTuner NT table, and saves whenever the robot
 * sends a save command via {@code ShooterTuner/Cmd/Save = true}.
 *
 * <p>NT table: "ShooterTuner"
 * <ul>
 *   <li>{@code Params/*}          — one Double entry per tunable parameter</li>
 *   <li>{@code Cmd/Save}          — Java sets true → Pi saves + resets to false</li>
 *   <li>{@code Status/Heartbeat}  — increments every loop</li>
 *   <li>{@code Status/SavedOk}    — true after a successful save</li>
 *   <li>{@code Status/ConfigFile} — absolute path of the active config file</li>
 * </ul>
 */
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.lang.reflect.Type;
import java.util.LinkedHashMap;
import java.util.Map;

public class ShooterTuner {

    private static final String CONFIG_FILE   = "/home/pi/shooter_tuner.json";
    private static final double LOOP_PERIOD_S = 0.02; // 50 Hz

    /**
     * Default values — mirror Java Constants.ShooterConstants /
     * Constants.UltraShooterConstants.
     */
    private static final Map<String, Double> DEFAULTS = new LinkedHashMap<>();
    static {
        DEFAULTS.put("CloseShotOffsetPercent",   0.0);
        DEFAULTS.put("MidShotOffsetPercent",     0.0);
        DEFAULTS.put("FarShotOffsetPercent",     0.0);
        DEFAULTS.put("ReadyTolerance",           1.745);  // ft/s ≈ 100 RPM
        DEFAULTS.put("ShootReadyTimeoutSeconds", 1.33);
        DEFAULTS.put("ShootWaitSeconds",         0.5);
        DEFAULTS.put("FloorFeedDelaySeconds",    0.25);
        DEFAULTS.put("Kp",                       0.003);
        DEFAULTS.put("FlywheelEfficiency",       0.43);
        DEFAULTS.put("DragCoefficient",          0.0132);
        DEFAULTS.put("BallMassLbs",              0.595);
    }

    private static final Gson GSON     = new GsonBuilder().setPrettyPrinting().create();
    private static final Type MAP_TYPE = new TypeToken<Map<String, Double>>(){}.getType();

    // ── Config helpers ────────────────────────────────────────────────────────────

    private static Map<String, Double> loadOrCreate() {
        Map<String, Double> config = new LinkedHashMap<>(DEFAULTS);
        File f = new File(CONFIG_FILE);

        if (f.exists()) {
            try (FileReader r = new FileReader(f)) {
                Map<String, Double> saved = GSON.fromJson(r, MAP_TYPE);
                if (saved != null) {
                    for (String key : DEFAULTS.keySet()) {
                        if (saved.containsKey(key)) config.put(key, saved.get(key));
                    }
                }
                System.out.println("[ShooterTuner] Loaded config from " + CONFIG_FILE);
            } catch (Exception e) {
                System.out.println("[ShooterTuner] Failed to load config (" + e.getMessage() + "), using defaults");
            }
        } else {
            System.out.println("[ShooterTuner] No config file found — writing defaults to " + CONFIG_FILE);
            save(config);
        }
        return config;
    }

    private static boolean save(Map<String, Double> config) {
        try {
            new File(CONFIG_FILE).getParentFile().mkdirs();
            try (FileWriter w = new FileWriter(CONFIG_FILE)) {
                GSON.toJson(config, w);
            }
            System.out.println("[ShooterTuner] Saved config to " + CONFIG_FILE);
            return true;
        } catch (Exception e) {
            System.out.println("[ShooterTuner] Save failed: " + e.getMessage());
            return false;
        }
    }

    // ── NT loop ───────────────────────────────────────────────────────────────────

    /**
     * Blocking 50 Hz loop. Called from {@link Main} in a daemon thread.
     *
     * @param table pre-initialized "ShooterTuner" NT table
     */
    public static void run(NetworkTable table) {
        NetworkTable paramsTable = table.getSubTable("Params");
        NetworkTable cmdTable    = table.getSubTable("Cmd");
        NetworkTable statusTable = table.getSubTable("Status");

        Map<String, Double> config = loadOrCreate();

        // Publish initial values and open subscribers for dashboard edits.
        Map<String, DoublePublisher>  pubs = new LinkedHashMap<>();
        Map<String, DoubleSubscriber> subs = new LinkedHashMap<>();

        for (Map.Entry<String, Double> e : DEFAULTS.entrySet()) {
            DoublePublisher pub = paramsTable.getDoubleTopic(e.getKey()).publish();
            pub.set(config.get(e.getKey()));
            pubs.put(e.getKey(), pub);
            subs.put(e.getKey(), paramsTable.getDoubleTopic(e.getKey()).subscribe(config.get(e.getKey())));
        }

        // Save command: Java writes true → Pi saves and resets to false.
        BooleanSubscriber saveSub = cmdTable.getBooleanTopic("Save").subscribe(false);
        BooleanPublisher  savePub = cmdTable.getBooleanTopic("Save").publish();
        savePub.set(false);

        // Status entries.
        IntegerPublisher hbPub      = statusTable.getIntegerTopic("Heartbeat").publish();
        BooleanPublisher savedOkPub = statusTable.getBooleanTopic("SavedOk").publish();
        StringPublisher  cfgFilePub = statusTable.getStringTopic("ConfigFile").publish();

        cfgFilePub.set(CONFIG_FILE);
        savedOkPub.set(new File(CONFIG_FILE).exists());

        System.out.println("[ShooterTuner] Running — listening for save commands.");

        long heartbeat = 0;
        long loopNs    = (long) (LOOP_PERIOD_S * 1_000_000_000L);

        while (true) {
            long start = System.nanoTime();

            hbPub.set(heartbeat++);

            if (saveSub.get()) {
                for (String key : DEFAULTS.keySet()) {
                    config.put(key, subs.get(key).get());
                }
                savedOkPub.set(save(config));
                savePub.set(false);
            }

            long sleep = loopNs - (System.nanoTime() - start);
            if (sleep > 0) {
                try { Thread.sleep(sleep / 1_000_000L, (int) (sleep % 1_000_000L)); }
                catch (InterruptedException e) { Thread.currentThread().interrupt(); return; }
            }
        }
    }
}
