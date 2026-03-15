/**
 * Pi Coprocessor — Team 539
 *
 * <p>Entry point for the combined physics-engine JAR. Initialises a single shared
 * NetworkTables connection, then launches all four coprocessor modules in daemon
 * threads — mirroring the structure of pi/main.py.
 *
 * <p>Modules started:
 * <ol>
 *   <li>{@link PhysicsEngine}        — 50 Hz, flywheel velocity calculator</li>
 *   <li>{@link ShooterTuner}         — 50 Hz, persistent shooter parameter tuner</li>
 *   <li>{@link BumpTuner}            — 50 Hz, persistent bump detection tuner</li>
 *   <li>{@link TrajectoryVisualizer} — 10 Hz, MJPEG shot trajectory stream</li>
 * </ol>
 *
 * <p>Build: {@code ./gradlew shadowJar} -> {@code pi-java/build/libs/pi-coprocessor-all.jar}
 * <p>Deploy: {@code scp build/libs/pi-coprocessor-all.jar pi@wpilibpi.local:/home/pi/}
 * <p>Launch: {@code java -jar /home/pi/pi-coprocessor-all.jar}
 */
import edu.wpi.first.networktables.NetworkTableInstance;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;

public class Main {

    private static final int TEAM_NUMBER = 539;

    /**
     * Extracts ARM64 WPILib native libraries from the JAR to a temp directory
     * and loads them in dependency order via {@link System#load}.
     *
     * <p>Must be called before any WPILib class is touched. Only runs on Linux
     * aarch64 (the Pi); on other platforms the JVM resolves natives normally.
     */
    private static void loadArm64Natives() throws IOException {
        String os   = System.getProperty("os.name",  "");
        String arch = System.getProperty("os.arch",  "");
        if (!os.startsWith("Linux") || !arch.equals("aarch64")) return;

        // Load in dependency order: wpiutil first, then wpinet, then ntcore.
        String[] libs = {
            "linux/arm64/shared/libwpiutil.so",
            "linux/arm64/shared/libwpinet.so",
            "linux/arm64/shared/libntcore.so",
            "linux/arm64/shared/libwpiutiljni.so",
            "linux/arm64/shared/libwpinetjni.so",
            "linux/arm64/shared/libntcorejni.so",
        };

        Path tempDir = Files.createTempDirectory("wpilib-natives-");

        for (String resource : libs) {
            String name = resource.substring(resource.lastIndexOf('/') + 1);
            Path   dest = tempDir.resolve(name);

            try (InputStream is = Main.class.getClassLoader().getResourceAsStream(resource)) {
                if (is == null) throw new IOException("Native not found in JAR: " + resource);
                Files.copy(is, dest);
            }

            System.load(dest.toAbsolutePath().toString());
            System.out.println("[Native] Loaded " + name);
        }
    }

    public static void main(String[] args) throws Exception {
        // Required for AWT (BufferedImage / Graphics2D) on a Pi without a display.
        System.setProperty("java.awt.headless", "true");

        // Extract and load WPILib ARM64 natives before any NT code runs.
        loadArm64Natives();

        // ── Single shared NT4 connection — all modules use this instance ──────────
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("frc539-pi");
        inst.setServerTeam(TEAM_NUMBER);
        inst.startDSClient(); // also accepts DS-forwarded address at competitions

        // ── Launch each module in a daemon thread ─────────────────────────────────
        // Daemon threads are killed automatically when main exits, so a crash in
        // one module won't hang the process.
        Thread[] threads = {
            new Thread(
                () -> PhysicsEngine.run(inst.getTable("UltraShooter")),
                "physics"
            ),
            new Thread(
                () -> ShooterTuner.run(inst.getTable("ShooterTuner")),
                "shooter-tuner"
            ),
            new Thread(
                () -> BumpTuner.run(inst.getTable("BumpTuner")),
                "bump-tuner"
            ),
            new Thread(
                () -> TrajectoryVisualizer.run(inst.getTable("UltraShooter")),
                "traj-viz"
            ),
        };

        for (Thread t : threads) {
            t.setDaemon(true);
            t.start();
            System.out.println("Started thread: " + t.getName());
        }

        // Keep the main thread alive — if a worker crashes, the JVM stays up and
        // the remaining threads keep running.
        for (Thread t : threads) {
            t.join();
        }
    }
}
