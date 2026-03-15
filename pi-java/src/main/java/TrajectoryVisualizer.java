/**
 * Shot Trajectory Visualizer — Team 539
 *
 * <p>Mirrors pi/trajectory_visualizer.py. Renders a 2D side-view of the current
 * physics-calculated shot as JPEG frames and streams them as MJPEG on port 1184.
 * Uses Java AWT (no OpenCV) so no native dependencies are needed on the Pi.
 *
 * <p>Stream URL: {@code http://10.5.39.XX:1184/stream.mjpg}
 * Add as a camera widget in Elastic.
 *
 * <p>NT reads from "UltraShooter" and "ShooterTuner/Params".
 * Runs at 10 Hz — the trajectory is static, so high frame rate is unnecessary.
 */
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

public class TrajectoryVisualizer {

    // ── Stream / canvas settings ──────────────────────────────────────────────────
    private static final int STREAM_PORT = 1184;
    private static final int CANVAS_W    = 800;
    private static final int CANVAS_H    = 420;
    private static final int FPS         = 10;

    // ── Colours ───────────────────────────────────────────────────────────────────
    private static final Color BG          = new Color(28,  28,  28);
    private static final Color WHITE       = new Color(255, 255, 255);
    private static final Color GREY        = new Color(140, 140, 140);
    private static final Color DARK_GREY   = new Color(70,  70,  70);
    private static final Color GREEN       = new Color(80,  200, 80);
    private static final Color YELLOW      = new Color(255, 215, 0);
    private static final Color TRAJ_COLOR  = new Color(210, 210, 210);
    private static final Color TEXT_COLOR  = new Color(225, 225, 225);
    private static final Color LABEL_COLOR = new Color(180, 200, 255);

    // ── Shared JPEG frame buffer ──────────────────────────────────────────────────
    private static final ReentrantLock frameLock   = new ReentrantLock();
    private static volatile byte[]     latestJpeg  = new byte[0];

    // ── Physics helpers ───────────────────────────────────────────────────────────

    /** Returns list of (x, y) world-space points in metres, y from the floor. */
    private static List<double[]> simulateTrajectory(
            double v0Mps, double angleRad, double dragPerMass, double hoodHeightM) {

        List<double[]> pts = new ArrayList<>();
        double vx = v0Mps * Math.cos(angleRad);
        double vy = v0Mps * Math.sin(angleRad);
        double x  = 0, y = hoodHeightM; // start at hood exit height above floor

        pts.add(new double[]{x, y});
        final double dt = 0.005;

        for (int i = 0; i < 4000; i++) {
            double speed = Math.sqrt(vx * vx + vy * vy);
            vx += (-dragPerMass * speed * vx) * dt;
            vy += (-9.81 - dragPerMass * speed * vy) * dt;
            x  += vx * dt;
            y  += vy * dt;
            pts.add(new double[]{x, y});
            if (y < -0.1 || x > 12.0) break;
        }
        return pts;
    }

    // ── Rendering ─────────────────────────────────────────────────────────────────

    private static byte[] drawFrame(
            double distanceM, double vFlywheelFps, double angleDeg,
            double hoodHM, double hubHM,
            double efficiency, double dragCoeff, double ballMassLbs,
            boolean piActive) {

        double angleRad    = Math.toRadians(angleDeg);
        double ballMassKg  = ballMassLbs * 0.453592;
        double dragPerMass = (dragCoeff > 0) ? dragCoeff / Math.max(ballMassKg, 0.001) : 0.0;
        double v0BallMps   = (vFlywheelFps / 3.28084) * efficiency;

        // Simulate trajectory.
        List<double[]> traj = new ArrayList<>();
        if (distanceM > 0.1 && v0BallMps > 0.5) {
            traj = simulateTrajectory(v0BallMps, angleRad, dragPerMass, hoodHM);
        }

        // World bounding box. Guard against zero to prevent divide-by-zero in w2c.
        double xMax = Math.max(distanceM * 1.12, 1.5);
        double yMax = Math.max(hubHM * 1.45, 0.1);
        if (!traj.isEmpty()) {
            for (double[] p : traj) yMax = Math.max(yMax, p[1] * 1.18);
        }

        // Layout margins (pixels).
        final int ML = 58, MR = 18, MT = 32, MB = 38;
        final int pw = CANVAS_W - ML - MR;
        final int ph = CANVAS_H - MT - MB;

        BufferedImage img = new BufferedImage(CANVAS_W, CANVAS_H, BufferedImage.TYPE_INT_RGB);
        Graphics2D g = img.createGraphics();
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);

        // World → canvas coordinate helper (captured as lambdas via helper method).
        final double _xMax = xMax, _yMax = yMax;
        java.util.function.BiFunction<Double, Double, int[]> w2c = (wx, wy) -> {
            int px = (int) (ML + wx / _xMax * pw);
            int py = (int) (CANVAS_H - MB - wy / _yMax * ph);
            return new int[]{
                Math.max(0, Math.min(CANVAS_W - 1, px)),
                Math.max(0, Math.min(CANVAS_H - 1, py))
            };
        };

        // Background.
        g.setColor(BG);
        g.fillRect(0, 0, CANVAS_W, CANVAS_H);

        // Floor line.
        int[] fl = w2c.apply(0.0, 0.0), fr = w2c.apply(xMax, 0.0);
        g.setColor(DARK_GREY);
        g.setStroke(new BasicStroke(1));
        g.drawLine(fl[0], fl[1], fr[0], fr[1]);

        // Y-axis ticks.
        g.setFont(new Font("Monospaced", Font.PLAIN, 11));
        for (double yTick : new double[]{0.0, hoodHM, hubHM}) {
            int[] tc = w2c.apply(0.0, yTick);
            g.setColor(GREY);
            g.drawLine(ML - 5, tc[1], ML, tc[1]);
            g.drawString(String.format("%.1fm", yTick), 2, tc[1] + 4);
        }

        // X-axis ticks.
        for (double xTick : new double[]{0.0, distanceM * 0.5, distanceM}) {
            int[] tc = w2c.apply(xTick, 0.0);
            g.setColor(GREY);
            g.drawLine(tc[0], tc[1], tc[0], tc[1] + 4);
            g.drawString(String.format("%.1fm", xTick), tc[0] - 14, tc[1] + 16);
        }

        // Robot / shooter box (left).
        double robotWM = 0.65, robotHM = hoodHM + 0.03;
        int[] rtl = w2c.apply(0.0, robotHM), rbr = w2c.apply(robotWM, 0.0);
        g.setColor(GREY);
        g.setStroke(new BasicStroke(2));
        g.drawRect(rtl[0], rtl[1], rbr[0] - rtl[0], rbr[1] - rtl[1]);

        // Shooter exit dot (green).
        int[] shootPx = w2c.apply(robotWM * 0.88, hoodHM);
        g.setColor(GREEN);
        g.fillOval(shootPx[0] - 5, shootPx[1] - 5, 10, 10);

        // Launch direction arrow.
        int[] arrowEnd = w2c.apply(
            robotWM * 0.88 + 0.22 * Math.cos(angleRad),
            hoodHM         + 0.22 * Math.sin(angleRad));
        g.setColor(GREEN);
        g.setStroke(new BasicStroke(2));
        drawArrow(g, shootPx[0], shootPx[1], arrowEnd[0], arrowEnd[1]);

        // Hub / target box (right).
        double hubWM = 0.28, hubOpenM = 0.28;
        int[] htl = w2c.apply(distanceM - hubWM, hubHM + hubOpenM / 2.0);
        int[] hbr = w2c.apply(distanceM,          hubHM - hubOpenM / 2.0);
        g.setColor(WHITE);
        g.setStroke(new BasicStroke(2));
        g.drawRect(htl[0], htl[1], hbr[0] - htl[0], hbr[1] - htl[1]);

        // Hub stand.
        int[] standTop = w2c.apply(distanceM - hubWM / 2.0, hubHM - hubOpenM / 2.0);
        int[] standBot = w2c.apply(distanceM - hubWM / 2.0, 0.0);
        g.setColor(DARK_GREY);
        g.setStroke(new BasicStroke(1));
        g.drawLine(standTop[0], standTop[1], standBot[0], standBot[1]);

        // Trajectory curve.
        if (traj.size() >= 2) {
            g.setColor(TRAJ_COLOR);
            g.setStroke(new BasicStroke(2));
            for (int i = 0; i < traj.size() - 1; i++) {
                int[] a = w2c.apply(traj.get(i)[0], traj.get(i)[1]);
                int[] b = w2c.apply(traj.get(i + 1)[0], traj.get(i + 1)[1]);
                g.drawLine(a[0], a[1], b[0], b[1]);
            }
        }

        // Ball — yellow dot at hub opening.
        int[] ballPx = w2c.apply(distanceM - hubWM * 0.15, hubHM);
        g.setColor(YELLOW);
        g.fillOval(ballPx[0] - 9, ballPx[1] - 9, 18, 18);
        g.setColor(WHITE);
        g.setStroke(new BasicStroke(1));
        g.drawOval(ballPx[0] - 9, ballPx[1] - 9, 18, 18);

        // Info overlay.
        g.setFont(new Font("Monospaced", Font.PLAIN, 12));
        String src = piActive ? "Pi" : "RIO";
        String[] lines = {
            String.format("Dist:   %.1f ft  (%.0f\")", distanceM * 3.28084, distanceM / 0.0254),
            String.format("Speed:  %.1f ft/s  (flywheel)", vFlywheelFps),
            String.format("Angle:  %.1f\u00b0  Hood: %.0f\"  Hub: %.0f\"",
                          angleDeg, hoodHM * 39.37, hubHM * 39.37),
            String.format("Effic:  %.0f%%   Drag: %.4f kg/m",
                          efficiency * 100, dragCoeff),
            "Source: " + src,
        };
        Color[] lineColors = {TEXT_COLOR, TEXT_COLOR, TEXT_COLOR, LABEL_COLOR,
                              piActive ? GREEN : LABEL_COLOR};
        for (int i = 0; i < lines.length; i++) {
            g.setColor(lineColors[i]);
            g.drawString(lines[i], ML + 6, MT + 10 + i * 20);
        }

        // Title.
        g.setColor(LABEL_COLOR);
        g.setFont(new Font("SansSerif", Font.PLAIN, 13));
        g.drawString("Team 539 \u2014 Shot Trajectory", CANVAS_W / 2 - 100, 20);

        g.dispose();

        // Encode to JPEG.
        try (ByteArrayOutputStream baos = new ByteArrayOutputStream()) {
            ImageIO.write(img, "jpg", baos);
            return baos.toByteArray();
        } catch (IOException e) {
            return new byte[0];
        }
    }

    /** Draws a simple arrow from (x1,y1) to (x2,y2). */
    private static void drawArrow(Graphics2D g, int x1, int y1, int x2, int y2) {
        g.drawLine(x1, y1, x2, y2);
        double angle = Math.atan2(y2 - y1, x2 - x1);
        int tipLen = 8;
        double spread = Math.PI / 6;
        int ax1 = (int) (x2 - tipLen * Math.cos(angle - spread));
        int ay1 = (int) (y2 - tipLen * Math.sin(angle - spread));
        int ax2 = (int) (x2 - tipLen * Math.cos(angle + spread));
        int ay2 = (int) (y2 - tipLen * Math.sin(angle + spread));
        g.drawLine(x2, y2, ax1, ay1);
        g.drawLine(x2, y2, ax2, ay2);
    }

    // ── MJPEG HTTP server ─────────────────────────────────────────────────────────

    private static void startServer() {
        try {
            HttpServer server = HttpServer.create(new InetSocketAddress(STREAM_PORT), 0);
            server.createContext("/stream.mjpg", TrajectoryVisualizer::handleStream);
            server.setExecutor(java.util.concurrent.Executors.newCachedThreadPool());
            server.start();
            System.out.printf("[TrajectoryViz] MJPEG stream on port %d → "
                + "http://10.5.39.XX:%d/stream.mjpg%n", STREAM_PORT, STREAM_PORT);
        } catch (IOException e) {
            System.out.println("[TrajectoryViz] Failed to start HTTP server: " + e.getMessage());
        }
    }

    private static void handleStream(HttpExchange exchange) {
        try {
            exchange.getResponseHeaders().set("Content-Type",
                "multipart/x-mixed-replace; boundary=frame");
            exchange.getResponseHeaders().set("Cache-Control", "no-cache");
            exchange.sendResponseHeaders(200, 0);
            OutputStream out = exchange.getResponseBody();

            long frameDelayMs = 1000L / FPS;
            while (true) {
                byte[] jpeg;
                frameLock.lock();
                try { jpeg = latestJpeg; } finally { frameLock.unlock(); }

                if (jpeg.length > 0) {
                    String header = "--frame\r\nContent-Type: image/jpeg\r\n"
                        + "Content-Length: " + jpeg.length + "\r\n\r\n";
                    out.write(header.getBytes());
                    out.write(jpeg);
                    out.write("\r\n".getBytes());
                    out.flush();
                }
                Thread.sleep(frameDelayMs);
            }
        } catch (Exception e) {
            // Client disconnected — normal.
        }
    }

    // ── NT loop ───────────────────────────────────────────────────────────────────

    /**
     * Blocking 10 Hz loop. Called from {@link Main} in a daemon thread.
     *
     * @param table pre-initialized "UltraShooter" NT table
     */
    public static void run(NetworkTable table) {
        NetworkTableInstance inst  = table.getInstance();
        NetworkTable tunerParams   = inst.getTable("ShooterTuner").getSubTable("Params");

        DoubleSubscriber  distSub    = table.getDoubleTopic("Avg Distance to Hub (ft)").subscribe(0.0);
        DoubleSubscriber  targetSub  = table.getDoubleTopic("Target ft/s").subscribe(0.0);
        DoubleSubscriber  physicsSub = table.getDoubleTopic("Physics Velocity ft/s").subscribe(0.0);
        DoubleSubscriber  angleSub   = table.getDoubleTopic("Launch Angle (deg)").subscribe(75.0);
        DoubleSubscriber  hoodSub    = table.getDoubleTopic("Hood Height From Floor (in)").subscribe(27.0);
        DoubleSubscriber  hubSub     = table.getDoubleTopic("Hub Center Height From Floor (in)").subscribe(96.0);
        BooleanSubscriber piActSub   = table.getBooleanTopic("Pi Active").subscribe(false);

        DoubleSubscriber efficSub = tunerParams.getDoubleTopic("FlywheelEfficiency").subscribe(0.85);
        DoubleSubscriber dragSub  = tunerParams.getDoubleTopic("DragCoefficient").subscribe(0.0132);
        DoubleSubscriber massSub  = tunerParams.getDoubleTopic("BallMassLbs").subscribe(0.595);

        // Start the MJPEG HTTP server in a background daemon thread.
        Thread serverThread = new Thread(TrajectoryVisualizer::startServer, "traj-http");
        serverThread.setDaemon(true);
        serverThread.start();

        long frameDelayMs = 1000L / FPS;

        while (true) {
            long start = System.currentTimeMillis();

            double d      = distSub.get() / 3.28084;   // ft → m
            double vFps   = targetSub.get();
            double angle  = angleSub.get();
            double hoodH  = hoodSub.get() * 0.0254;    // in → m
            double hubH   = hubSub.get()  * 0.0254;
            boolean piAct = piActSub.get();
            double effic  = efficSub.get();
            double drag   = dragSub.get();
            double mass   = massSub.get();

            // Fall back to physics velocity if no active setpoint.
            if (vFps < 1.0) vFps = physicsSub.get();

            byte[] frame = drawFrame(d, vFps, angle, hoodH, hubH, effic, drag, mass, piAct);
            frameLock.lock();
            try { latestJpeg = frame; } finally { frameLock.unlock(); }

            long elapsed = System.currentTimeMillis() - start;
            long sleep   = frameDelayMs - elapsed;
            if (sleep > 0) {
                try { Thread.sleep(sleep); }
                catch (InterruptedException e) { Thread.currentThread().interrupt(); return; }
            }
        }
    }
}
