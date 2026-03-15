/**
 * UltraShooter Physics Engine — Team 539
 *
 * <p>Mirrors pi/physics_engine.py exactly. Runs at 50 Hz in a daemon thread
 * launched by {@link Main}. Reads the 1-second averaged hub distance from NT,
 * computes required flywheel surface velocity (with optional aerodynamic drag
 * and flywheel efficiency), and publishes results back to the roboRIO.
 *
 * <p>Physics models:
 * <ul>
 *   <li>No drag: analytic vacuum range equation solved for v₀</li>
 *   <li>With drag: Euler integration + binary search on ball exit speed</li>
 * </ul>
 *
 * <p>NT table: "UltraShooter"
 * <ul>
 *   <li>Reads:  {@code Avg Distance to Hub (ft)}</li>
 *   <li>Writes: {@code Pi Physics Velocity ft/s}, {@code Pi Time of Flight (s)},
 *               {@code Pi Heartbeat}</li>
 * </ul>
 */
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PhysicsEngine {

    // ── Geometry — mirror Constants.java UltraShooterConstants ──────────────────
    static final double HOOD_HEIGHT_INCHES    = 27.0;
    static final double HUB_HEIGHT_INCHES     = 72.0;
    static final double SHOOTER_OFFSET_INCHES = 8.0;
    static final double LAUNCH_ANGLE_DEG      = 75.0;

    // ── Physics defaults (overridden live from ShooterTuner NT each cycle) ───────
    static final double DEFAULT_EFFICIENCY    = 0.43;
    static final double DEFAULT_DRAG_COEFF    = 0.0132;
    static final double DEFAULT_BALL_MASS_LBS = 0.595;

    private static final double LOOP_PERIOD_S = 0.02; // 50 Hz

    // ── Unit helpers ─────────────────────────────────────────────────────────────

    static double inToM(double inches)  { return inches * 0.0254; }
    static double mpsToFps(double mps)  { return mps * 3.28084; }
    static double ftToM(double feet)    { return feet / 3.28084; }

    // ── Euler trajectory simulation ──────────────────────────────────────────────

    /**
     * Euler-integrates the trajectory under quadratic drag and returns the ball's
     * height (m, relative to hood exit) when it crosses {@code targetX} metres downrange.
     */
    static double simulateYAtX(double v0, double targetX, double angleRad,
                                double dragPerMass, double dt) {
        double vx = v0 * Math.cos(angleRad);
        double vy = v0 * Math.sin(angleRad);
        double x = 0, y = 0, prevX = 0, prevY = 0;

        for (int i = 0; i < 5000; i++) {
            double speed = Math.sqrt(vx * vx + vy * vy);
            vx += (-dragPerMass * speed * vx) * dt;
            vy += (-9.81 - dragPerMass * speed * vy) * dt;
            prevX = x; prevY = y;
            x += vx * dt;
            y += vy * dt;
            if (x >= targetX) {
                double t = (x - prevX) > 1e-9 ? (targetX - prevX) / (x - prevX) : 0.0;
                return prevY + t * (y - prevY);
            }
            if (y < -2.0) break;
        }
        return y;
    }

    /**
     * Binary-searches for ball exit speed v₀ (m/s) that places the trajectory at
     * height {@code h} above the hood exit at horizontal range {@code d}.
     * Returns 0.0 if the shot is physically impossible.
     */
    static double binarySearchV0(double d, double h, double angleRad, double dragPerMass) {
        final double HI = 40.0;
        final double dt = 0.005;

        if (simulateYAtX(HI, d, angleRad, dragPerMass, dt) < h) return 0.0;

        double lo = 0.5, hi = HI;
        for (int i = 0; i < 60; i++) {
            double mid = (lo + hi) * 0.5;
            if (simulateYAtX(mid, d, angleRad, dragPerMass, dt) < h) lo = mid;
            else                                                       hi = mid;
        }
        return (lo + hi) * 0.5;
    }

    /**
     * Returns time of flight (s) for the ball to travel {@code targetX} metres
     * under quadratic drag. Sub-step accurate via linear interpolation.
     */
    static double simulateTof(double v0, double targetX, double angleRad,
                               double dragPerMass, double dt) {
        double vx = v0 * Math.cos(angleRad);
        double vy = v0 * Math.sin(angleRad);
        double x = 0, prevX = 0;

        for (int i = 0; i < 5000; i++) {
            double speed = Math.sqrt(vx * vx + vy * vy);
            vx += (-dragPerMass * speed * vx) * dt;
            vy += (-9.81 - dragPerMass * speed * vy) * dt;
            prevX = x;
            x += vx * dt;
            if (x >= targetX) {
                double frac = (x - prevX) > 1e-9 ? (targetX - prevX) / (x - prevX) : 0.0;
                return (i + frac) * dt;
            }
        }
        return 0.0;
    }

    // ── Public physics API ───────────────────────────────────────────────────────

    /**
     * Required flywheel surface velocity (ft/s) for the given robot-center-to-hub
     * distance. Returns 0.0 if the shot is geometrically impossible.
     *
     * @param distanceToHubM     robot-center → hub-center distance (metres)
     * @param flywheelEfficiency ball exit speed / flywheel surface speed (0–1)
     * @param dragCoeff          aerodynamic B = 0.5·Cd·ρ·A (kg/m); 0 = vacuum model
     * @param ballMassLbs        ball mass (lbs); converted to kg internally
     */
    public static double calculateVelocityFps(double distanceToHubM, double flywheelEfficiency,
                                              double dragCoeff, double ballMassLbs) {
        double angleRad = Math.toRadians(LAUNCH_ANGLE_DEG);
        double d = distanceToHubM + inToM(SHOOTER_OFFSET_INCHES);
        double h = inToM(HUB_HEIGHT_INCHES) - inToM(HOOD_HEIGHT_INCHES);

        if (d <= 0 || flywheelEfficiency <= 0) return 0.0;

        double v0Mps;
        if (dragCoeff <= 0) {
            double cosT  = Math.cos(angleRad);
            double tanT  = Math.tan(angleRad);
            double denom = 2.0 * cosT * cosT * (d * tanT - h);
            if (denom <= 0) return 0.0;
            v0Mps = d * Math.sqrt(9.81 / denom);
        } else {
            double ballMassKg  = ballMassLbs * 0.453592;
            double dragPerMass = dragCoeff / Math.max(ballMassKg, 0.001);
            v0Mps = binarySearchV0(d, h, angleRad, dragPerMass);
            if (v0Mps <= 0) return 0.0;
        }

        return mpsToFps(v0Mps / flywheelEfficiency);
    }

    /**
     * Ball time of flight (s) to the hub. Analytic when {@code dragCoeff == 0};
     * numerical otherwise.
     *
     * @param distanceToHubM robot-center → hub-center distance (metres)
     * @param dragCoeff      aerodynamic B (kg/m); 0 = vacuum model
     * @param ballMassLbs    ball mass (lbs)
     */
    public static double calculateTofSeconds(double distanceToHubM,
                                             double dragCoeff, double ballMassLbs) {
        double angleRad = Math.toRadians(LAUNCH_ANGLE_DEG);
        double d = distanceToHubM + inToM(SHOOTER_OFFSET_INCHES);
        double h = inToM(HUB_HEIGHT_INCHES) - inToM(HOOD_HEIGHT_INCHES);

        if (d <= 0) return 0.0;

        if (dragCoeff <= 0) {
            double cosT  = Math.cos(angleRad);
            double tanT  = Math.tan(angleRad);
            double denom = 2.0 * cosT * cosT * (d * tanT - h);
            if (denom <= 0) return 0.0;
            double v0Mps = d * Math.sqrt(9.81 / denom);
            return d / (v0Mps * cosT);
        } else {
            double ballMassKg  = ballMassLbs * 0.453592;
            double dragPerMass = dragCoeff / Math.max(ballMassKg, 0.001);
            double v0Mps = binarySearchV0(d, h, angleRad, dragPerMass);
            if (v0Mps <= 0) return 0.0;
            return simulateTof(v0Mps, d, angleRad, dragPerMass, 0.005);
        }
    }

    // ── NT loop ──────────────────────────────────────────────────────────────────

    /**
     * Blocking 50 Hz loop. Called from {@link Main} in a daemon thread.
     *
     * @param table pre-initialized "UltraShooter" NT table
     */
    public static void run(NetworkTable table) {
        NetworkTableInstance inst  = table.getInstance();
        NetworkTable tunerParams   = inst.getTable("ShooterTuner").getSubTable("Params");

        DoubleSubscriber distSub  = table.getDoubleTopic("Avg Distance to Hub (ft)").subscribe(0.0);
        DoubleSubscriber efficSub = tunerParams.getDoubleTopic("FlywheelEfficiency").subscribe(DEFAULT_EFFICIENCY);
        DoubleSubscriber dragSub  = tunerParams.getDoubleTopic("DragCoefficient").subscribe(DEFAULT_DRAG_COEFF);
        DoubleSubscriber massSub  = tunerParams.getDoubleTopic("BallMassLbs").subscribe(DEFAULT_BALL_MASS_LBS);

        DoublePublisher  velPub = table.getDoubleTopic("Pi Physics Velocity ft/s").publish();
        DoublePublisher  tofPub = table.getDoubleTopic("Pi Time of Flight (s)").publish();
        IntegerPublisher hbPub  = table.getIntegerTopic("Pi Heartbeat").publish();

        System.out.println("[PhysicsEngine] Running at 50 Hz (drag-aware).");

        long heartbeat = 0;
        long loopNs    = (long) (LOOP_PERIOD_S * 1_000_000_000L);

        while (true) {
            long start = System.nanoTime();

            double distM = ftToM(distSub.get());
            double effic = efficSub.get();
            double drag  = dragSub.get();
            double mass  = massSub.get();

            velPub.set(calculateVelocityFps(distM, effic, drag, mass));
            tofPub.set(calculateTofSeconds(distM, drag, mass));
            hbPub.set(heartbeat++);

            long sleep = loopNs - (System.nanoTime() - start);
            if (sleep > 0) {
                try { Thread.sleep(sleep / 1_000_000L, (int) (sleep % 1_000_000L)); }
                catch (InterruptedException e) { Thread.currentThread().interrupt(); return; }
            }
        }
    }
}
