package frc.robot.subsystems.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.GameData;
import frc.robot.Landmarks;
import frc.robot.subsystems.iodiagnostics.UltraShooterIO;
import frc.robot.subsystems.iodiagnostics.UltraShooterIOInputsAutoLogged;
import frc.util.LoggedTracer;
// import frc.robot.subsystems.tuning.ShooterTuner; // Pi-backed live shooter tuning (disabled)

/**
 * UltraShooter — physics-based shooter subsystem.
 *
 * <p>Built on the same three-motor SparkFlex hardware as {@link ShooterOrca},
 * but derives the required flywheel surface speed from a projectile-motion
 * formula rather than an empirical distance-to-speed look-up table.
 *
 * <p>Key constants live in {@link Constants.UltraShooterConstants}; update the
 * geometry values (hood height, shooter offset, launch angle) after physically
 * measuring the robot.
 *
 * <h2>Physics model</h2>
 * With a fixed launch angle θ, horizontal distance d (shooter exit → hub
 * center), and height h (hub center above the hood exit):
 * <pre>
 *   v₀ = d × √( g / (2 · cos²θ · (d·tanθ − h)) )
 * </pre>
 * where g = 9.81 m/s².  The result is converted from m/s to ft/s to match the
 * existing flywheel velocity convention.
 */
public class UltraShooter extends SubsystemBase {

    // ── IO layer ──────────────────────────────────────────────────────────────

    private final UltraShooterIO io;
    private final UltraShooterIOInputsAutoLogged inputs = new UltraShooterIOInputsAutoLogged();

    private static final double WHEEL_CIRCUMFERENCE_FEET = Math.PI * (4.0 / 12.0);

    // ── Cached physics constants (derived from Constants at class-load time) ──
    // Avoids calling Math.toRadians / Units.inchesToMeters on every solver call.
    private static final double LAUNCH_ANGLE_RAD =
            Math.toRadians(Constants.UltraShooterConstants.kLaunchAngleDegrees);
    private static final double SHOOTER_OFFSET_M =
            Units.inchesToMeters(Constants.UltraShooterConstants.kShooterCenterlineOffsetInches);
    private static final double HOOD_HEIGHT_M =
            Units.inchesToMeters(Constants.UltraShooterConstants.kHoodHeightFromFloorInches);
    private static final double HUB_HEIGHT_M =
            Units.inchesToMeters(Constants.UltraShooterConstants.kHubCenterHeightFromFloorInches);
    private static final double METERS_TO_FEET = 3.28084;

    // ── State ─────────────────────────────────────────────────────────────────

    /** Average of all three encoder velocities sampled once per periodic() cycle (ft/s). */
    private double cachedVelocity = 0;
    /** Final velocity target set by commands (ft/s). */
    private double velocityTarget = 0;
    /** Accumulated feedforward voltage correction (V). Absorbs friction, battery sag, and
     *  KV calibration error. Only integrates after the ramp has reached target; resets on stop. */
    private double voltageBias = 0.0;
    /** Ramped setpoint currently fed to the PID controllers (ft/s). */
    private double rampedSetpoint = 0;
    /** Latches true once the flywheel first reaches target; cleared on target change. */
    private boolean readyLatch = false;
    /** Throttles trajectory visualization to ~10 Hz (every 5 cycles at 50 Hz). */
    private int vizSkipCounter = 0;

    /** Circular buffer for rolling-average velocity filtering. */
    private final double[] velocityBuffer =
            new double[Constants.UltraShooterConstants.kVelocityAvgSamples];
    private int    velocityBufferIndex = 0;
    private double velocityBufferSum   = 0.0;

    /** Circular buffer for 1-second (50-sample) rolling-average distance filtering. */
    private static final int DISTANCE_AVG_SAMPLES = 50; // 50 Hz × 1 s
    private final double[] distanceBuffer = new double[DISTANCE_AVG_SAMPLES];
    private int    distanceBufferIndex = 0;
    private double distanceBufferSum   = 0.0;

    /**
     * Stationary physics cache — avoids re-running the binary-search solver every
     * cycle when the robot isn't moving and distance hasn't meaningfully changed.
     * Both fields are reset to -1 when the robot is moving.
     */
    private double physicsTargetCacheFps  = -1.0;
    private double physicsTargetCacheDist = -1.0;
    /** Distance change (ft) that invalidates the stationary physics cache. */
    private static final double PHYSICS_CACHE_DIST_TOLERANCE_FT = 0.5; // 6 in
    /** Cached physics telemetry — recomputed at ~10 Hz with the visualization, logged every cycle. */
    private double cachedTelemetryPhysicsFps = 0.0;
    private double cachedTelemetryTof        = 0.0;

    // ── Swerve reference (for distance-to-hub queries) ────────────────────────

    private final Swerve swerve;

    // private final ShooterTuner shooterTuner; // Pi-backed live shooter tuning (disabled)

    // ── NetworkTable — Pi physics engine reader entries only ──────────────────
    // These entries are READ from physics_engine.py running on the Pi.
    // All output telemetry is handled by Logger.recordOutput() in updateNetworkTable().

    private final NetworkTable        nt           = NetworkTableInstance.getDefault().getTable("UltraShooter");
    // ── Trajectory Mechanism2d canvas ─────────────────────────────────────────
    // Draws a side-view of the current shot: robot box, arc, hub box, yellow fuel ball.
    // Published to SmartDashboard as "Shot Trajectory" — add as a Mechanism2d widget
    // in Elastic.

    private static final double TRAJ_CANVAS_W_FT = 21.33;  // canvas width  (ft)
    private static final double TRAJ_CANVAS_H_FT = 10.50;  // canvas height (ft)
    private static final double VIZ_HUB_W_FT     = 0.722;  // hub box depth (ft)
    private static final double VIZ_HUB_OPEN_FT  = 0.853;  // hub opening height (ft)
    private static final int    TRAJ_SEG_COUNT    = 60;    // trajectory ligament segments

    private final Mechanism2d         trajCanvas;

    // Roots — the three hub/fuel roots are repositioned every cycle.
    private final MechanismRoot2d     trajShooterRoot;
    private final MechanismRoot2d     trajHubBoxRoot;   // bottom-left of hub opening
    private final MechanismRoot2d     trajHubStandRoot; // base of hub stand
    private final MechanismRoot2d     trajFuelRoot;     // fuel ball centre

    // Ideal trajectory arc — white, physics only (efficiency = 1)
    private final MechanismLigament2d[] trajSegs      = new MechanismLigament2d[TRAJ_SEG_COUNT];
    // Tuned trajectory arc — red, ideal v0 scaled by parabolic offset
    private final MechanismLigament2d[] trajTunedSegs = new MechanismLigament2d[TRAJ_SEG_COUNT];

    private final MechanismRoot2d     trajTunedRoot;

    // Hub structure: stand (grey) + 3-sided open box (white, open toward shooter)
    private final MechanismLigament2d trajHubStand;
    private final MechanismLigament2d trajHubBottom;
    private final MechanismLigament2d trajHubBack;
    private final MechanismLigament2d trajHubTop;

    // Fuel ball: short, thick, yellow ligament that reads as a dot
    private final MechanismLigament2d trajFuelDot;

    // ── Pi physics engine integration ─────────────────────────────────────────
    // physics_engine.py (on WPILibPi) publishes raw physics velocity and a
    // heartbeat counter every 20 ms.  We prefer the Pi result when the heartbeat
    // has changed within the last 25 cycles (500 ms); otherwise we fall back to
    // the local calculateRequiredVelocityFPS() call transparently.

    private final NetworkTableEntry   ntPiPhysics  = nt.getEntry("Pi Physics Velocity ft/s");
    private final NetworkTableEntry   ntPiHB       = nt.getEntry("Pi Heartbeat");

    /** Last heartbeat counter seen from the Pi. */
    private long piLastHeartbeat = Long.MIN_VALUE;
    /** Cycles since the heartbeat last changed (each cycle = 20 ms). */
    private int  piStaleFrames   = PI_STALE_THRESHOLD;
    /** A Pi is considered disconnected after this many stale cycles (500 ms). */
    private static final int PI_STALE_THRESHOLD = 25;

    /** kP value most recently written to the SparkFlex controllers. */
    private double appliedKp = Constants.UltraShooterConstants.kP;

    // ─────────────────────────────────────────────────────────────────────────
    // Construction
    // ─────────────────────────────────────────────────────────────────────────

    public UltraShooter(UltraShooterIO io, Swerve swerve) {
        this.io    = io;
        this.swerve = swerve;
        // this.shooterTuner = shooterTuner;

        // ── Mechanism2d trajectory canvas ─────────────────────────────────────
        final double hoodH = Constants.UltraShooterConstants.kHoodHeightFromFloorInches / 12.0;
        final double hubH  = Constants.UltraShooterConstants.kHubCenterHeightFromFloorInches / 12.0;

        trajCanvas = new Mechanism2d(TRAJ_CANVAS_W_FT, TRAJ_CANVAS_H_FT);

        // ── Robot: a vertical grey line representing the shooter-side wall ─────
        MechanismRoot2d robotRoot = trajCanvas.getRoot("Robot", 0.0, 0.0);
        robotRoot.append(new MechanismLigament2d(
                "wall", hoodH + 0.2, 90.0, 4, new Color8Bit(Color.kGray)));

        // ── Shooter exit: short green arrow at launch angle ───────────────────
        trajShooterRoot = trajCanvas.getRoot("ShooterExit", 0.0, hoodH);
        trajShooterRoot.append(new MechanismLigament2d(
                "arrow", 0.59,
                Constants.UltraShooterConstants.kLaunchAngleDegrees,
                3, new Color8Bit(Color.kGreen)));

        // ── Ideal trajectory arc: chain of white ligaments (efficiency = 1) ───
        trajSegs[0] = trajShooterRoot.append(new MechanismLigament2d(
                "seg_0", 0.164,
                Constants.UltraShooterConstants.kLaunchAngleDegrees,
                2, new Color8Bit(Color.kWhite)));
        for (int i = 1; i < TRAJ_SEG_COUNT; i++) {
            trajSegs[i] = trajSegs[i - 1].append(new MechanismLigament2d(
                    "seg_" + i, 0.164, 0.0, 2, new Color8Bit(Color.kWhite)));
        }

        // ── Tuned trajectory arc: chain of red ligaments (ideal × offset) ────
        trajTunedRoot = trajCanvas.getRoot("TunedExit", 0.0, hoodH);
        trajTunedSegs[0] = trajTunedRoot.append(new MechanismLigament2d(
                "tuned_0", 0.164,
                Constants.UltraShooterConstants.kLaunchAngleDegrees,
                2, new Color8Bit(Color.kRed)));
        for (int i = 1; i < TRAJ_SEG_COUNT; i++) {
            trajTunedSegs[i] = trajTunedSegs[i - 1].append(new MechanismLigament2d(
                    "tuned_" + i, 0.164, 0.0, 2, new Color8Bit(Color.kRed)));
        }

        // ── Hub stand: grey vertical line from floor to bottom of opening ─────
        trajHubStandRoot = trajCanvas.getRoot("HubStand", 6.56, 0.0);
        trajHubStand     = trajHubStandRoot.append(new MechanismLigament2d(
                "stand", hubH - VIZ_HUB_OPEN_FT / 2,
                90.0, 2, new Color8Bit(Color.kGray)));

        // ── Hub box: 3-sided (bottom → back → top), open toward shooter ──────
        // Root at bottom-left corner of the opening (shooter-facing side).
        trajHubBoxRoot = trajCanvas.getRoot("HubBox", 6.56, hubH - VIZ_HUB_OPEN_FT / 2);
        trajHubBottom  = trajHubBoxRoot.append(new MechanismLigament2d(
                "bottom", VIZ_HUB_W_FT, 0.0, 3, new Color8Bit(Color.kWhite)));
        trajHubBack    = trajHubBottom.append(new MechanismLigament2d(
                "back", VIZ_HUB_OPEN_FT, 90.0, 3, new Color8Bit(Color.kWhite)));
        trajHubTop     = trajHubBack.append(new MechanismLigament2d(
                "top", VIZ_HUB_W_FT, 90.0, 3, new Color8Bit(Color.kWhite)));

        // ── Fuel ball: thick short yellow ligament ≈ dot ─────────────────────
        trajFuelRoot = trajCanvas.getRoot("Fuel", 6.56, hubH);
        trajFuelDot  = trajFuelRoot.append(new MechanismLigament2d(
                "ball", 0.003, 0.0, 14, new Color8Bit(Color.kYellow)));

        SmartDashboard.putData("Shot Trajectory", trajCanvas);
        SmartDashboard.putData(this);
    }


    // ─────────────────────────────────────────────────────────────────────────
    // Projectile-motion calculator
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Calculates the required flywheel surface velocity (ft/s) to arc a ball into
     * the hub from the given robot-center-to-hub distance.
     *
     * <p>When drag is non-zero a numerical binary search is used (60 bisection
     * steps at 5 ms time-step); otherwise the closed-form analytic solution is used.
     * Flywheel speed = ball exit speed / efficiency.
     *
     * @param distanceToHubMeters  Odometry distance from robot center to hub (m).
     * @param flywheelEfficiency   Ball-to-flywheel velocity ratio (0–1).
     * @param dragCoefficient      Aerodynamic drag constant B = 0.5*Cd*rho*A (kg/m).
     * @param ballMassLbs          Ball mass (lbs), used in drag term B/m (converted to kg internally).
     * @return Required flywheel surface velocity in ft/s, or 0 if impossible.
     */
    public static double calculateRequiredVelocityFPS(
            double distanceToHubMeters,
            double flywheelEfficiency,
            double dragCoefficient,
            double ballMassLbs) {
        if (flywheelEfficiency <= 0) return 0;
        final double v0_mps = calcBallExitSpeedMps(distanceToHubMeters, dragCoefficient, ballMassLbs);
        if (v0_mps <= 0) return 0;
        // flywheel surface speed = ball exit speed / efficiency, then m/s → ft/s
        return (v0_mps / flywheelEfficiency) * METERS_TO_FEET;
    }

    /**
     * Convenience overload that uses the default physics constants from
     * {@link Constants.UltraShooterConstants}.  Suitable for static contexts
     * (e.g. {@link #initSendable}) where a {@link ShooterTuner} is not available.
     */
    public static double calculateRequiredVelocityFPS(double distanceToHubMeters) {
        return calculateRequiredVelocityFPS(
                distanceToHubMeters,
                Constants.UltraShooterConstants.kFlywheelEfficiency,
                Constants.UltraShooterConstants.kDragCoefficient,
                Constants.UltraShooterConstants.kBallMassLbs);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Time-of-flight calculator
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Calculates the ball's time of flight (seconds) from hood exit to hub center.
     * Uses the same physics model as {@link #calculateRequiredVelocityFPS} but
     * does not require flywheel efficiency — TOF depends only on ball exit speed.
     *
     * <p>Analytic (no drag): {@code t = d / (v₀ · cosθ)}<br>
     * With drag: single forward Euler simulation at 5 ms time-step.
     *
     * @param distanceToHubMeters  Odometry distance from robot center to hub (m).
     * @param dragCoefficient      Aerodynamic drag constant B = 0.5*Cd*rho*A (kg/m).
     * @param ballMassLbs          Ball mass (lbs).
     * @return Time of flight in seconds, or 0 if the shot is physically impossible.
     */
    public static double calculateTimeOfFlightSeconds(
            double distanceToHubMeters,
            double dragCoefficient,
            double ballMassLbs) {
        final double d      = distanceToHubMeters + SHOOTER_OFFSET_M;
        if (d <= 0) return 0;
        final double v0_mps = calcBallExitSpeedMps(distanceToHubMeters, dragCoefficient, ballMassLbs);
        if (v0_mps <= 0) return 0;
        if (dragCoefficient <= 0) {
            // Analytic: horizontal velocity is constant → t = d / vx
            return d / (v0_mps * Math.cos(LAUNCH_ANGLE_RAD));
        } else {
            final double ballMassKg  = ballMassLbs * 0.453592;
            final double dragPerMass = dragCoefficient / Math.max(ballMassKg, 0.001);
            return simulateTimeOfFlight(v0_mps, d, LAUNCH_ANGLE_RAD, dragPerMass);
        }
    }

    /** Convenience overload using default physics constants. */
    public static double calculateTimeOfFlightSeconds(double distanceToHubMeters) {
        return calculateTimeOfFlightSeconds(
                distanceToHubMeters,
                Constants.UltraShooterConstants.kDragCoefficient,
                Constants.UltraShooterConstants.kBallMassLbs);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Numerical solver helpers (drag physics)
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Core solver: returns the ball exit speed (m/s) needed to reach the hub
     * from {@code distanceToHubMeters}.  Handles both the analytic (no-drag)
     * and numerical (drag) cases.  All callers that previously duplicated this
     * logic now delegate here so the binary search runs at most once per call
     * site instead of once per derived quantity.
     *
     * @return Ball exit speed v₀ in m/s, or 0 if the shot is physically impossible.
     */
    private static double calcBallExitSpeedMps(
            double distanceToHubMeters,
            double dragCoefficient,
            double ballMassLbs) {
        final double d = distanceToHubMeters + SHOOTER_OFFSET_M;
        final double h = HUB_HEIGHT_M - HOOD_HEIGHT_M;
        if (d <= 0) return 0;
        if (dragCoefficient <= 0) {
            final double cosTheta = Math.cos(LAUNCH_ANGLE_RAD);
            final double tanTheta = Math.tan(LAUNCH_ANGLE_RAD);
            final double denom    = 2.0 * cosTheta * cosTheta * (d * tanTheta - h);
            if (denom <= 0) return 0;
            return d * Math.sqrt(9.81 / denom);
        } else {
            final double ballMassKg  = ballMassLbs * 0.453592;
            final double dragPerMass = dragCoefficient / Math.max(ballMassKg, 0.001);
            return binarySearchV0(d, h, LAUNCH_ANGLE_RAD, dragPerMass);
        }
    }

    /**
     * Binary-searches for the ball exit speed v₀ (m/s) such that the
     * simulated trajectory lands at horizontal distance {@code d} and
     * vertical rise {@code h}, subject to quadratic air resistance.
     *
     * @return v₀ in m/s, or 0 if the shot is physically impossible.
     */
    private static double binarySearchV0(double d, double h, double angleRad, double dragPerMass) {
        final double DT     = 0.005;   // 5 ms simulation time-step
        final double LO_MPS = 0.5;
        final double HI_MPS = 40.0;

        // Feasibility check: even at max speed can the ball reach hub height?
        if (simulateYAtX(HI_MPS, d, h, angleRad, dragPerMass, DT) < h) return 0;

        double lo = LO_MPS, hi = HI_MPS;
        for (int i = 0; i < 60; i++) {
            double mid = (lo + hi) * 0.5;
            if (simulateYAtX(mid, d, h, angleRad, dragPerMass, DT) < h) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return (lo + hi) * 0.5;
    }

    /**
     * Simulates projectile motion with quadratic drag and returns the time (s)
     * for the ball to reach {@code targetX} meters downrange.
     * Uses linear interpolation to sub-step accuracy.
     */
    private static double simulateTimeOfFlight(
            double v0Mps, double targetX, double angleRad, double dragPerMass) {
        final double DT = 0.005;
        double vx = v0Mps * Math.cos(angleRad);
        double vy = v0Mps * Math.sin(angleRad);
        double x = 0, prevX = 0;
        for (int i = 0; i < 5000; i++) {
            double speed = Math.sqrt(vx * vx + vy * vy);
            vx += (-dragPerMass * speed * vx) * DT;
            vy += (-9.81 - dragPerMass * speed * vy) * DT;
            prevX = x;
            x += vx * DT;
            if (x >= targetX) {
                double frac = (x - prevX) > 1e-9 ? (targetX - prevX) / (x - prevX) : 0.0;
                return (i + frac) * DT;
            }
        }
        return 0;
    }

    /**
     * Simulates projectile motion with quadratic drag and returns the ball's
     * height (m, relative to hood exit) when it crosses {@code targetX} meters
     * downrange.  Uses a simple Euler integration at fixed time-step {@code dt}.
     */
    private static double simulateYAtX(
            double v0, double targetX, double h,
            double angleRad, double dragPerMass, double dt) {
        double vx = v0 * Math.cos(angleRad);
        double vy = v0 * Math.sin(angleRad);
        double x  = 0, y = 0, prevX = 0, prevY = 0;
        for (int i = 0; i < 5000; i++) {
            double speed = Math.sqrt(vx * vx + vy * vy);
            vx += (-dragPerMass * speed * vx) * dt;
            vy += (-9.81 - dragPerMass * speed * vy) * dt;
            prevX = x;  prevY = y;
            x += vx * dt;
            y += vy * dt;
            if (x >= targetX) {
                // Linear interpolation to exact x = targetX
                double t = (x - prevX) > 1e-9 ? (targetX - prevX) / (x - prevX) : 0;
                return prevY + t * (y - prevY);
            }
            if (y < -2.0) break;  // ball hit the floor
        }
        return y;
    }

    /**
     * Computes the full trajectory as a list of (x, y) world-space points
     * (meters) starting at the hood exit, for use by NT-based visualizers.
     *
     * @param v0BallMps     Ball exit speed (m/s) — not flywheel speed.
     * @param angleRad      Launch angle (radians).
     * @param dragPerMass   Drag per unit mass B/m (1/m).
     * @param maxPoints     Maximum number of points to return.
     * @return Flat array [x0, y0, x1, y1, ...] in meters.
     */
    static double[] computeTrajectoryPoints(
            double v0BallMps, double angleRad, double dragPerMass, int maxPoints) {
        final double DT = 0.005;
        double vx = v0BallMps * Math.cos(angleRad);
        double vy = v0BallMps * Math.sin(angleRad);
        double x  = 0, y = 0;
        double[] pts = new double[maxPoints * 2];
        int n = 0;
        while (n / 2 < maxPoints) {
            pts[n++] = x;
            pts[n++] = y;
            double speed = Math.sqrt(vx * vx + vy * vy);
            vx += (-dragPerMass * speed * vx) * DT;
            vy += (-9.81 - dragPerMass * speed * vy) * DT;
            x  += vx * DT;
            y  += vy * DT;
            if (y < -2.0 || x > 20.0) break;
        }
        return Arrays.copyOf(pts, n);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Fine-tune offset interpolation
    // ─────────────────────────────────────────────────────────────────────────

    private static final double CLOSE_ANCHOR_FT = Constants.UltraShooterConstants.kCloseShotAnchorFeet;
    private static final double MID_ANCHOR_FT   = Constants.UltraShooterConstants.kMidShotAnchorFeet;
    private static final double FAR_ANCHOR_FT   = Constants.UltraShooterConstants.kFarShotAnchorFeet;

    // Precomputed Lagrange quadratic denominators — constant for fixed anchor distances.
    private static final double LAGRANGE_D0 =
            (CLOSE_ANCHOR_FT - MID_ANCHOR_FT) * (CLOSE_ANCHOR_FT - FAR_ANCHOR_FT);
    private static final double LAGRANGE_D1 =
            (MID_ANCHOR_FT - CLOSE_ANCHOR_FT) * (MID_ANCHOR_FT - FAR_ANCHOR_FT);
    private static final double LAGRANGE_D2 =
            (FAR_ANCHOR_FT - CLOSE_ANCHOR_FT) * (FAR_ANCHOR_FT - MID_ANCHOR_FT);

    /**
     * Parabolic (Lagrange quadratic) interpolation of the fine-tune speed offset (%)
     * through three anchor points (close / mid / far).  Distance is converted to feet
     * and clamped to [close, far] before evaluation so the parabola never extrapolates
     * wildly outside the tuned region.
     *
     * @param distanceMeters Distance from shooter to hub (meters).
     * @return Offset as a fraction (e.g. 5.0 % → 0.05).
     */
    private double interpolateOffsetFraction(double distanceMeters) {
        final double d  = Math.max(CLOSE_ANCHOR_FT, Math.min(FAR_ANCHOR_FT, distanceMeters * METERS_TO_FEET));
        final double p0 = Constants.UltraShooterConstants.kCloseShotOffsetPercent;
        final double p1 = Constants.UltraShooterConstants.kMidShotOffsetPercent;
        final double p2 = Constants.UltraShooterConstants.kFarShotOffsetPercent;
        // Lagrange basis polynomials evaluated at d (ft) — denominators precomputed.
        final double l0 = (d - MID_ANCHOR_FT) * (d - FAR_ANCHOR_FT) / LAGRANGE_D0;
        final double l1 = (d - CLOSE_ANCHOR_FT) * (d - FAR_ANCHOR_FT) / LAGRANGE_D1;
        final double l2 = (d - CLOSE_ANCHOR_FT) * (d - MID_ANCHOR_FT) / LAGRANGE_D2;
        return (p0 * l0 + p1 * l1 + p2 * l2) / 100.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Target control
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Sets the flywheel velocity target (ft/s).
     *
     * <p>The ready latch is only cleared when transitioning between stopped (0) and
     * running (non-zero). Mid-shot physics recalculations that nudge the target
     * value will NOT clear the latch, so a flywheel that has already been confirmed
     * ready stays ready for the duration of the shot.
     */
    public void setTarget(double targetFPS) {
        boolean wasRunning = velocityTarget != 0;
        boolean willRun    = targetFPS      != 0;
        if (wasRunning != willRun || targetFPS > velocityTarget) readyLatch = false;
        velocityTarget = targetFPS;
    }

    /**
     * Updates the flywheel target using the projectile-motion calculator from the
     * 1-second averaged distance, with the fine-tune offset blended in.
     *
     */
    public void setPhysicsTarget() {
        // Cache field velocity once — reused for both distance selection and radial correction.
        final ChassisSpeeds fieldVelocity = swerve.getFieldVelocity();
        final double distance = getAverageDistanceToHub(fieldVelocity);

        final double speedFps = Math.hypot(
                fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond) * METERS_TO_FEET;
        final boolean isStationary = speedFps <= MOVING_SPEED_THRESHOLD_FPS;

        // When stationary, skip the physics solver if the cached distance is still valid.
        if (isStationary
                && physicsTargetCacheFps >= 0
                && Math.abs(distance - physicsTargetCacheDist) < PHYSICS_CACHE_DIST_TOLERANCE_FT) {
            setTarget(physicsTargetCacheFps);
            return;
        }

        final double offsetFraction = interpolateOffsetFraction(distance);

        // if (isPiResultFresh()) { physicsSpeed = ntPiPhysics.getDouble(0.0); } else { ... }
        final double physicsSpeed = calculateRequiredVelocityFPS(
                distance,
                Constants.UltraShooterConstants.kFlywheelEfficiency,
                Constants.UltraShooterConstants.kDragCoefficient,
                Constants.UltraShooterConstants.kBallMassLbs);

        // Radial velocity compensation: if the robot is moving toward the hub,
        // the ball arrives faster and needs a lower exit speed (and vice versa).
        // Project field velocity onto the robot→hub unit vector.
        final Translation2d robotToHub =
                Landmarks.hubPosition().minus(swerve.getPose().getTranslation());
        final double distNorm = robotToHub.getNorm();
        final double radialVelocityMps = distNorm > 0.1
                ? (fieldVelocity.vxMetersPerSecond * robotToHub.getX() / distNorm
                   + fieldVelocity.vyMetersPerSecond * robotToHub.getY() / distNorm)
                : 0.0;
        // Convert radial speed to flywheel-surface ft/s (same scaling as physicsSpeed).
        final double radialCorrectionFps =
                radialVelocityMps * METERS_TO_FEET / Math.max(Constants.UltraShooterConstants.kFlywheelEfficiency, 0.01);
        final double adjustedSpeed = Math.max(0.0, physicsSpeed - radialCorrectionFps);
        final double target = adjustedSpeed * (1.0 + offsetFraction);

        if (isStationary) {
            physicsTargetCacheFps  = target;
            physicsTargetCacheDist = distance;
        } else {
            // Invalidate cache so it recomputes once when the robot comes to a stop.
            physicsTargetCacheFps = -1.0;
        }

        setTarget(target);
    }

    /**
     * Returns true when the Pi physics engine heartbeat has updated within the
     * last {@value #PI_STALE_THRESHOLD} cycles (~500 ms).
     */
    private boolean isPiResultFresh() {
        return piStaleFrames < PI_STALE_THRESHOLD;
    }

    /** Called every periodic cycle to track whether the Pi heartbeat is advancing. */
    private void updatePiStaleness() {
        long hb = ntPiHB.getInteger(Long.MIN_VALUE);
        if (hb != piLastHeartbeat) {
            piLastHeartbeat = hb;
            piStaleFrames   = 0;
        } else {
            piStaleFrames++;
        }
    }

    public void stop() {
        setTarget(0);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Getters
    // ─────────────────────────────────────────────────────────────────────────

    /** Instantaneous primary encoder velocity (ft/s). */
    public double getVelocity() {
        return cachedVelocity;
    }

    /** Rolling average of primary encoder velocity (ft/s). */
    public double getAverageVelocity() {
        return velocityBufferSum / Constants.UltraShooterConstants.kVelocityAvgSamples;
    }

    public double getTarget()        { return velocityTarget; }
    public double getRampedSetpoint(){ return rampedSetpoint; }

    /**
     * True once the flywheel has reached target speed (latches through the RPM
     * dip caused by ball contact). Resets when {@link #setTarget} is called with
     * a new value.
     */
    public boolean isReady() {
        if (velocityTarget <= 0) {
            readyLatch = false;
            return false;
        }
        if (!readyLatch) {
            readyLatch = rampedSetpoint >= velocityTarget
                    && Math.abs(getAverageVelocity() - velocityTarget)
                            < Constants.UltraShooterConstants.kReadyTolerance;
        }
        return readyLatch;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Commands
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Continuously spins up the flywheel using the physics calculator, updating
     * every cycle as the robot moves. Stops on interrupt.
     */
    public Command spinUpPhysicsCommand() {
        return runEnd(this::setPhysicsTarget, this::stop).withName("SpinUpPhysics");
    }

    /**
     * Pre-spins the flywheel to a fraction of the physics-calculated speed.
     *
     * <p>When {@link Constants.UltraShooterConstants#kEnableFMSAwarePreSpinLatch} is
     * {@code true}, pre-spin is gated by:
     * <ol>
     *   <li>The hub is active (within the 5-second expanded window).</li>
     *   <li>The robot is in its own alliance zone or the neutral zone.</li>
     * </ol>
     * When that constant is {@code false}, the flywheel always spins up to the
     * pre-spin speed regardless of FMS state, fuel detection, or field position.
     *
     * <p>Runs as the default command and is automatically interrupted by any shoot command.
     */
    public Command preSpinCommand(BooleanSupplier fuelReady) {
        return run(() -> {
            boolean shouldPrespin = !Constants.UltraShooterConstants.kEnableFMSAwarePreSpinLatch
                    || (GameData.isHubActiveExpanded(5.0)
                            && Landmarks.isInScoringZone(swerve.getPose()));
            if (shouldPrespin) {
                double physicsSpeed = calculateRequiredVelocityFPS(
                        swerve.getDistanceToHub(),
                        Constants.UltraShooterConstants.kFlywheelEfficiency,
                        Constants.UltraShooterConstants.kDragCoefficient,
                        Constants.UltraShooterConstants.kBallMassLbs);
                setTarget(physicsSpeed * Constants.UltraShooterConstants.kPreSpinFraction);
            } else {
                setTarget(0);
            }
        }).withName("PreSpin");
    }

    /** Holds the current target for the given duration, then stops. */
    public Command holdSpeedCommand(double seconds) {
        return run(() -> {}).withTimeout(seconds).andThen(runOnce(this::stop));
    }

    /** Continuously spins down to zero (for endgame). */
    public Command spinDownCommand() {
        return run(() -> setTarget(0)).withName("SpinDown");
    }

    /**
     * Ramps the flywheel to the pre-spin speed over 1 second, then ends.
     * The pre-spin target is recalculated each cycle so it tracks the current
     * distance. After this command finishes, {@link #preSpinCommand} takes over
     * at the same speed with no discontinuity.
     */
    public Command idleDownCommand() {
        return run(() -> {
            double preSpinSpeed = calculateRequiredVelocityFPS(swerve.getDistanceToHub())
                    * Constants.UltraShooterConstants.kPreSpinFraction;
            setTarget(preSpinSpeed);
        }).withTimeout(1.0).withName("IdleDown");
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal periodic helpers
    // ─────────────────────────────────────────────────────────────────────────

    private void updateDistanceBuffer() {
        double newest = swerve.getDistanceToHub();
        distanceBufferSum -= distanceBuffer[distanceBufferIndex];
        distanceBuffer[distanceBufferIndex] = newest;
        distanceBufferSum += newest;
        distanceBufferIndex = (distanceBufferIndex + 1) % DISTANCE_AVG_SAMPLES;
    }

    /** Speed above which the robot is considered "moving" and the instant distance is used (ft/s). */
    private static final double MOVING_SPEED_THRESHOLD_FPS = 1.0;

    /**
     * Returns the best distance estimate for physics calculations.
     * When the robot is stationary the 1-second rolling average is used to
     * reject odometry noise.  When moving, the rolling average lags the true
     * distance by up to ~0.5 s, so the instantaneous distance is used instead.
     */
    public double getAverageDistanceToHub() {
        return getAverageDistanceToHub(swerve.getFieldVelocity());
    }

    /** Overload that accepts a pre-fetched {@link ChassisSpeeds} to avoid a
     *  redundant {@code getFieldVelocity()} call when the caller already has it. */
    private double getAverageDistanceToHub(ChassisSpeeds fieldVel) {
        final double speedFps = Math.hypot(fieldVel.vxMetersPerSecond, fieldVel.vyMetersPerSecond)
                * METERS_TO_FEET;
        if (speedFps > MOVING_SPEED_THRESHOLD_FPS) {
            return swerve.getDistanceToHub();
        }
        return distanceBufferSum / DISTANCE_AVG_SAMPLES;
    }

    private void updateVelocityBuffer() {
        double newest = cachedVelocity;
        velocityBufferSum -= velocityBuffer[velocityBufferIndex];
        velocityBuffer[velocityBufferIndex] = newest;
        velocityBufferSum += newest;
        velocityBufferIndex =
                (velocityBufferIndex + 1) % Constants.UltraShooterConstants.kVelocityAvgSamples;
    }

    private void rampSetpoint() {
        if (rampedSetpoint < velocityTarget) {
            rampedSetpoint = Math.min(
                    rampedSetpoint + Constants.UltraShooterConstants.kRampUpRate, velocityTarget);
        } else if (rampedSetpoint > velocityTarget) {
            rampedSetpoint = velocityTarget; // coast down — no active braking
        }
    }

    /**
     * Re-configures all three SparkFlex controllers with a new kP value via the IO layer.
     * Uses kNoResetSafeParameters + kNoPersistParameters — the Pi JSON is the persistence layer.
     */
    private void applyKpToMotors(double kP) {
        io.setKp(kP);
        appliedKp = kP;
    }

    private void applyPID() {
        if (Math.abs(rampedSetpoint) > 3.0) {
            // Accumulate voltage bias only once the ramp has reached target.
            // Integrating during ramp-up would fight the ramp and cause overshoot.
            if (rampedSetpoint >= velocityTarget) {
                double error = rampedSetpoint - cachedVelocity;
                voltageBias += Constants.UltraShooterConstants.kVoltageBiasRate * error;
                voltageBias  = MathUtil.clamp(voltageBias,
                                              -Constants.UltraShooterConstants.kMaxVoltageBias,
                                               Constants.UltraShooterConstants.kMaxVoltageBias);
            }
            double ff = Constants.UltraShooterConstants.kS + rampedSetpoint * Constants.UltraShooterConstants.kV + voltageBias;
            io.setVelocity(rampedSetpoint, ff);
        } else {
            voltageBias = 0.0;  // Reset on stop — fresh calibration every shot.
            io.stop();
        }
    }

    private void updateNetworkTable() {
        Logger.recordOutput("UltraShooter/Velocity_fps",   getVelocity());
        Logger.recordOutput("UltraShooter/Velocity_RPM",  getVelocity() * 60.0 / WHEEL_CIRCUMFERENCE_FEET);
        Logger.recordOutput("UltraShooter/Target_fps",     velocityTarget);
        Logger.recordOutput("UltraShooter/Target_RPM",    velocityTarget * 60.0 / WHEEL_CIRCUMFERENCE_FEET);
        Logger.recordOutput("UltraShooter/Ramped_fps",     rampedSetpoint);
        Logger.recordOutput("UltraShooter/AvgVelocity_fps", getAverageVelocity());
        Logger.recordOutput("UltraShooter/Ready",          isReady());
        Logger.recordOutput("UltraShooter/VoltageBias_V",  voltageBias);

        final double effic = Constants.UltraShooterConstants.kFlywheelEfficiency;
        final double drag  = Constants.UltraShooterConstants.kDragCoefficient;
        final double mass  = Constants.UltraShooterConstants.kBallMassLbs;
        final double dist  = getAverageDistanceToHub();

        Logger.recordOutput("UltraShooter/Distance_ft",           swerve.getDistanceToHub() * METERS_TO_FEET);
        Logger.recordOutput("UltraShooter/AvgDistance_ft",        dist * METERS_TO_FEET);
        Logger.recordOutput("UltraShooter/LaunchAngle_deg",       Constants.UltraShooterConstants.kLaunchAngleDegrees);
        Logger.recordOutput("UltraShooter/HoodHeight_in",         Constants.UltraShooterConstants.kHoodHeightFromFloorInches);
        Logger.recordOutput("UltraShooter/HubHeight_in",          Constants.UltraShooterConstants.kHubCenterHeightFromFloorInches);
        Logger.recordOutput("UltraShooter/PrimaryCurrent_A",      inputs.leftCurrentAmps);
        Logger.recordOutput("UltraShooter/SecondaryCurrent_A",    inputs.middleCurrentAmps);
        Logger.recordOutput("UltraShooter/TertiaryCurrent_A",     inputs.rightCurrentAmps);
        Logger.recordOutput("UltraShooter/PiActive",              isPiResultFresh());

        // Publish average distance directly to NT so physics_engine.py can read it.
        // Logger.recordOutput() goes through AdvantageKit and may be published under a
        // different key prefix; this direct publish guarantees the Pi reads the right value.
        nt.getEntry("Avg Distance to Hub (ft)").setDouble(dist * METERS_TO_FEET);

        // Physics solver + trajectory visualization at ~10 Hz (every 5 cycles).
        // calcBallExitSpeedMps() runs a 60-step Euler binary search — too expensive for 50 Hz.
        // Cached results are logged every cycle so telemetry stays continuous.
        if (++vizSkipCounter % 5 == 0) {
            // Solve once — derive physFPS, TOF, and visualization v0 from a single binary search.
            final double v0Mps = calcBallExitSpeedMps(dist, drag, mass);
            cachedTelemetryPhysicsFps = v0Mps > 0 ? (v0Mps / Math.max(effic, 0.001)) * METERS_TO_FEET : 0;
            if (v0Mps <= 0) {
                cachedTelemetryTof = 0;
            } else if (drag <= 0) {
                final double d = dist + SHOOTER_OFFSET_M;
                cachedTelemetryTof = d > 0 ? d / (v0Mps * Math.cos(LAUNCH_ANGLE_RAD)) : 0;
            } else {
                final double ballMassKg  = mass * 0.453592;
                final double dragPerMass = drag / Math.max(ballMassKg, 0.001);
                final double d           = dist + SHOOTER_OFFSET_M;
                cachedTelemetryTof = simulateTimeOfFlight(v0Mps, d, LAUNCH_ANGLE_RAD, dragPerMass);
            }
            final double v0Tuned = v0Mps * (1.0 + interpolateOffsetFraction(dist));
            final double dpm     = drag > 0 ? drag / Math.max(mass, 0.001) : 0;
            updateTrajectoryVisualization(dist, v0Mps, v0Tuned, dpm);
        }

        Logger.recordOutput("UltraShooter/Physics_fps",    cachedTelemetryPhysicsFps);
        Logger.recordOutput("UltraShooter/TimeOfFlight_s", cachedTelemetryTof);
    }

    /**
     * Repositions the hub/fuel roots and redraws both trajectory arcs inside
     * {@link #trajCanvas}.
     *
     * <p>White arc: ideal physics (efficiency = 1) — where the ball must go to
     * land in the hub under perfect conditions.
     * Red arc: tuned shot — ideal v0 scaled by the parabolic offset percentage,
     * showing the actual trajectory when the tuner is non-zero.
     */
    private void updateTrajectoryVisualization(
            double distM, double v0IdealMps, double v0TunedMps, double dragPerMass) {
        final double hubH  = Constants.UltraShooterConstants.kHubCenterHeightFromFloorInches / 12.0;
        final double distFt = distM * METERS_TO_FEET;

        // Reposition the hub box, stand, and fuel roots based on current distance.
        trajHubStandRoot.setPosition(distFt, 0.0);
        trajHubBoxRoot  .setPosition(distFt, hubH - VIZ_HUB_OPEN_FT / 2.0);
        trajFuelRoot    .setPosition(distFt, hubH);
        trajHubStand.setLength(Math.max(0.033, hubH - VIZ_HUB_OPEN_FT / 2.0));

        if (v0IdealMps < 0.5 || distM < 0.1) {
            for (MechanismLigament2d seg : trajSegs)      seg.setLength(0);
            for (MechanismLigament2d seg : trajTunedSegs) seg.setLength(0);
            return;
        }

        // Draw the ideal (white) arc.
        applyTrajectoryToSegments(
                trajSegs, v0IdealMps, LAUNCH_ANGLE_RAD, dragPerMass);

        // Draw the tuned (red) arc — only visible when offset is non-zero.
        if (Math.abs(v0TunedMps - v0IdealMps) > 0.05) {
            applyTrajectoryToSegments(
                    trajTunedSegs, v0TunedMps, LAUNCH_ANGLE_RAD, dragPerMass);
        } else {
            for (MechanismLigament2d seg : trajTunedSegs) seg.setLength(0);
        }
    }

    /** Shared helper: maps trajectory points onto a ligament segment chain. */
    private void applyTrajectoryToSegments(
            MechanismLigament2d[] segs, double v0Mps, double angleRad, double dragPerMass) {
        final double[] pts = computeTrajectoryPoints(v0Mps, angleRad, dragPerMass, TRAJ_SEG_COUNT + 1);
        double prevWorldDeg = Constants.UltraShooterConstants.kLaunchAngleDegrees;
        for (int i = 0; i < segs.length; i++) {
            final int base = i * 2;
            final int next = base + 2;
            if (next + 1 >= pts.length) {
                segs[i].setLength(0);
                continue;
            }
            final double dx       = (pts[next]     - pts[base])     * METERS_TO_FEET;
            final double dy       = (pts[next + 1] - pts[base + 1]) * METERS_TO_FEET;
            final double len      = Math.sqrt(dx * dx + dy * dy);
            final double worldDeg = Math.toDegrees(Math.atan2(dy, dx));
            final double relDeg   = (i == 0) ? worldDeg : worldDeg - prevWorldDeg;
            segs[i].setAngle(relDeg);
            segs[i].setLength(len);
            prevWorldDeg = worldDeg;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // SubsystemBase overrides
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("UltraShooter", inputs);

        cachedVelocity = (inputs.leftVelocityFPS
                        + inputs.middleVelocityFPS
                        + inputs.rightVelocityFPS) / 3.0;
        // updatePiStaleness(); // Pi physics engine (disabled)
        updateDistanceBuffer();
        updateVelocityBuffer();
        rampSetpoint();
        applyPID();
        updateNetworkTable();

        // if (DriverStation.isDisabled()) { // Pi live kP update path (disabled)
        //     double newKp = shooterTuner.getKp();
        //     if (newKp != appliedKp) {
        //         applyKpToMotors(newKp);
        //     }
        // }
        LoggedTracer.record("UltraShooter");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Left ft/s",      () -> inputs.leftVelocityFPS,    null);
        builder.addDoubleProperty("Middle ft/s",    () -> inputs.middleVelocityFPS,  null);
        builder.addDoubleProperty("Right ft/s",     () -> inputs.rightVelocityFPS,   null);
        builder.addDoubleProperty("Left Current",   () -> inputs.leftCurrentAmps,    null);
        builder.addDoubleProperty("Middle Current", () -> inputs.middleCurrentAmps,  null);
        builder.addDoubleProperty("Right Current",  () -> inputs.rightCurrentAmps,   null);
        builder.addDoubleProperty("Target ft/s",    () -> velocityTarget,            null);
        builder.addDoubleProperty("Ramped ft/s",    () -> rampedSetpoint,            null);
        builder.addDoubleProperty("Avg ft/s",       this::getAverageVelocity,        null);
        builder.addDoubleProperty("Physics ft/s",
                () -> calculateRequiredVelocityFPS(swerve.getDistanceToHub()),       null);
        builder.addBooleanProperty("Ready",         this::isReady,                   null);
        builder.addStringProperty("Command",
                () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none", null);
    }
}
