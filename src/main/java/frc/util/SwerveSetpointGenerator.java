package frc.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Kinematic-constraint-enforcing swerve setpoint generator.
 *
 * <p>Each call to {@link #generateSetpoint} smooths the requested {@link ChassisSpeeds} so that:
 * <ul>
 *   <li>No module exceeds {@link KinematicLimits#maxDriveVelocity}.</li>
 *   <li>No module accelerates faster than {@link KinematicLimits#maxDriveAcceleration}.</li>
 *   <li>No module steers faster than {@link KinematicLimits#maxSteeringVelocity}.</li>
 * </ul>
 *
 * <p>Adapted from frc5687/2023-robot (originally from Team 254).
 * Geometry helpers from frc5687's {@code GeometryUtil} are inlined to avoid external dependency.
 */
public class SwerveSetpointGenerator {

    // ── Kinematic limit specification ─────────────────────────────────────────

    public static class KinematicLimits {
        /** Maximum wheel speed (m/s). */
        public final double maxDriveVelocity;
        /** Maximum wheel linear acceleration (m/s²). */
        public final double maxDriveAcceleration;
        /** Maximum azimuth angular velocity (rad/s). */
        public final double maxSteeringVelocity;

        public KinematicLimits(double maxDriveVelocity, double maxDriveAcceleration,
                double maxSteeringVelocity) {
            this.maxDriveVelocity    = maxDriveVelocity;
            this.maxDriveAcceleration = maxDriveAcceleration;
            this.maxSteeringVelocity = maxSteeringVelocity;
        }
    }

    // ── Fields ────────────────────────────────────────────────────────────────

    private static final double EPSILON = 1e-9;

    private final SwerveDriveKinematics kinematics;
    private final Translation2d[]       modulePositions;

    public SwerveSetpointGenerator(SwerveDriveKinematics kinematics,
            Translation2d[] modulePositions) {
        this.kinematics      = kinematics;
        this.modulePositions = modulePositions;
    }

    // ── Geometry helpers (inlined from frc5687 GeometryUtil) ──────────────────

    /** True if {@code speeds} is within epsilon of a full stop on all axes. */
    private static boolean isZero(ChassisSpeeds s) {
        return epsilonEquals(s.vxMetersPerSecond,       0)
            && epsilonEquals(s.vyMetersPerSecond,       0)
            && epsilonEquals(s.omegaRadiansPerSecond,   0);
    }

    /** Flip a heading by 180°. */
    private static Rotation2d flip(Rotation2d r) {
        return r.rotateBy(Rotation2d.fromRadians(Math.PI));
    }

    /** Invert (negate) a rotation. */
    private static Rotation2d inverse(Rotation2d r) {
        return r.unaryMinus();
    }

    // ── Math helpers ──────────────────────────────────────────────────────────

    private static boolean epsilonEquals(double a, double b) {
        return Math.abs(a - b) <= EPSILON;
    }

    private static boolean flipHeading(Rotation2d prevToGoal) {
        return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
    }

    private static double unwrapAngle(double ref, double angle) {
        double diff = angle - ref;
        if      (diff >  Math.PI) return angle - 2.0 * Math.PI;
        else if (diff < -Math.PI) return angle + 2.0 * Math.PI;
        else                      return angle;
    }

    @FunctionalInterface
    private interface Function2d { double f(double x, double y); }

    // Regula-Falsi root finder (used to find max feasible interpolation fraction).
    private static double findRoot(Function2d fn,
            double x0, double y0, double f0,
            double x1, double y1, double f1,
            int iters) {
        if (iters < 0 || epsilonEquals(f0, f1)) return 1.0;
        double s  = Math.max(0.0, Math.min(1.0, -f0 / (f1 - f0)));
        double xg = (x1 - x0) * s + x0;
        double yg = (y1 - y0) * s + y0;
        double fg = fn.f(xg, yg);
        if (Math.signum(f0) == Math.signum(fg)) {
            return s + (1.0 - s) * findRoot(fn, xg, yg, fg, x1, y1, f1, iters - 1);
        } else {
            return s * findRoot(fn, x0, y0, f0, xg, yg, fg, iters - 1);
        }
    }

    private static double findSteeringMaxS(
            double x0, double y0, double f0,
            double x1, double y1, double f1,
            double maxDev, int maxIter) {
        f1 = unwrapAngle(f0, f1);
        double diff = f1 - f0;
        if (Math.abs(diff) <= maxDev) return 1.0;
        double offset = f0 + Math.signum(diff) * maxDev;
        Function2d fn = (x, y) -> unwrapAngle(f0, Math.atan2(y, x)) - offset;
        return findRoot(fn, x0, y0, f0 - offset, x1, y1, f1 - offset, maxIter);
    }

    private static double findDriveMaxS(
            double x0, double y0, double f0,
            double x1, double y1, double f1,
            double maxStep, int maxIter) {
        if (Math.abs(f1 - f0) <= maxStep) return 1.0;
        double offset = f0 + Math.signum(f1 - f0) * maxStep;
        Function2d fn = (x, y) -> Math.hypot(x, y) - offset;
        return findRoot(fn, x0, y0, f0 - offset, x1, y1, f1 - offset, maxIter);
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Generate a new constrained setpoint.
     *
     * @param limits       Kinematic limits to enforce.
     * @param prevSetpoint Setpoint from the previous cycle (tracks momentum state).
     * @param desired      Unconstrained desired chassis speeds.
     * @param dt           Loop period in seconds (typically 0.02).
     * @return A setpoint that satisfies all limits while converging toward {@code desired}.
     */
    public SwerveSetpoint generateSetpoint(KinematicLimits limits, SwerveSetpoint prevSetpoint,
            ChassisSpeeds desired, double dt) {
        final int n = modulePositions.length;

        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(desired);
        if (limits.maxDriveVelocity > 0) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, limits.maxDriveVelocity);
            desired = kinematics.toChassisSpeeds(desiredStates);
        }

        // If desired is a full stop, preserve current azimuth angles.
        boolean needToSteer = true;
        if (isZero(desired)) {
            needToSteer = false;
            for (int i = 0; i < n; i++) {
                desiredStates[i].angle               = prevSetpoint.moduleStates[i].angle;
                desiredStates[i].speedMetersPerSecond = 0.0;
            }
        }

        // Decompose each module into Vx/Vy components.
        double[]     prevVx      = new double[n];
        double[]     prevVy      = new double[n];
        Rotation2d[] prevHeading = new Rotation2d[n];
        double[]     desVx       = new double[n];
        double[]     desVy       = new double[n];
        Rotation2d[] desHeading  = new Rotation2d[n];
        boolean allShouldFlip = true;

        for (int i = 0; i < n; i++) {
            prevVx[i] = prevSetpoint.moduleStates[i].angle.getCos()
                      * prevSetpoint.moduleStates[i].speedMetersPerSecond;
            prevVy[i] = prevSetpoint.moduleStates[i].angle.getSin()
                      * prevSetpoint.moduleStates[i].speedMetersPerSecond;
            prevHeading[i] = prevSetpoint.moduleStates[i].angle;
            if (prevSetpoint.moduleStates[i].speedMetersPerSecond < 0)
                prevHeading[i] = flip(prevHeading[i]);

            desVx[i] = desiredStates[i].angle.getCos() * desiredStates[i].speedMetersPerSecond;
            desVy[i] = desiredStates[i].angle.getSin() * desiredStates[i].speedMetersPerSecond;
            desHeading[i] = desiredStates[i].angle;
            if (desiredStates[i].speedMetersPerSecond < 0) desHeading[i] = flip(desHeading[i]);

            if (allShouldFlip) {
                double req = Math.abs(inverse(prevHeading[i]).rotateBy(desHeading[i]).getRadians());
                if (req < Math.PI / 2.0) allShouldFlip = false;
            }
        }

        // If all modules need to flip direction, it's faster to stop, re-orient, then accelerate.
        if (allShouldFlip && !isZero(prevSetpoint.chassisSpeeds) && !isZero(desired)) {
            return generateSetpoint(limits, prevSetpoint, new ChassisSpeeds(), dt);
        }

        double dx = desired.vxMetersPerSecond        - prevSetpoint.chassisSpeeds.vxMetersPerSecond;
        double dy = desired.vyMetersPerSecond        - prevSetpoint.chassisSpeeds.vyMetersPerSecond;
        double dw = desired.omegaRadiansPerSecond    - prevSetpoint.chassisSpeeds.omegaRadiansPerSecond;

        // Find the minimum feasible interpolation fraction across all modules.
        double minS = 1.0;
        List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(n);
        double maxThetaStep = dt * limits.maxSteeringVelocity;

        for (int i = 0; i < n; i++) {
            if (!needToSteer) {
                overrideSteering.add(Optional.of(prevSetpoint.moduleStates[i].angle));
                continue;
            }
            overrideSteering.add(Optional.empty());

            if (epsilonEquals(prevSetpoint.moduleStates[i].speedMetersPerSecond, 0.0)) {
                if (epsilonEquals(desiredStates[i].speedMetersPerSecond, 0.0)) {
                    overrideSteering.set(i, Optional.of(prevSetpoint.moduleStates[i].angle));
                    continue;
                }
                Rotation2d rot = inverse(prevSetpoint.moduleStates[i].angle)
                        .rotateBy(desiredStates[i].angle);
                if (flipHeading(rot)) rot = rot.rotateBy(Rotation2d.fromRadians(Math.PI));
                double stepsNeeded = Math.abs(rot.getRadians()) / maxThetaStep;
                if (stepsNeeded <= 1.0) {
                    overrideSteering.set(i, Optional.of(desiredStates[i].angle));
                } else {
                    overrideSteering.set(i, Optional.of(
                        prevSetpoint.moduleStates[i].angle.rotateBy(
                            Rotation2d.fromRadians(Math.signum(rot.getRadians()) * maxThetaStep))));
                    minS = 0.0;
                }
                continue;
            }
            if (minS == 0.0) continue;

            double s = findSteeringMaxS(
                prevVx[i], prevVy[i], prevHeading[i].getRadians(),
                desVx[i],  desVy[i],  desHeading[i].getRadians(),
                maxThetaStep, 8);
            minS = Math.min(minS, s);
        }

        // Enforce drive acceleration limits.
        double maxVelStep = dt * limits.maxDriveAcceleration;
        for (int i = 0; i < n; i++) {
            if (minS == 0.0) break;
            double vxMs = minS == 1.0 ? desVx[i] : (desVx[i] - prevVx[i]) * minS + prevVx[i];
            double vyMs = minS == 1.0 ? desVy[i] : (desVy[i] - prevVy[i]) * minS + prevVy[i];
            double s = minS * findDriveMaxS(
                prevVx[i], prevVy[i], Math.hypot(prevVx[i], prevVy[i]),
                vxMs,      vyMs,      Math.hypot(vxMs, vyMs),
                maxVelStep, 10);
            minS = Math.min(minS, s);
        }

        ChassisSpeeds retSpeeds = new ChassisSpeeds(
            prevSetpoint.chassisSpeeds.vxMetersPerSecond       + minS * dx,
            prevSetpoint.chassisSpeeds.vyMetersPerSecond       + minS * dy,
            prevSetpoint.chassisSpeeds.omegaRadiansPerSecond   + minS * dw);

        SwerveModuleState[] retStates = kinematics.toSwerveModuleStates(retSpeeds);
        for (int i = 0; i < n; i++) {
            var maybeOverride = overrideSteering.get(i);
            if (maybeOverride.isPresent()) {
                Rotation2d override = maybeOverride.get();
                if (flipHeading(inverse(retStates[i].angle).rotateBy(override))) {
                    retStates[i].speedMetersPerSecond *= -1.0;
                }
                retStates[i].angle = override;
            }
            Rotation2d delta = inverse(prevSetpoint.moduleStates[i].angle)
                    .rotateBy(retStates[i].angle);
            if (flipHeading(delta)) {
                retStates[i].angle = flip(retStates[i].angle);
                retStates[i].speedMetersPerSecond *= -1.0;
            }
        }
        return new SwerveSetpoint(retSpeeds, retStates);
    }
}
