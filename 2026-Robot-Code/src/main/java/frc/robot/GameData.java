package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Utility class for reading and interpreting 2026 game-specific data.
 *
 * <p>Game data format: a single character — 'R' means the Red alliance's goal
 * is inactive first; 'B' means the Blue alliance's goal is inactive first.
 * The string is empty until approximately 3 seconds into the match.
 *
 * <p>During teleop, the hub alternates active/inactive in four 25-second shifts
 * based on which alliance scored more fuel in autonomous.
 */
public final class GameData {

    private GameData() {}

    /**
     * Returns whether your alliance's hub (goal) is currently active and able to score.
     *
     * <ul>
     *   <li>Always active during autonomous.</li>
     *   <li>Always active during the last 30 seconds (end game).</li>
     *   <li>During teleop, alternates in 25-second shifts based on autonomous result.</li>
     *   <li>Assumes active if game data has not yet arrived.</li>
     * </ul>
     */
    public static boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, assume hub is active (likely early in teleop).
        if (gameData.isEmpty()) {
            return true;
        }

        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // Invalid game data — assume hub is active.
                return true;
            }
        }

        // Shift 1 is active for blue if red won auto, or for red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift — hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game — hub always active.
            return true;
        }
    }

    /**
     * Like {@link #isHubActive()}, but expands each active window by {@code expandSeconds}
     * on both edges — allowing shooting {@code expandSeconds} before your alliance's phase
     * begins and {@code expandSeconds} after it ends.
     *
     * <p>Implementation: the hub is considered active if it is active now, was active
     * {@code expandSeconds} ago, or will be active in {@code expandSeconds}.
     *
     * @param expandSeconds seconds to add to each edge of every active window
     */
    public static boolean isHubActiveExpanded(double expandSeconds) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled()) return false;

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) return true;

        boolean redInactiveFirst;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> { return true; }
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        // Active if the hub is active now, was active expandSeconds ago, or will be active soon.
        return isShiftActiveAt(matchTime, shift1Active)
            || isShiftActiveAt(matchTime + expandSeconds, shift1Active)
            || isShiftActiveAt(matchTime - expandSeconds, shift1Active);
    }

    /**
     * Returns true if the hub active state will change within {@code withinSeconds}.
     * Only meaningful during teleop; returns false otherwise or if game data is unavailable.
     */
    public static boolean isPhaseChangeSoon(double withinSeconds) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;
        if (!DriverStation.isTeleopEnabled()) return false;

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) return false;

        boolean redInactiveFirst;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> { return false; }
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        // matchTime counts down; state in withinSeconds is at matchTime - withinSeconds
        return isShiftActiveAt(matchTime, shift1Active)
            != isShiftActiveAt(matchTime - withinSeconds, shift1Active);
    }

    /**
     * Evaluates shift activity for a given countdown time, without any expansion.
     * Used internally by {@link #isHubActiveExpanded}.
     */
    private static boolean isShiftActiveAt(double matchTime, boolean shift1Active) {
        if (matchTime > 130) return true;       // Transition
        if (matchTime > 105) return shift1Active;  // Shift 1
        if (matchTime > 80)  return !shift1Active; // Shift 2
        if (matchTime > 55)  return shift1Active;  // Shift 3
        if (matchTime > 30)  return !shift1Active; // Shift 4
        return true;                               // End game
    }
}
