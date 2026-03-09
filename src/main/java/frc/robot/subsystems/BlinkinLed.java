package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GameData;
import frc.robot.Ports;

/**
 * Subsystem for the REV Blinkin LED Driver (REV-11-1105).
 *
 * The Blinkin is controlled via a PWM signal using WPILib's Spark class.
 * Pattern values range from -1.0 to 1.0 and correspond to entries in the
 * Blinkin LED pattern table (see REV-11-1105 User Manual).
 *
 * For Color 1 & 2 patterns, set the physical colors on the Blinkin hardware:
 *   - Color 1: White
 *   - Color 2: Green
 * Use the SET and MODE buttons on the Blinkin unit to configure these colors.
 */
public class BlinkinLed extends SubsystemBase {

    // REV Blinkin pattern values (PWM -1.0 to 1.0)
    // Source: REV-11-1105 Blinkin LED Driver User Manual pattern table
    public static final class Pattern {
        // Color 1 & 2 Patterns — requires Color 1/2 set on hardware (see class javadoc)
        public static final double SINELON_COLOR1_AND_2   = 0.37; // slow moving dot — Color 1 background, Color 2 pulse
        public static final double LARSON_SCANNER_COLOR1  = 0.05; // "Cylon eye" sweep in Color 1
        public static final double CHASE_COLOR1_AND_2     = 0.09; // running/chasing lights, Color 1 & 2
        public static final double HEARTBEAT_SLOW_COLOR1  = 0.17; // slow throb in Color 1
        public static final double HEARTBEAT_SLOW_COLOR2  = 0.19; // slow throb in Color 2
        public static final double BREATH_SLOW_COLOR1     = 0.23; // slow breath fade, Color 1
        public static final double STROBE_COLOR1          = 0.29; // fast strobe, Color 1
        public static final double STROBE_COLOR2          = 0.31; // fast strobe, Color 2

        // Fixed Palette Patterns (no hardware color setup required)
        public static final double SINELON_FOREST         = -0.11; // greens/dark, moving dot
        public static final double SINELON_OCEAN          = -0.13; // blues/teals, moving dot
        public static final double COLOR_WAVES_FOREST     = -0.19; // green color wave
        public static final double FIRE_MEDIUM            = -0.59; // fire effect
        public static final double RAINBOW_SLOW           = -0.99; // rainbow scroll

        // Solid Colors
        public static final double SOLID_WHITE            = 0.93;
        public static final double SOLID_GREEN            = 0.77;
        public static final double SOLID_BLACK            = 0.99; // LEDs off
    }

    // The pattern applied during normal robot operation
    // White background with slow-moving green pulse (Sinelon, Color 1 & 2)
    // Color 1 = White, Color 2 = Green — configure on the Blinkin hardware
    private static final double DEFAULT_PATTERN = Pattern.SINELON_COLOR1_AND_2;

    private final Spark blinkin;

    public BlinkinLed() {
        blinkin = new Spark(Ports.kBlinkinLed);
        setPattern(DEFAULT_PATTERN);
    }

    /**
     * Sets the LED pattern.
     *
     * @param pattern A value from -1.0 to 1.0 corresponding to a Blinkin pattern.
     *                Use constants from {@link Pattern} for readability.
     */
    public void setPattern(double pattern) {
        blinkin.set(pattern);
    }

    /** Restores the default white-with-green-dashes pattern. */
    public void setDefaultPattern() {
        setPattern(DEFAULT_PATTERN);
    }

    /** Turns off the LEDs. */
    public void off() {
        setPattern(Pattern.SOLID_BLACK);
    }

    /**
     * Sets the LED pattern based on the current game phase.
     * Strobes in the incoming phase color 5 seconds before each phase switch.
     * Color 1 (scoring phase active) or Color 2 (inactive phase).
     * Falls back to the default pattern outside of teleop.
     */
    public void setPhasePattern() {
        if (!DriverStation.isTeleopEnabled()) {
            setDefaultPattern();
            return;
        }
        if (GameData.isPhaseChangeSoon(5.0)) {
            // Flash the color of the INCOMING phase so drivers know what's coming
            setPattern(GameData.isHubActive() ? Pattern.STROBE_COLOR2 : Pattern.STROBE_COLOR1);
        } else if (GameData.isHubActive()) {
            setPattern(Pattern.HEARTBEAT_SLOW_COLOR1);
        } else {
            setPattern(Pattern.HEARTBEAT_SLOW_COLOR2);
        }
    }
}
