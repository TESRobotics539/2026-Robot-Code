package frc.util;

/**
 * First-order IIR (exponential moving average) low-pass filter.
 *
 * <p>Output = alpha * input + (1 - alpha) * previousOutput
 *
 * <p>alpha near 1.0 → fast response, little filtering (passes high-frequency noise).
 * alpha near 0.0 → slow response, heavy filtering (rejects transients like bump spikes).
 * Typical bump-rejection value: 0.1 – 0.2.
 */
public class LowPassFilter {
    private final double alpha;
    private double output;
    private boolean initialized = false;

    /**
     * @param alpha Smoothing factor [0, 1]. Lower = heavier filtering.
     */
    public LowPassFilter(double alpha) {
        this.alpha = alpha;
    }

    /** Feed a new sample into the filter and return the smoothed output. */
    public double calculate(double input) {
        if (!initialized) {
            output = input;
            initialized = true;
        } else {
            output = alpha * input + (1.0 - alpha) * output;
        }
        return output;
    }

    /** Return the last filtered value without updating. */
    public double get() {
        return output;
    }

    /** Reset so the next call to {@link #calculate} re-seeds the filter. */
    public void reset() {
        initialized = false;
    }
}
