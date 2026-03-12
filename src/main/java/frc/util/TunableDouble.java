package frc.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * NT-backed tunable double that detects when its value changes.
 *
 * <p>Publish to NetworkTables under {@code <table>/<key>} and update live during a match
 * without redeploying code. Call {@link #hasChanged()} to check whether the value was
 * updated since the last call, then reconfigure the relevant controller.
 *
 * <p>Adapted from frc5687/2025-robot {@code TunableDouble}.
 */
public class TunableDouble {

    private final double          _defaultValue;
    private final NetworkTableEntry _entry;
    private double  _value;
    private double  _lastValue;
    private boolean _hasChanged;

    /**
     * @param table        NetworkTable group name (e.g. {@code "PathPlanner"}).
     * @param key          Entry key within that table (e.g. {@code "TranslationkP"}).
     * @param defaultValue Initial value, used when no NT override exists.
     */
    public TunableDouble(String table, String key, double defaultValue) {
        _defaultValue = defaultValue;
        _value        = defaultValue;
        _lastValue    = defaultValue;
        _entry        = NetworkTableInstance.getDefault().getTable(table).getEntry(key);
        _entry.setDouble(defaultValue);
        _hasChanged   = false;
    }

    /**
     * Returns the current value, pulling any NT update first.
     * Marks the value as changed if NT differs from the cached value.
     */
    public double get() {
        _lastValue = _value;
        double ntValue = _entry.getDouble(_value);
        if (ntValue != _value) {
            _value      = ntValue;
            _hasChanged = true;
        }
        return _value;
    }

    /**
     * Returns {@code true} if the value changed since the last call to this method,
     * then resets the flag. Typical usage:
     * <pre>{@code
     * if (kP.hasChanged() || kD.hasChanged()) { controller.setP(kP.get()); }
     * }</pre>
     */
    public boolean hasChanged() {
        get(); // ensure _hasChanged is current
        boolean changed = _hasChanged;
        _hasChanged = false;
        return changed;
    }

    /** The value as of the previous {@link #get()} call. */
    public double getLastValue() { return _lastValue; }

    /** Programmatically set a new value and push it to NT. */
    public void set(double value) {
        if (_value != value) {
            _lastValue  = _value;
            _value      = value;
            _hasChanged = true;
            _entry.setDouble(value);
        }
    }

    /** Restore the default value. */
    public void reset() { set(_defaultValue); }

    public double getDefault() { return _defaultValue; }
}
