package frc.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

/**
 * Lightweight post-match CSV logger that writes to USB on robot disable.
 *
 * <h2>Usage</h2>
 * <ol>
 *   <li>During robot init, call {@link #addColumn(String)} for each metric you want to track.
 *       The first column is always "Timestamp (s)" added automatically.</li>
 *   <li>At the top of each periodic cycle, call {@link #newRow()}.</li>
 *   <li>Call {@link #record(String, double)} (or {@link #set(int, double)}) for each metric.</li>
 *   <li>In {@code Robot.disabledInit()}, call {@link #flush()} to write the CSV to USB.</li>
 * </ol>
 *
 * <p>Data is buffered in RAM (up to {@link #MAX_ROWS} rows ≈ 60 s at 50 Hz) and written
 * atomically on disable. If the USB drive is absent the error is reported to Driver Station
 * and the buffer is cleared for the next match period.
 *
 * <p>Inspired by frc5687/2020-robot {@code MetricTracker}.
 */
public class MetricTracker {

    private static final String USB_PATH = "/media/sda1/";
    /** Maximum buffered rows before older data is silently dropped. */
    public static final int MAX_ROWS = 3000; // ~60 s at 50 Hz

    // Eager initialization — safe for single-threaded FRC use and avoids a null-check
    // data race if getInstance() were ever called from multiple threads concurrently.
    private static final MetricTracker instance = new MetricTracker();

    /** Returns the singleton instance. */
    public static MetricTracker getInstance() {
        return instance;
    }

    // ── Internal state ────────────────────────────────────────────────────────

    private final List<String>   headers  = new ArrayList<>();
    private final List<double[]> rows     = new ArrayList<>(MAX_ROWS);
    private double[] currentRow;
    private boolean  headersFrozen = false;
    private int      timestampCol  = -1;

    private MetricTracker() {
        timestampCol = addColumn("Timestamp (s)");
    }

    // ── Column registration ───────────────────────────────────────────────────

    /**
     * Register a metric column. Must be called before the first {@link #newRow()}.
     *
     * @param name Column header label.
     * @return Column index for use with {@link #set(int, double)}.
     */
    public int addColumn(String name) {
        if (headersFrozen) {
            DriverStation.reportError(
                "MetricTracker: cannot add column \"" + name + "\" after first newRow()", false);
            return -1;
        }
        headers.add(name);
        return headers.size() - 1;
    }

    // ── Per-cycle API ─────────────────────────────────────────────────────────

    /**
     * Commit the current row and start a new one. Call once per periodic cycle,
     * before any {@link #record} or {@link #set} calls for that cycle.
     */
    public void newRow() {
        if (!headersFrozen) {
            headersFrozen = true;
            currentRow = new double[headers.size()];
        }
        if (rows.size() < MAX_ROWS && currentRow != null) {
            rows.add(currentRow);
        }
        currentRow = new double[headers.size()];
        if (timestampCol >= 0) currentRow[timestampCol] = Timer.getFPGATimestamp();
    }

    /**
     * Write a value into the current row at the given column index.
     * Prefer this over {@link #record(String, double)} in hot paths to avoid string lookup.
     */
    public void set(int col, double value) {
        if (currentRow != null && col >= 0 && col < currentRow.length) {
            currentRow[col] = value;
        }
    }

    /**
     * Record a metric by column name. Slightly slower than {@link #set(int, double)}
     * due to linear name lookup — use column indices in periodic() for efficiency.
     */
    public void record(String name, double value) {
        set(headers.indexOf(name), value);
    }

    // ── Flush to USB ──────────────────────────────────────────────────────────

    /**
     * Write all buffered rows to a timestamped CSV on the USB drive, then clear the buffer.
     * Call from {@code Robot.disabledInit()}.
     */
    public void flush() {
        if (rows.isEmpty()) return;
        String filename = USB_PATH + "metrics_" + (long) Timer.getFPGATimestamp() + ".csv";
        try (PrintWriter pw = new PrintWriter(new FileWriter(filename))) {
            pw.println(String.join(",", headers));
            for (double[] row : rows) {
                StringBuilder sb = new StringBuilder();
                for (int i = 0; i < row.length; i++) {
                    if (i > 0) sb.append(',');
                    sb.append(row[i]);
                }
                pw.println(sb);
            }
            DriverStation.reportWarning(
                "MetricTracker: wrote " + rows.size() + " rows to " + filename, false);
        } catch (IOException e) {
            DriverStation.reportError(
                "MetricTracker: failed to write " + filename + ": " + e.getMessage(), false);
        }
        rows.clear();
    }
}
