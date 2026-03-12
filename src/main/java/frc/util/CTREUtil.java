package frc.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/**
 * Retry wrapper for CTRE Phoenix 6 device configuration.
 *
 * <p>CAN bus {@code .apply()} calls can silently fail on a busy bus. This utility
 * retries up to {@link #MAX_RETRIES} times with a 20 ms delay between attempts and
 * reports persistent failures to the Driver Station.
 *
 * <p>Adapted from frc5687/2025-robot {@code CTREUtil}.
 */
public final class CTREUtil {

    private static final int MAX_RETRIES = 5;

    private CTREUtil() {}

    private static StatusCode retry(Supplier<StatusCode> fn, int deviceId, String type) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < MAX_RETRIES; i++) {
            status = fn.get();
            if (status.isOK()) return status;
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        if (!status.isOK()) {
            DriverStation.reportError(
                String.format("CTREUtil: failed to configure %s (ID %d) after %d attempts: %s",
                    type, deviceId, MAX_RETRIES, status),
                true);
        }
        return status;
    }

    /** Apply a full {@link TalonFXConfiguration} with retry. */
    public static StatusCode applyConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return retry(() -> motor.getConfigurator().apply(config), motor.getDeviceID(), "TalonFX");
    }

    /** Apply a full {@link CANcoderConfiguration} with retry. */
    public static StatusCode applyConfiguration(CANcoder cancoder, CANcoderConfiguration config) {
        return retry(() -> cancoder.getConfigurator().apply(config), cancoder.getDeviceID(), "CANcoder");
    }

    /** Refresh a {@link TalonFXConfiguration} from device with retry. */
    public static StatusCode refreshConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return retry(() -> motor.getConfigurator().refresh(config), motor.getDeviceID(), "TalonFX");
    }
}
