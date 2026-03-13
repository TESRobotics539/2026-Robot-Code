package frc.robot.subsystems.iodiagnostics;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.Ports;

public class FloorIOReal implements FloorIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public FloorIOReal() {
        motor = new SparkMax(Ports.kFloor, MotorType.kBrushless);
        encoder = motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(Constants.FloorConstants.kIdleMode);

        config.signals
            .primaryEncoderPositionPeriodMs(500)
            .primaryEncoderVelocityPeriodMs(100);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(FloorIOInputs inputs) {
        inputs.velocityRPM = encoder.getVelocity();
        inputs.outputCurrentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
