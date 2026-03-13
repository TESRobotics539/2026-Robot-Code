package frc.robot.subsystems.iodiagnostics;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.Ports;

public class HangerIOReal implements HangerIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public HangerIOReal() {
        motor = new SparkMax(Ports.kHanger, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true)
            .idleMode(Constants.HangerConstants.kIdleMode)
            .smartCurrentLimit(Constants.HangerConstants.kSmartCurrentLimit)
            .secondaryCurrentLimit(Constants.HangerConstants.kSecondaryCurrentLimit);

        config.signals
            .primaryEncoderVelocityPeriodMs(500);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
        encoder.setPosition(0);
    }

    @Override
    public void updateInputs(HangerIOInputs inputs) {
        inputs.encoderPositionRot = encoder.getPosition();
        inputs.outputCurrentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setPercentOutput(double percent) {
        motor.set(percent);
    }

    @Override
    public void resetEncoder() {
        encoder.setPosition(0);
    }
}
