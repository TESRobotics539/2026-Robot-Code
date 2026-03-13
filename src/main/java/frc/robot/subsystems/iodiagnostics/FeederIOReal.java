package frc.robot.subsystems.iodiagnostics;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.Ports;

public class FeederIOReal implements FeederIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public FeederIOReal() {
        motor = new SparkMax(Ports.kFeeder, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(Constants.FeederConstants.kIdleMode);
        config.smartCurrentLimit(Constants.FeederConstants.kSmartCurrentLimit);

        config.closedLoop
            .pid(Constants.FeederConstants.kP, Constants.FeederConstants.kI, Constants.FeederConstants.kD)
            .velocityFF(Constants.FeederConstants.kVelocityFF);

        config.signals
            .primaryEncoderPositionPeriodMs(500);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        encoder = motor.getEncoder();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.velocityRPM = encoder.getVelocity();
        inputs.outputCurrentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setVelocityRPM(double rpm) {
        motor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }

    @Override
    public void setPercentOutput(double percent) {
        motor.set(percent);
    }
}
