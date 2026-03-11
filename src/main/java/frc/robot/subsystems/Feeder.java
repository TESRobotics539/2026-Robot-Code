package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Feeder extends SubsystemBase {
    public enum Speed {
        FEED(Constants.FeederConstants.kFeedRPM),
        REVERSE(Constants.FeederConstants.kReverseRPM);

        private final double rpm;

        private Speed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity angularVelocity() {
            return RPM.of(rpm);
        }
    }

    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public Feeder() {
        motor = new SparkMax(Ports.kFeeder, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(Constants.FeederConstants.kSmartCurrentLimit);

        config.closedLoop
            .pid(Constants.FeederConstants.kP, Constants.FeederConstants.kI, Constants.FeederConstants.kD)
            .velocityFF(Constants.FeederConstants.kVelocityFF);

        // Position is never used; velocity is needed for closed-loop but current is dashboard-only.
        config.signals
            .primaryEncoderPositionPeriodMs(500);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        encoder = motor.getEncoder();
        
        SmartDashboard.putData(this);
    }

    public void set(Speed speed) {
        motor.getClosedLoopController().setSetpoint(
            speed.angularVelocity().in(RPM),
            ControlType.kVelocity
        );
    }

    public void setPercentOutput(double percentOutput) {
        motor.set(percentOutput);
    }

    public Command feedCommand() {
        return startEnd(() -> set(Speed.FEED), () -> setPercentOutput(0));
    }
    
    public Command reverseCommand() {
        return startEnd(() -> set(Speed.REVERSE), () -> setPercentOutput(0));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("RPM", () -> encoder.getVelocity(), null);
        builder.addDoubleProperty("Output Current", () -> motor.getOutputCurrent(), null);
    }
}
