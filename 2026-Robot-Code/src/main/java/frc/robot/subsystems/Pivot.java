package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Pivot extends SubsystemBase {

    private double targetPivotPosition = 0.0;
    private boolean usePercentOutput = false;

    private final SparkMax pivotMotor = new SparkMax(Ports.kPivot, MotorType.kBrushless);
    private final SparkClosedLoopController pivotController;

    private final RelativeEncoder encoder;
    private final AbsoluteEncoder absEncoder;

    public Pivot() {
        pivotController = pivotMotor.getClosedLoopController();
        configurePivotMotor();

        encoder = pivotMotor.getEncoder();
        absEncoder = pivotMotor.getAbsoluteEncoder();

        targetPivotPosition = 0;

        SmartDashboard.putData(this);
    }

    private void configurePivotMotor() {
        final SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(true)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12);

        config.smartCurrentLimit(15);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(2.5, 0, 5)
            .outputRange(-0.15, 0.4);

        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPercentOutput(double percentOutput) {
        usePercentOutput = true;
        pivotMotor.set(percentOutput);
    }

    public void setTarget(double position) {
        usePercentOutput = false;
        targetPivotPosition = position;
    }

    @Override
    public void periodic() {
        if (!usePercentOutput && targetPivotPosition != 0) {
            pivotController.setSetpoint(
                targetPivotPosition,
                ControlType.kPosition
            );
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Absolute Encoder Position", () -> absEncoder.getPosition(), null);
        builder.addDoubleProperty("Relative Encoder Position", () -> encoder.getPosition(), null);
        builder.addDoubleProperty("Target Position", () -> targetPivotPosition, null);
        builder.addDoubleProperty("Output Current (A)", () -> pivotMotor.getOutputCurrent(), null);
    }
}
