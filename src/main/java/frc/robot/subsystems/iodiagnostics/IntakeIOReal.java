package frc.robot.subsystems.iodiagnostics;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.Ports;

public class IntakeIOReal implements IntakeIO {
    private final SparkMax pivotMotor;
    private final SparkClosedLoopController pivotController;
    private final AbsoluteEncoder absEncoder;
    private final RelativeEncoder encoder;

    private final SparkFlex rollerMotor;
    private final SparkClosedLoopController rollerController;

    public IntakeIOReal() {
        pivotMotor = new SparkMax(Ports.kIntakePivot, MotorType.kBrushless);
        pivotController = pivotMotor.getClosedLoopController();
        absEncoder = pivotMotor.getAbsoluteEncoder();
        encoder = pivotMotor.getEncoder();
        configurePivotMotor();

        rollerMotor = new SparkFlex(Ports.kIntakeRollers, MotorType.kBrushless);
        rollerController = rollerMotor.getClosedLoopController();
        configureRollerMotor();
    }

    private void configurePivotMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(true)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12);

        config.smartCurrentLimit(80)
            .secondaryCurrentLimit(120);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(Constants.IntakeConstants.kPivotP, Constants.IntakeConstants.kPivotI, Constants.IntakeConstants.kPivotD)
            .outputRange(Constants.IntakeConstants.kPivotOutputRangeMin, Constants.IntakeConstants.kPivotOutputRangeMax);

        config.signals
            .primaryEncoderPositionPeriodMs(500)
            .primaryEncoderVelocityPeriodMs(500)
            .absoluteEncoderPositionPeriodMs(20);

        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void configureRollerMotor() {
        SparkFlexConfig config = new SparkFlexConfig();

        config.inverted(true);
        config.idleMode(Constants.IntakeConstants.kRollerIdleMode);
        config.smartCurrentLimit(80);
        config.secondaryCurrentLimit(120);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.IntakeConstants.kRollerP, Constants.IntakeConstants.kRollerI, Constants.IntakeConstants.kRollerD)
            .velocityFF(12.0 / Constants.IntakeConstants.kRollerFreeSpeedRPM);

        config.signals
            .primaryEncoderPositionPeriodMs(500);

        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.pivotAbsEncoderPosition = absEncoder.getPosition();
        inputs.pivotRelEncoderPosition = encoder.getPosition();
        inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();
        inputs.rollerVelocityRPM = rollerMotor.getEncoder().getVelocity();
        inputs.rollerCurrentAmps = rollerMotor.getOutputCurrent();
    }

    @Override
    public void setPivotSetpoint(double position) {
        pivotController.setSetpoint(position, ControlType.kPosition);
    }

    @Override
    public void setPivotPercentOutput(double percent) {
        pivotMotor.set(percent);
    }

    @Override
    public void setPivotCoastMode(boolean coast) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
        pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setRollerRPM(double rpm) {
        rollerController.setSetpoint(rpm, ControlType.kVelocity);
    }

    @Override
    public void stopRoller() {
        rollerMotor.set(0);
    }
}
