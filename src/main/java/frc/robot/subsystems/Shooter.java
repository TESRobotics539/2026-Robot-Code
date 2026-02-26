package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import java.util.List;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Shooter extends SubsystemBase {
    public enum Speed {
        SHOOT(5000),
        DASHBOARD(0);

        private final double rpm;

        private Speed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity angularVelocity() {
            return RPM.of(rpm);
        }
    }

    private static final double kNeoVortexFreeSpeed = 6784.0; // RPM
    private static final AngularVelocity kVelocityTolerance = RPM.of(100);

    private final SparkFlex leftMotor, middleMotor, rightMotor;
    private final RelativeEncoder leftEncoder, middleEncoder, rightEncoder;
    private final List<RelativeEncoder> encoders;
    private final SparkClosedLoopController leftController, middleController, rightController;

    private double targetRPM = 0.0;
    private double dashboardTargetRPM = 0.0;
    private boolean isVelocityMode = false;

    public Shooter() {
        leftMotor = new SparkFlex(Ports.kShooterLeft, MotorType.kBrushless);
        middleMotor = new SparkFlex(Ports.kShooterMiddle, MotorType.kBrushless);
        rightMotor = new SparkFlex(Ports.kShooterRight, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        middleEncoder = middleMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        encoders = List.of(leftEncoder, middleEncoder, rightEncoder);

        leftController = leftMotor.getClosedLoopController();
        middleController = middleMotor.getClosedLoopController();
        rightController = rightMotor.getClosedLoopController();

        configureMotor(leftMotor, leftController, false); // Inverted in REV client
        configureMotor(middleMotor, middleController, false);
        configureMotor(rightMotor, rightController, true);

        SmartDashboard.putData(this);
    }

    private void configureMotor(SparkFlex motor, SparkClosedLoopController controller, boolean inverted) {
        SparkFlexConfig config = new SparkFlexConfig();
        
        config.inverted(inverted);
        config.closedLoopRampRate(0.5);
       //config.idleMode(IdleMode.kCoast);
        //config.smartCurrentLimit(70); // Supply current limit
        //config.secondaryCurrentLimit(120); // Stator current limit
        
        // PID configuration
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.0002, 0.0008, 0.0) // kP, kI, kD for velocity control in RPM
            .velocityFF(12.0 / kNeoVortexFreeSpeed) // kV: 12 volts when requesting max RPM
            .iZone(0);
        
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // public void setRPM(double rpm) {
    //     targetRPM = rpm;
    //     isVelocityMode = true;
    //     leftController.setSetpoint(rpm, SparkFlex.ControlType.kVelocity);
    //     middleController.setSetpoint(rpm, SparkFlex.ControlType.kVelocity);
    //     rightController.setSetpoint(rpm, SparkFlex.ControlType.kVelocity);
    // }

    public void setPercentOutput(double percentOutput) {
        isVelocityMode = false;
        double voltage = percentOutput * 12.0;
        leftMotor.setVoltage(voltage);
        middleMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }
    
    public void set(Speed speed) {
        leftMotor.getClosedLoopController().setSetpoint(
            speed.angularVelocity().in(RPM),
            ControlType.kVelocity
        );
        middleMotor.getClosedLoopController().setSetpoint(
            speed.angularVelocity().in(RPM),
            ControlType.kVelocity
        );
        rightMotor.getClosedLoopController().setSetpoint(
            speed.angularVelocity().in(RPM),
            ControlType.kVelocity
        );
    }

    public void stop() {
        setPercentOutput(0.0);
    }

    public Command spinUpCommand() {
        return startEnd(() -> set(Speed.SHOOT), () -> stop());
        //return runOnce(() -> setRPM(rpm));
            //.andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }
    
    public Command spinUpCommand(Speed speed) {
        return startEnd(() -> set(speed), () -> stop());
        //return runOnce(() -> setRPM(rpm));
            //.andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command dashboardSpinUpCommand() {
        return defer(() -> spinUpCommand(Speed.DASHBOARD)); 
    }
 
    // public boolean isVelocityWithinTolerance() {
    //     if (!isVelocityMode) return false;
        
    //     return encoders.stream().allMatch(encoder -> {
    //         final AngularVelocity currentVelocity = RPM.of(encoder.getVelocity());
    //         final AngularVelocity targetVelocity = RPM.of(targetRPM);
    //         return currentVelocity.isNear(targetVelocity, kVelocityTolerance);
    //     });
    // }

    private void initSendable(SendableBuilder builder, SparkFlex motor, RelativeEncoder encoder, String name) {
        builder.addDoubleProperty(name + " RPM", () -> encoder.getVelocity(), null);
        builder.addDoubleProperty(name + " Output Current", () -> motor.getOutputCurrent(), null);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        initSendable(builder, leftMotor, leftEncoder, "Left");
        initSendable(builder, middleMotor, middleEncoder, "Middle");
        initSendable(builder, rightMotor, rightEncoder, "Right");
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Dashboard RPM", () -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
        builder.addDoubleProperty("Target RPM", () -> targetRPM, null);
    }
}
