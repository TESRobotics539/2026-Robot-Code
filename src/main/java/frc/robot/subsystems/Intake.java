package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Intake extends SubsystemBase {
    public enum Speed {
        FEED(6000),
        STOP(0);

        private final double rpm;

        private Speed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity angularVelocity() {
            return RPM.of(rpm);
        }
    }

    public enum Position {
        HOMED(50),
        STOWED(50),
        INTAKE(35),
        AGITATE(40);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    private static final double kNeoVortexFreeSpeed = 6784.0; // RPM
    private static final double kPivotReduction = 65.625;
    private static final AngularVelocity kMaxPivotSpeed = Constants.intakeMaxSpeed.div(kPivotReduction);
    private static final Angle kPositionTolerance = Degrees.of(5);
    private double targetPivotPosition = 0.0;

    private final SparkMax pivotMotor;
    private final SparkFlex rollerMotor;

    private boolean isHomed = false;

    public Intake() {
        pivotMotor = new SparkMax(Ports.kIntakePivot, MotorType.kBrushless);
        rollerMotor = new SparkFlex(Ports.kIntakeRollers, MotorType.kBrushless);
        configurePivotMotor();
        configureRollerMotor();

        // Read initial position from absolute encoder
        targetPivotPosition = pivotMotor.getAbsoluteEncoder().getPosition();

        SmartDashboard.putData(this);
    }

    private void configurePivotMotor() {
        final SparkMaxConfig config = new SparkMaxConfig();
        
        config.inverted(true)
            .idleMode(IdleMode.kBrake);
        
        config.smartCurrentLimit(10) //was 70
            .secondaryCurrentLimit(120);
        
        config.absoluteEncoder
            .positionConversionFactor(360.0) // degrees
            .velocityConversionFactor(60.0 / kPivotReduction)  // RPM
            .inverted(false); // Set based on your encoder mounting
        
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(0.013)
            .i(0.0)
            .d(0.0)
            //.velocityFF(12.0 / (kNeoVortexFreeSpeed / 60.0)) // kV: 12 volts at max speed (converted to RPS)
            .maxMotion
                .maxVelocity(4000)//kMaxPivotSpeed.in(RPM) * 0.01)  // Limit to 30% of max speed for smoother control
                .maxAcceleration(15000)//kMaxPivotSpeed.in(RPM) * 0.001);  // Limit acceleration for smoother control
                .allowedClosedLoopError(5);

        config.softLimit
            .forwardSoftLimit(120)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(0)
            .reverseSoftLimitEnabled(true);

        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureRollerMotor() {
        SparkFlexConfig config = new SparkFlexConfig();
        
        config.inverted(true);
        //config.idleMode(IdleMode.kCoast);  //Rev Client Manages This
        config.smartCurrentLimit(80); // Supply current limit
        config.secondaryCurrentLimit(120); // Stator current limit
        
        // PID configuration
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.0001, 0, 0) // kP, kI, kD for velocity control in RPM
            .velocityFF(12.0 / 6784) // kV: 12 volts when requesting max RPM
            .iZone(0);
        
        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private boolean isPositionWithinTolerance() {
        final double currentPosition = pivotMotor.getAbsoluteEncoder().getPosition();
        return Math.abs(currentPosition - targetPivotPosition) < kPositionTolerance.in(Degrees);
    }

    private void setPivotPercentOutput(double percentOutput) {
        pivotMotor.set(percentOutput);
    }

    public void set(Position position) {
        targetPivotPosition = position.angle().in(Degrees);
        pivotMotor.getClosedLoopController().setSetpoint(
            targetPivotPosition,
            SparkMax.ControlType.kMAXMotionPositionControl
        );
    }

    public void set(Speed speed) {
        rollerMotor.getClosedLoopController().setSetpoint(
            speed.angularVelocity().in(RPM),
            ControlType.kVelocity
        );
    }

    public Command intakeCommand() {
        return startEnd(
            () -> {
                set(Position.INTAKE);
                set(Speed.FEED);
            },
            () -> set(Speed.STOP)
        );
    }

    public Command agitateCommand() {
        return runOnce(() -> set(Speed.FEED))
            .andThen(
                Commands.sequence(
                    runOnce(() -> set(Position.AGITATE)),
                    Commands.waitUntil(this::isPositionWithinTolerance),
                    runOnce(() -> set(Position.INTAKE)),
                    Commands.waitUntil(this::isPositionWithinTolerance)
                )
                .repeatedly()
            )
            .finallyDo(() -> { // Changed from handleInterrupt
                set(Position.INTAKE);
                set(Speed.STOP);
            });
    }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(0.15)),
                Commands.waitUntil(() -> pivotMotor.getOutputCurrent() > 15)
                    .withTimeout(3.0), // ADD TIMEOUT
                runOnce(() -> {
                    setPivotPercentOutput(0); // STOP MOTOR
                    pivotMotor.getEncoder().setPosition(Position.HOMED.angle().in(Degrees));
                    isHomed = true;
                }),
                Commands.waitSeconds(0.1), // Let things settle
                runOnce(() -> set(Position.STOWED))
            )
            .unless(() -> isHomed)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Angle (degrees)", () -> pivotMotor.getAbsoluteEncoder().getPosition(), null);
        builder.addDoubleProperty("Target Angle (degrees)", () -> targetPivotPosition, null);
        builder.addBooleanProperty("At Target", this::isPositionWithinTolerance, null);
        // Remove: builder.addBooleanProperty("Is Homed", () -> isHomed, null);
        builder.addDoubleProperty("Roller RPM", () -> rollerMotor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("Pivot Current (A)", () -> pivotMotor.getOutputCurrent(), null);
        builder.addDoubleProperty("Roller Current (A)", () -> rollerMotor.getOutputCurrent(), null);
    }
}
