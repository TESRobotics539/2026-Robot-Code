package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Intake extends SubsystemBase {
    public enum Speed {
        STOP(0),
        FEED(0.25);  // TODO: was initially 0.8

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    public enum Position {
        HOMED(110),
        STOWED(100),
        INTAKE(-4),
        AGITATE(20);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    private static final double kPivotReduction = 50.0;
    private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
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
        SmartDashboard.putData(this);
    }

    private void configurePivotMotor() {
        final SparkMaxConfig config = new SparkMaxConfig();
        
        config.inverted(false)
            .idleMode(IdleMode.kBrake);
        
        config.smartCurrentLimit(70)
            .secondaryCurrentLimit(120);
        
        config.encoder
            .positionConversionFactor(360.0 / kPivotReduction) // degrees
            .velocityConversionFactor(1.0 / kPivotReduction);  // RPM
        
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.1)
            .i(0.0)
            .d(0.0)
            .velocityFF(12.0 / kMaxPivotSpeed.in(RotationsPerSecond))
            .maxMotion
                .maxVelocity(kMaxPivotSpeed.in(RPM))
                .maxAcceleration(kMaxPivotSpeed.in(RPM) * 0.25);
        
        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureRollerMotor() {
        final SparkFlexConfig config = new SparkFlexConfig();
        
        config.inverted(true)
            .idleMode(IdleMode.kBrake);
        
        config.smartCurrentLimit(70)
            .secondaryCurrentLimit(120);
        
        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private boolean isPositionWithinTolerance() {
        final double currentPosition = pivotMotor.getEncoder().getPosition();
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
        rollerMotor.setVoltage(speed.voltage().in(Volts));
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
        builder.addDoubleProperty("Angle (degrees)", () -> pivotMotor.getEncoder().getPosition(), null);
        builder.addDoubleProperty("Target Angle (degrees)", () -> targetPivotPosition, null);
        builder.addBooleanProperty("At Target", this::isPositionWithinTolerance, null);
        builder.addBooleanProperty("Is Homed", () -> isHomed, null);
        builder.addDoubleProperty("Roller RPM", () -> rollerMotor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("Pivot Current (A)", () -> pivotMotor.getOutputCurrent(), null);
        builder.addDoubleProperty("Roller Current (A)", () -> rollerMotor.getOutputCurrent(), null);
    }
}
