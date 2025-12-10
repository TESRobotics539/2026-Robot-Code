package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Mk4iModule {
    // SDS Mk4i L3 Constants
    private static final double DRIVE_GEAR_RATIO = 6.12; //Changed from 6.75 which is the GR for L3's
    private static final double STEER_GEAR_RATIO = 150.0 / 7.0;
    private static final double WHEEL_DIAMETER_METERS = 0.1016;
    
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;
    private final CANcoder steerEncoder;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerRelativeEncoder;
    
    private final PIDController steerPID;
    private final SimpleMotorFeedforward driveFeedforward;
    private final double chassisAngularOffset;
    
    public Mk4iModule(int driveID, int steerID, int encoderID, double chassisAngularOffset) {
        this.chassisAngularOffset = chassisAngularOffset;
        
        // Drive motor (Neo Vortex)
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.smartCurrentLimit(60);
        
        double drivePositionConversion = (WHEEL_DIAMETER_METERS * Math.PI) / DRIVE_GEAR_RATIO;
        double driveVelocityConversion = drivePositionConversion / 60.0;
        driveConfig.encoder.positionConversionFactor(drivePositionConversion);
        driveConfig.encoder.velocityConversionFactor(driveVelocityConversion);
        
        driveMotor.configure(driveConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        
        // Steer motor (Neo)
        steerMotor = new SparkMax(steerID, MotorType.kBrushless);
        steerRelativeEncoder = steerMotor.getEncoder();
        
        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig.idleMode(IdleMode.kBrake);
        steerConfig.smartCurrentLimit(30);
        
        double steerPositionConversion = 2 * Math.PI / STEER_GEAR_RATIO;
        steerConfig.encoder.positionConversionFactor(steerPositionConversion);
        
        steerMotor.configure(steerConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        
        // CANcoder for absolute position
        steerEncoder = new CANcoder(encoderID);
        
        // PID for steering
        steerPID = new PIDController(0.375, 0.0, 0.0);
        steerPID.enableContinuousInput(-Math.PI, Math.PI);
        
        // Feedforward for driving
        driveFeedforward = new SimpleMotorFeedforward(0.1, 2.5);
        
        resetEncoders();
    }
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            new Rotation2d(steerEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI - chassisAngularOffset)
        );
    }
    
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            new Rotation2d(steerEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI - chassisAngularOffset)
        );
    }
    
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getState().angle);
        
        // Calculate drive output
        double driveOutput = driveFeedforward.calculate(state.speedMetersPerSecond);
        driveMotor.setVoltage(driveOutput);
        
        // Calculate steer output
        double steerOutput = steerPID.calculate(getState().angle.getRadians(), state.angle.getRadians());
        steerMotor.setVoltage(steerOutput);
    }
    
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        double absolutePosition = steerEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI - chassisAngularOffset;
        steerRelativeEncoder.setPosition(absolutePosition);
    }
}