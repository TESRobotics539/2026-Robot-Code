package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Motor IDs
    public static final int kIntakePivot = 40;
    public static final int kIntakeRollers = 41;
    
    public static final int kFloor = 54;
    public static final int kFeeder = 61;

    public static final int kShooterLeft = 57;
    public static final int kShooterMiddle = 56;
    public static final int kShooterRight = 55;
    
    public static final int kHanger = 18;

    // PWM Ports
    public static final int kHoodLeftServo = 3;
    public static final int kHoodRightServo = 4;
}
