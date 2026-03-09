package frc.robot.subsystems;


import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.config.ServoHubParameter;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHubSim;


import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Ports;

import frc.robot.Constants;


/**
public class ServoHub {


    private final m_servoHub;

    m_servoHub = new ServoHub(kServoHub);




    /*
    // Initialize the servo hub
    ServoHub m_servoHub = new ServoHub(kServoHub);

    // Obtain a servo channel controller
    ServoChannel m_channel0 = m_servoHub.getServoChannel(ChannelId.kChannelId0);
    ServoChannel m_channel1 = m_servoHub.getServoChannel(ChannelId.kChannelId1);
    ...
    ServoChannel m_channel5 = m_servoHub.getServoChannel(ChannelId.kChannelId5);
    */
//}

