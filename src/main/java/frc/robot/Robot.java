// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.OptionalInt;

import com.ctre.phoenix6.SignalLogger;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.LogFileUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.util.LoggedTracer;
import frc.util.MetricTracker;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
    private final RobotContainer m_robotContainer;
    private Command m_autonomousCommand;
    
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        // Configure AdvantageKit logger before anything else.
        Logger.recordMetadata("ProjectName", "2026-Robot-Code");
        Logger.recordMetadata("RobotName",   "Helga");
        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // logs to /U/logs on USB stick
            Logger.addDataReceiver(new NT4Publisher());
        } else if (System.getenv("AKIT_REPLAY") != null) {
            // Log replay mode: run deterministically against a recorded .wpilog file.
            // Set AKIT_REPLAY=1 (or any value) in your run config, then pick the file when prompted.
            setUseTiming(false); // replay as fast as possible, not real-time
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }
        // Suppress CTRE's built-in signal logger — prevents double-logging with AdvantageKit.
        SignalLogger.enableAutoLogging(false);
        Logger.start();

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        // Do NOT call SmartDashboard.putData(CommandScheduler.getInstance()) — registering the
        // scheduler for SmartDashboard NT updates causes multi-millisecond spikes in
        // SmartDashboard.updateValues() every cycle. AdvantageKit handles all telemetry.
        // Lower brownout threshold so the robot keeps running longer on a depleted battery.
        // Default (~6.3V) causes spurious shutdowns under hard-stall loads; 4.6V is the
        // 1678 standard that avoids false brownouts while still protecting the RIO.
        RobotController.setBrownoutVoltage(Volts.of(4.6));
    }
    
    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        LoggedTracer.reset();
        double loopStart = Timer.getFPGATimestamp();
        CommandScheduler.getInstance().run();
        Logger.recordOutput("LoopTime/ms",              (Timer.getFPGATimestamp() - loopStart) * 1000.0);
        Logger.recordOutput("GameData/HubActive",        GameData.isHubActive());
        Logger.recordOutput("GameData/HubActiveExpanded", GameData.isHubActiveExpanded(5.0));
        // Health metrics read by health_watchdog.py and match_logger.py on the Pi.
        Logger.recordOutput("Robot/BatteryVoltage_V", RobotController.getBatteryVoltage());
        Logger.recordOutput("Robot/MatchTime_s",      DriverStation.getMatchTime());
        Logger.recordOutput("Robot/Enabled",          DriverStation.isEnabled());
    }

    @Override
    public void disabledInit() {
        MetricTracker.getInstance().flush();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousExit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopInit()
    {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        
        // Set the robot's starting position for teleop
        // Choose the starting location that matches where you placed the robot

        OptionalInt dsLocation = DriverStation.getLocation();
    
        Landmarks.StartingLocation location;
        if (dsLocation.isPresent()) {
            location = switch (dsLocation.getAsInt()) {
            case 1 -> Landmarks.StartingLocation.LEFT;
            case 2 -> Landmarks.StartingLocation.CENTER;
            case 3 -> Landmarks.StartingLocation.RIGHT;
            default -> Landmarks.StartingLocation.CENTER;
            };
        } else {
            location = Landmarks.StartingLocation.CENTER; // Default if not connected
        }

        m_robotContainer.zeroGyroWithAlliance();
        m_robotContainer.resetOdometry(
            Landmarks.getStartingPosition(location)
        );
    }
}
