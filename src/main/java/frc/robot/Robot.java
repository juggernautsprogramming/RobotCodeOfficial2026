// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision.ReadAprilTag;
import frc.robot.subsystems.Vision.VisionSubsystem;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private edu.wpi.first.networktables.GenericEntry m_tagIdWidget;
    private edu.wpi.first.networktables.GenericEntry m_yawWidget;
    private edu.wpi.first.networktables.GenericEntry m_pitchWidget;
    private edu.wpi.first.networktables.GenericEntry m_areaWidget;
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

   public Robot() {
        m_robotContainer = new RobotContainer();
        var tab = Shuffleboard.getTab("Example Tab");
    
        tab.add("Robot Heading", m_robotContainer.drivetrain.getPigeon2());

        // 2. Initialize the entries in the constructor
        m_tagIdWidget = tab.add("Tag ID", -1).withPosition(0, 0).getEntry();
        m_yawWidget   = tab.add("Yaw", 0).withPosition(0, 1).getEntry();
        m_pitchWidget = tab.add("Pitch", 0).withPosition(1, 1).getEntry();
        m_areaWidget  = tab.add("Area", 0).withPosition(1, 0).getEntry();

        tab.addCamera("Camera Back Left", "Camera_BackLeft", "mjpg:http://photonvision.local:1181/?action=stream")
            .withProperties(Map.of("showControls", false))
            .withPosition(2, 0)
            .withSize(3, 3);
    }
    
    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 

        // 3. Update all widgets every loop
        var vision = m_robotContainer.visionSubsystem;

        if (vision.hasValidTarget()) {
            // This pulls the actual target object to get detailed info
            var target = vision.getBestTarget(); 
            
            m_tagIdWidget.setInteger(target.getFiducialId());
            m_yawWidget.setDouble(target.getYaw());
            m_pitchWidget.setDouble(target.getPitch());
            m_areaWidget.setDouble(target.getArea());
        } else {
            // Clear values when no target is seen
            m_tagIdWidget.setInteger(-1);
            m_yawWidget.setDouble(0);
            m_pitchWidget.setDouble(0);
            m_areaWidget.setDouble(0);
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
