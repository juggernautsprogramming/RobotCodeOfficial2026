package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Robot — top-level robot class.
 *
 * <h3>Deprecation fix</h3>
 * {@code Command.schedule()} is deprecated in WPILib 2026.
 * Use {@code CommandScheduler.getInstance().schedule(command)} instead.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData("Drive/Robot Heading", m_robotContainer.drivetrain.getPigeon2());
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            // FIX: Command.schedule() is deprecated — use CommandScheduler directly
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override public void disabledInit()       {}
    @Override public void disabledPeriodic()   {}
    @Override public void disabledExit()       {}
    @Override public void autonomousPeriodic() {}
    @Override public void autonomousExit()     {}
    @Override public void teleopPeriodic()     {}
    @Override public void teleopExit()         {}
    @Override public void testPeriodic()       {}
    @Override public void testExit()           {}
    @Override public void simulationPeriodic() {}
}