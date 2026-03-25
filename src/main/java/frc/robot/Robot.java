package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Robot — top-level robot class using AdvantageKit.
 *
 * Modes:
 *   - Real robot:  logs to USB/onboard + NT4
 *   - Simulation:  logs to NT4 only (no replay source)
 *   - Replay:      reads existing log, writes _sim-suffixed output
 */
public class Robot extends LoggedRobot {  // ✅ Must extend LoggedRobot, not TimedRobot

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        // ── 1. Metadata ──────────────────────────────────────────────────────
        Logger.recordMetadata("ProjectName", "MyRobotProject");
        Logger.recordMetadata("RuntimeType", getRuntimeType().toString());       // optional

        // ── 2. Configure data receivers by mode ──────────────────────────────
        if (isReal()) {
            // On real hardware: write to onboard storage + publish to NT4
            Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
            Logger.addDataReceiver(new NT4Publisher());

        } else if (isSimulation()) {
            // Pure simulation (no replay): publish to NT4 only for Glass/Advantage Scope
            Logger.addDataReceiver(new NT4Publisher());

        } else {
            // Replay mode: read an existing log, write deterministic output
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(
                new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))
            );
        }

        // ── 3. Log Power Distribution (PDP/PDH) ──────────────────────────────
        // Remove this block if you don't have a PDP/PDH on your robot
        new PowerDistribution(1, ModuleType.kRev);  // Instantiating logs it automatically via AdvantageKit

        // ── 4. Start the logger — must happen before any subsystems are created
        Logger.start();

        // ── 5. Build subsystems / commands ───────────────────────────────────
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData("Drive/Robot Heading",
            m_robotContainer.drivetrain.getPigeon2());
    }

    // ── Periodic (runs every loop) ────────────────────────────────────────────

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }

    // ── Mode inits ────────────────────────────────────────────────────────────

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
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

    // ── Stubs (keep for completeness) ─────────────────────────────────────────

    @Override public void disabledInit()       {}
    @Override public void disabledPeriodic()   {}
    @Override public void disabledExit()       {}
    @Override public void autonomousPeriodic() {}
    @Override public void autonomousExit()     {}
    @Override
    public void teleopPeriodic() {
        frc.robot.util.ShiftStateTracker.publishTelemetry();
    }
    @Override public void teleopExit()         {}
    @Override public void testPeriodic()       {}
    @Override public void testExit()           {}
    @Override public void simulationPeriodic() {}
}