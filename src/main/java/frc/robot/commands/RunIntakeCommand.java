package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.ActuationSubsystem;
import frc.robot.subsystems.Intake.IntakeAdapter;
import frc.robot.subsystems.Intake.UptakeSubsystem;

/**
 * RunIntakeCommand — runs the intake via IntakeAdapter until cancelled.
 *
 * Always use with a timeout in autonomous:
 * <pre>
 *   new RunIntakeCommand(adapter, actuation, uptake).withTimeout(3.0)
 * </pre>
 */
public class RunIntakeCommand extends Command {

    private final IntakeAdapter m_adapter;

    /**
     * @param adapter   IntakeAdapter coordinating arm + roller.
     * @param actuation Declared as requirement so the scheduler prevents conflicts.
     * @param uptake    Declared as requirement so the scheduler prevents conflicts.
     */
    public RunIntakeCommand(IntakeAdapter adapter,
                            ActuationSubsystem actuation,
                            UptakeSubsystem    uptake) {
        m_adapter = adapter;
        // Register both physical subsystems as requirements.
        // They are not stored as fields because IntakeAdapter already holds them.
        addRequirements(actuation, uptake);
    }

    @Override
    public void initialize() {
        m_adapter.run();
    }

    @Override
    public void execute() {
        // Nothing per-tick — IntakeAdapter handles the hardware.
        // Add ball-count logic here once beam-break sensor is wired.
    }

    @Override
    public boolean isFinished() {
        return false; // always rely on .withTimeout() to end this command
    }

    @Override
    public void end(boolean interrupted) {
        m_adapter.stop();
    }
}