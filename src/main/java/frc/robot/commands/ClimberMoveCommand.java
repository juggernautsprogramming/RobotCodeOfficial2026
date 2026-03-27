package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * ClimberPowerCommand — runs the climber at a specified power level.
 *
 * Can be used for manual control or timed climbing sequences.
 */
public class ClimberMoveCommand extends Command {

    private final ClimberSubsystem m_climber;
    private final DoubleSupplier m_powerSupplier;

    public ClimberMoveCommand(ClimberSubsystem climber, DoubleSupplier powerSupplier) {
        m_climber = climber;
        m_powerSupplier = powerSupplier;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("ClimberMove/Status", "RUNNING");
    }

    @Override
    public void execute() {
        double power = m_powerSupplier.getAsDouble();
        m_climber.setPowerLevel(power);
        SmartDashboard.putNumber("ClimberMove/Power", power);
        SmartDashboard.putNumber("ClimberMove/Position_rot", m_climber.getCurrentPosition());
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
        SmartDashboard.putString("ClimberMove/Status", "STOPPED");
    }
}
