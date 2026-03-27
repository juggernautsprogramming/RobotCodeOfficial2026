package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * ClimberManualTuneCommand — manually jog the climber with a joystick axis.
 *
 * Use this during tuning to find your UP and DOWN positions.
 * Read the position from SmartDashboard → Climber/Position_rot.
 */
public class ClimberManualTuneCommand extends Command {

    private final ClimberSubsystem m_climber;
    private final DoubleSupplier m_powerSupplier;

    /**
     * @param climber the climber subsystem
     * @param powerSupplier joystick axis supplier (typically -1.0 to 1.0)
     */
    public ClimberManualTuneCommand(ClimberSubsystem climber, DoubleSupplier powerSupplier) {
        m_climber = climber;
        m_powerSupplier = powerSupplier;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("ClimberTune/Status", "MANUAL CONTROL ACTIVE");
    }

    @Override
    public void execute() {
        double power = m_powerSupplier.getAsDouble();
        m_climber.setPowerLevel(power);
        
        // Update dashboard with current position for easy reading
        SmartDashboard.putNumber("ClimberTune/JoystickPower", power);
        SmartDashboard.putNumber("ClimberTune/CurrentPosition_rot", m_climber.getCurrentPosition());
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
        SmartDashboard.putString("ClimberTune/Status", "STOPPED");
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
