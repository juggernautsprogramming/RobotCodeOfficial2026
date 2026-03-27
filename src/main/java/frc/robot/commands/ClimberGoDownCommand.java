package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * ClimberGoDownCommand — moves the climber down to the DOWN position.
 *
 * Runs at fixed power until the climber reaches the target position,
 * then holds with brake.
 */
public class ClimberGoDownCommand extends Command {

    private final ClimberSubsystem m_climber;
    private static final double TARGET_POSITION = ClimberConstants.DOWN_POSITION_ROTATIONS;
    private static final double DOWN_POWER = -0.5;  // Power to move down
    private static final double TOLERANCE = ClimberConstants.POSITION_TOLERANCE_ROTATIONS;

    public ClimberGoDownCommand(ClimberSubsystem climber) {
        m_climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("ClimberDown/Status", "MOVING DOWN");
    }

    @Override
    public void execute() {
        m_climber.setPowerLevel(DOWN_POWER);
        double currentPos = m_climber.getCurrentPosition();
        SmartDashboard.putNumber("ClimberDown/Current_rot", currentPos);
        SmartDashboard.putNumber("ClimberDown/Target_rot", TARGET_POSITION);
        SmartDashboard.putNumber("ClimberDown/Error_rot", Math.abs(currentPos - TARGET_POSITION));
    }

    @Override
    public boolean isFinished() {
        double currentPos = m_climber.getCurrentPosition();
        return Math.abs(currentPos - TARGET_POSITION) < TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
        SmartDashboard.putString("ClimberDown/Status", interrupted ? "INTERRUPTED" : "AT DOWN");
    }
}
