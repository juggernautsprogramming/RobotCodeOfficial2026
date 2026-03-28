package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * ClimberGoUpCommand — moves the climber up to the UP position.
 *
 * Runs at fixed power until the climber reaches the target position,
 * then stops (brake mode holds the position).
 */
public class ClimberGoUpCommand extends Command {

    private final ClimberSubsystem m_climber;
    private static final double TARGET_POSITION = ClimberConstants.UP_POSITION_ROTATIONS;
    private static final double UP_POWER = 0.8;
    private static final double TOLERANCE = ClimberConstants.POSITION_TOLERANCE_ROTATIONS;

    public ClimberGoUpCommand(ClimberSubsystem climber) {
        m_climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("ClimberUp/Status", "MOVING UP");
    }

    @Override
    public void execute() {
        m_climber.setPowerLevel(UP_POWER);
        double currentPos = m_climber.getCurrentPosition();
        SmartDashboard.putNumber("ClimberUp/Current_rot", currentPos);
        SmartDashboard.putNumber("ClimberUp/Target_rot", TARGET_POSITION);
        SmartDashboard.putNumber("ClimberUp/Error_rot", Math.abs(currentPos - TARGET_POSITION));
    }

    @Override
    public boolean isFinished() {
        double currentPos = m_climber.getCurrentPosition();
        return Math.abs(currentPos - TARGET_POSITION) < TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
        SmartDashboard.putString("ClimberUp/Status", interrupted ? "INTERRUPTED" : "AT UP");
    }
}
