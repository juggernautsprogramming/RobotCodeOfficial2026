package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * AutoClimbCommand — moves the climber up at full power during auto.
 *
 * <p>Runs the climber at fixed power level (0.5) until timeout or interrupted.
 *
 * <h3>PathPlanner usage</h3>
 * Registered as {@code "AutoClimbUp"} in RobotContainer named commands.
 * Add it at the end of your auto path after the last shot sequence.
 */
public class AutoClimbCommand extends Command {

    private final ClimberSubsystem m_climber;

    public AutoClimbCommand(ClimberSubsystem climber) {
        m_climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        m_climber.setPowerLevel(-0.75);
        SmartDashboard.putString("AutoClimb/Status", "CLIMBING");
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("AutoClimb/Position_rot", m_climber.getCurrentPosition());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_climber.getCurrentPosition() - ClimberConstants.DOWN_POSITION_ROTATIONS)
            < ClimberConstants.POSITION_TOLERANCE_ROTATIONS;
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
        SmartDashboard.putString("AutoClimb/Status", interrupted ? "INTERRUPTED" : "DONE");
    }
}
