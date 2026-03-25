package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * AutoClimbCommand — drives the climber to the Level 1 rung during auto.
 *
 * <p>Calls MotionMagic to smoothly reach {@link ClimberConstants#LEVEL1_CLIMB_DEGREES}.
 * Ends when the climber is within {@link ClimberConstants#CLIMB_POSITION_TOLERANCE_DEG}
 * of the target, or when the command is interrupted (e.g. auto timeout).
 *
 * <h3>Tuning</h3>
 * Manually jog the climber up to touch the Level 1 rung, read the position from
 * SmartDashboard → {@code Climber/Position_rot}, convert to degrees
 * ({@code pos_rot × 360}), and update {@link ClimberConstants#LEVEL1_CLIMB_DEGREES}.
 *
 * <h3>PathPlanner usage</h3>
 * Registered as {@code "AutoClimbLevel1"} in RobotContainer named commands.
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
        m_climber.setPositionDegrees(ClimberConstants.LEVEL1_CLIMB_DEGREES);
        SmartDashboard.putString("AutoClimb/Status", "CLIMBING");
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("AutoClimb/AtTarget",
            m_climber.isAtPosition(ClimberConstants.LEVEL1_CLIMB_DEGREES));
    }

    @Override
    public boolean isFinished() {
        return m_climber.isAtPosition(ClimberConstants.LEVEL1_CLIMB_DEGREES);
    }

    @Override
    public void end(boolean interrupted) {
        // Hold position via MotionMagic — don't stop the motor
        SmartDashboard.putString("AutoClimb/Status", interrupted ? "INTERRUPTED" : "DONE");
    }
}
