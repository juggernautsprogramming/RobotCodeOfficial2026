package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/**
 * ShootNBallsCommand — fires a fixed number of balls through the shooter.
 *
 * <h3>Sequence (per ball)</h3>
 * <ol>
 *   <li>Call {@link ShooterSubsystem#shoot()} immediately — flywheel is pre-spun by caller.</li>
 *   <li>Wait {@link #CYCLE_TIME_S} for the feeder to advance the next ball.</li>
 * </ol>
 *
 * <p><b>Note:</b> The RPM setpoint must already be set before this command starts.
 * {@code DriveToHubAndShoot} or a parallel {@code InstantCommand} should call
 * {@code setFlywheelRPM()} before scheduling this command.
 */
public class ShootNBallsCommand extends Command {

    /** Time between shots for the feeder to cycle the next ball. */
    private static final double CYCLE_TIME_S = 0.4;

    private enum Phase { SHOOTING, CYCLING, DONE }

    private final ShooterSubsystem m_shooter;
    private final int              m_totalBalls;
    private final double           m_targetRPM;

    private Phase  m_phase;
    private int    m_ballsFired;
    private double m_phaseStartS;

    /**
     * @param shooter    Shooter subsystem.
     * @param balls      Number of balls to fire.
     * @param targetRPM  RPM setpoint (passed to {@code isAtTargetRPM}).
     */
    public ShootNBallsCommand(ShooterSubsystem shooter, int balls, double targetRPM) {
        m_shooter    = shooter;
        m_totalBalls = balls;
        m_targetRPM  = targetRPM;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        m_phase       = Phase.SHOOTING;
        m_ballsFired  = 0;
        m_phaseStartS = Timer.getFPGATimestamp();
        m_shooter.setFlywheelRPM(m_targetRPM);
    }

    @Override
    public void execute() {
        double now     = Timer.getFPGATimestamp();
        double elapsed = now - m_phaseStartS;

        switch (m_phase) {

            case SHOOTING -> {
                m_shooter.shoot();
                m_ballsFired++;
                m_phase       = (m_ballsFired >= m_totalBalls) ? Phase.DONE : Phase.CYCLING;
                m_phaseStartS = now;
            }

            case CYCLING -> {
                // Keep flywheel at speed while feeder advances the next ball
                m_shooter.setFlywheelRPM(m_targetRPM);
                if (elapsed >= CYCLE_TIME_S) {
                    m_phase       = Phase.SHOOTING;
                    m_phaseStartS = now;
                }
            }

            case DONE -> { /* isFinished() exits */ }
        }

        SmartDashboard.putString("AutoShoot/Phase",       m_phase.name());
        SmartDashboard.putNumber("AutoShoot/BallsFired",  m_ballsFired);
        SmartDashboard.putNumber("AutoShoot/TargetBalls", m_totalBalls);
        SmartDashboard.putNumber("AutoShoot/TargetRPM",   m_targetRPM);
    }

    @Override
    public boolean isFinished() {
        return m_phase == Phase.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.idleFlywheel();
        SmartDashboard.putString("AutoShoot/Phase",
            interrupted ? "INTERRUPTED" : "COMPLETE");
    }
}