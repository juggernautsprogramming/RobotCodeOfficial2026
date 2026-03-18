package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * IntakeAdapter — thin facade that coordinates ActuationSubsystem (arm deploy),
 * UptakeSubsystem (uptake roller, ID 42), and IntakeRollerSubsystem (bar
 * motors, IDs 27 & 28) so commands can call run/stop/eject simply.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Actuation motor  (ID 43): deploys/retracts the intake arm
 * Uptake motor     (ID 42): spins the uptake roller
 * Intake rollers   (IDs 27 & 28): spins the intake bar/rod
 * ──────────────────────────────────────────────────────────────────────────
 *
 * When stop() is called, the arm retracts and the uptake stops immediately,
 * but the bar keeps spinning until the arm reaches the stow position so any
 * ball caught during retraction is still fed in.
 */
public class IntakeAdapter extends SubsystemBase {

    private final ActuationSubsystem     m_actuation;
    private final UptakeSubsystem        m_uptake;
    private final IntakeRollerSubsystem  m_rollers;

    /** True while the arm is retracting and bar should still spin. */
    private boolean m_retractingToStow = false;

    public IntakeAdapter(
            ActuationSubsystem actuation,
            UptakeSubsystem uptake,
            IntakeRollerSubsystem rollers) {
        m_actuation = actuation;
        m_uptake    = uptake;
        m_rollers   = rollers;
    }

    /** Deploy arm + spin uptake roller + spin intake bar — call at start of intake phase. */
    public void run() {
        m_retractingToStow = false;
        m_actuation.setPosition(IntakeConstants.DEPLOYED_ROTATIONS);
        m_uptake.setPower(IntakeConstants.ROLLER_POWER);
        m_rollers.setPower(IntakeConstants.BAR_POWER);
    }

    /**
     * Begin stow sequence — retracts arm and stops uptake immediately.
     * The bar keeps spinning until the arm reaches stow position (handled by periodic).
     */
    public void stop() {
        m_retractingToStow = true;
        m_uptake.stopMotors();
        // bar intentionally left running — periodic() stops it once arm is home
        m_actuation.setPosition(IntakeConstants.STOWED_ROTATIONS);
    }

    /** Reverse both rollers and bar to eject. */
    public void eject() {
        m_retractingToStow = false;
        m_uptake.setPower(-IntakeConstants.ROLLER_POWER);
        m_rollers.setPower(-IntakeConstants.BAR_POWER);
    }

    /**
     * Stops the bar once the arm reaches the stow position after stop() is called.
     * Runs every 20 ms by the command scheduler.
     */
    @Override
    public void periodic() {
        if (m_retractingToStow
                && m_actuation.getCurrentPosition()
                    <= IntakeConstants.STOWED_ROTATIONS + IntakeConstants.STOW_TOLERANCE) {
            m_rollers.stop();
            m_retractingToStow = false;
        }
    }

    /**
     * Ball detection stub — returns false until you wire a beam-break sensor.
     * Replace with:  return !m_beamBreak.get();
     */
    public boolean hasBall() {
        return false;
    }
}
