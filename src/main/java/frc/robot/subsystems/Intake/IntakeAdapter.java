package frc.robot.subsystems.Intake;

/**
 * IntakeAdapter — thin facade that coordinates ActuationSubsystem (arm deploy)
 * and UptakeSubsystem (roller) so autonomous commands can call run/stop simply.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Actuation motor  (ID 43): deploys/retracts the intake arm
 * Uptake motor     (ID 42): spins the intake roller
 * ──────────────────────────────────────────────────────────────────────────
 *
 * TUNE: Set DEPLOYED_ROTATIONS to the motor rotations that fully extend
 * your intake arm. Set STOWED_ROTATIONS to fully retracted.
 */
public class IntakeAdapter {

    // ── Tune these to your mechanism ──────────────────────────────────────────
    private static final double DEPLOYED_ROTATIONS = 11.33; // actuation arm out
    private static final double STOWED_ROTATIONS   =  0.0; // actuation arm in
    private static final double ROLLER_POWER       =  0.8; // uptake roller speed

    private final ActuationSubsystem m_actuation;
    private final UptakeSubsystem    m_uptake;

    public IntakeAdapter(ActuationSubsystem actuation, UptakeSubsystem uptake) {
        m_actuation = actuation;
        m_uptake    = uptake;
    }

    /** Deploy arm + spin roller — call at start of intake phase. */
    public void run() {
        m_actuation.setPosition(DEPLOYED_ROTATIONS);
        m_uptake.setPower(ROLLER_POWER);
    }

    /** Stop roller + retract arm — call when done collecting. */
    public void stop() {
        m_uptake.stopMotors();
        m_actuation.setPosition(STOWED_ROTATIONS);
    }

    /** Reverse roller to eject. */
    public void eject() {
        m_uptake.setPower(-ROLLER_POWER);
    }

    /**
     * Ball detection stub — returns false until you wire a beam-break sensor.
     * Replace with:  return !m_beamBreak.get();
     */
    public boolean hasBall() {
        return false;
    }
}