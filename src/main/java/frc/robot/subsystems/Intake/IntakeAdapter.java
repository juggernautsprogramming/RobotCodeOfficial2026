package frc.robot.subsystems.Intake;

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
 * TUNE: Set DEPLOYED_ROTATIONS to the motor rotations that fully extend
 * your intake arm. Set STOWED_ROTATIONS to fully retracted.
 */
public class IntakeAdapter {

    // ── Tune these to your mechanism ──────────────────────────────────────────
    private static final double DEPLOYED_ROTATIONS = 11.33; // actuation arm out
    private static final double STOWED_ROTATIONS   =  0.0;  // actuation arm in
    private static final double ROLLER_POWER       =  0.8;  // uptake roller speed
    private static final double BAR_POWER          =  0.8;  // intake bar/rod speed

    private final ActuationSubsystem     m_actuation;
    private final UptakeSubsystem        m_uptake;
    private final IntakeRollerSubsystem  m_rollers;

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
        m_actuation.setPosition(DEPLOYED_ROTATIONS);
        m_uptake.setPower(ROLLER_POWER);
        m_rollers.setPower(BAR_POWER);
    }

    /** Stop all motors + retract arm — call when done collecting. */
    public void stop() {
        m_uptake.stopMotors();
        m_rollers.stop();
        m_actuation.setPosition(STOWED_ROTATIONS);
    }

    /** Reverse both rollers and bar to eject. */
    public void eject() {
        m_uptake.setPower(-ROLLER_POWER);
        m_rollers.setPower(-BAR_POWER);
    }

    /**
     * Ball detection stub — returns false until you wire a beam-break sensor.
     * Replace with:  return !m_beamBreak.get();
     */
    public boolean hasBall() {
        return false;
    }
}