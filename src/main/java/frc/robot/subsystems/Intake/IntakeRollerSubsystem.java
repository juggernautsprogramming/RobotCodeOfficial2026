package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * IntakeRollerSubsystem — drives the two intake bar/rod motors (IDs 27 and 28).
 *
 * Motor 27 is the leader; motor 28 follows in the opposite direction
 * (they spin toward each other to pull game pieces in).
 * Flip kFollowerInverted if your mechanism spins the wrong way.
 */
public class IntakeRollerSubsystem extends SubsystemBase {

    // ── Hardware IDs ──────────────────────────────────────────────────────────
    private static final int    kLeaderID        = 27;
    private static final int    kFollowerID      = 28;
    private static final String kCANBus          = "ChassisCAN"; // change if on RIO bus

    // ── Inversion — flip kFollowerInverted if rollers fight each other ────────
    private static final boolean kLeaderInverted   = false;
    private static final boolean kFollowerInverted = true;  // opposite of leader

    // ── Current limits ────────────────────────────────────────────────────────
    private static final double kStatorLimit = 40.0; // Amps
    private static final double kSupplyLimit = 30.0; // Amps

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final TalonFX m_leader   = new TalonFX(kLeaderID,   kCANBus);
    private final TalonFX m_follower = new TalonFX(kFollowerID, kCANBus);

    // ── Control request (reused to avoid allocation) ──────────────────────────
    private final DutyCycleOut m_request = new DutyCycleOut(0.0);

    public IntakeRollerSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kStatorLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(kSupplyLimit)
            .withSupplyCurrentLimitEnable(true);

        // Apply to leader
        cfg.MotorOutput.Inverted =
            kLeaderInverted
                ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        m_leader.getConfigurator().apply(cfg);

        // Apply to follower (inverted relative to leader)
        cfg.MotorOutput.Inverted =
            kFollowerInverted
                ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        m_follower.getConfigurator().apply(cfg);
    }

    /**
     * Spin both rollers at the given duty cycle.
     * @param power -1.0 to 1.0 (positive = intake direction)
     */
    public void setPower(double power) {
        m_leader  .setControl(m_request.withOutput(power));
        m_follower.setControl(m_request.withOutput(power));
    }

    /** Stop both rollers (neutral output). */
    public void stop() {
        m_leader  .setControl(m_request.withOutput(0.0));
        m_follower.setControl(m_request.withOutput(0.0));
    }
}