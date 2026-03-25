package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * IntakeRollerSubsystem — drives the two intake bar/rod motors.
 *
 * Motor ROLLER_LEADER_ID is the leader; ROLLER_FOLLOWER_ID follows in the
 * opposite direction (they spin toward each other to pull game pieces in).
 * Flip kFollowerInverted if your mechanism spins the wrong way.
 */
public class IntakeRollerSubsystem extends SubsystemBase {

    // ── Inversion — flip kFollowerInverted if rollers fight each other ────────
    private static final boolean kLeaderInverted   = false;
    private static final boolean kFollowerInverted = true;  // opposite of leader

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final CANBus   m_bus      = new CANBus(IntakeConstants.ROLLER_CAN_BUS);
    private final TalonFX  m_leader   = new TalonFX(IntakeConstants.ROLLER_LEADER_ID,   m_bus);
    private final TalonFX  m_follower = new TalonFX(IntakeConstants.ROLLER_FOLLOWER_ID, m_bus);

    // ── Control request (reused to avoid allocation) ──────────────────────────
    private final DutyCycleOut m_request = new DutyCycleOut(0.0);

    public IntakeRollerSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(IntakeConstants.ROLLER_STATOR_LIMIT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(IntakeConstants.ROLLER_SUPPLY_LIMIT)
            .withSupplyCurrentLimitEnable(true);

        // Apply to leader
        cfg.MotorOutput.Inverted = kLeaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        m_leader.getConfigurator().apply(cfg);

        // Apply to follower (inverted relative to leader)
        cfg.MotorOutput.Inverted = kFollowerInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
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
