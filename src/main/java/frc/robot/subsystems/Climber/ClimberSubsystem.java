package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * ClimberSubsystem — single motor climber with voltage-controlled duty cycle.
 *
 * Uses VoltageOut control requests for consistent torque regardless of battery
 * voltage. Hold triggers to move manually; brake mode holds position on release.
 */
public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX   motor;
    private final VoltageOut m_voltageRequest = new VoltageOut(0);

    public ClimberSubsystem() {
        CANBus chassisCAN = new CANBus(ClimberConstants.CAN_BUS);
        motor = new TalonFX(ClimberConstants.MOTOR_ID, chassisCAN);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.StatorCurrentLimit       = ClimberConstants.STATOR_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Hardware-enforced soft limits — stops motion before mechanism damage
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.UP_POSITION_ROTATIONS;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.DOWN_POSITION_ROTATIONS;

        motor.getConfigurator().apply(config);
        motor.setPosition(0);
    }

    // ── Soft limit management (for match-start zeroing) ───────────────────────

    /**
     * Disables soft limits and zeros the encoder to the current physical position.
     * Call ONLY when the climber is physically at its lowest resting position.
     */
    public void zeroAndDisableSoftLimits() {
        var cfg = new TalonFXConfiguration();
        motor.getConfigurator().refresh(cfg);

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        motor.getConfigurator().apply(cfg);
        motor.setPosition(0.0);

        SmartDashboard.putString("Climber/LimitStatus", "DISABLED — ZEROED");
    }

    /**
     * Re-enables soft limits at their configured constants.
     * Call after zeroing is confirmed (i.e. on button release).
     */
    public void restoreSoftLimits() {
        var cfg = new TalonFXConfiguration();
        motor.getConfigurator().refresh(cfg);

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.UP_POSITION_ROTATIONS;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.DOWN_POSITION_ROTATIONS;

        motor.getConfigurator().apply(cfg);
        SmartDashboard.putString("Climber/LimitStatus", "RESTORED");
    }

    // ── Motion control ────────────────────────────────────────────────────────

    /**
     * Set motor power using voltage control.
     * Range: -1.0 (down) to 1.0 (up) — scaled to ±12V internally.
     * VoltageOut compensates for battery sag, giving consistent climb force
     * regardless of battery state. Soft limits still enforce travel bounds.
     *
     * @param power duty-cycle fraction, -1.0 to 1.0
     */
    public void setPowerLevel(double power) {
        motor.setControl(m_voltageRequest.withOutput(power * 12.0));
    }

    /**
     * Stop the motor. Brake mode holds position passively.
     */
    public void stop() {
        motor.setControl(m_voltageRequest.withOutput(0.0));
    }

    /**
     * Get current motor position in rotations.
     */
    public double getCurrentPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    // ── Telemetry ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        double posRot = getCurrentPosition();
        SmartDashboard.putNumber("Climber/Position_rot",  posRot);
        SmartDashboard.putNumber("Climber/Position_deg",  posRot * 360.0);
        SmartDashboard.putNumber("Climber/Velocity_rps",  motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Climber/Current_amps",  motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Climber/Voltage_V",     motor.getMotorVoltage().getValueAsDouble());
    }
}