package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * ClimberSubsystem — single motor climber with pure duty cycle control.
 *
 * Uses simple duty cycle commands (hold down triggers to move manually).
 */
public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX motor;

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

    /**
     * Set motor power directly using duty cycle.
     * Range: -1.0 (down) to 1.0 (up)
     * Hardware soft limits enforce UP_POSITION_ROTATIONS and DOWN_POSITION_ROTATIONS.
     */
    public void setPowerLevel(double power) {
        motor.set(power);
    }

    /**
     * Stop the motor.
     */
    public void stop() {
        motor.set(0);
    }

    /**
     * Get current motor position in rotations.
     */
    public double getCurrentPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        double posRot = getCurrentPosition();
        SmartDashboard.putNumber("Climber/Position_rot", posRot);
        SmartDashboard.putNumber("Climber/Position_deg", posRot * 360.0);
        SmartDashboard.putNumber("Climber/Velocity_rps", motor.getVelocity().getValueAsDouble());
    }
}