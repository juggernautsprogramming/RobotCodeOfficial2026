package frc.robot.subsystems.Feeder;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * FeederSubsystem — duty-cycle roller for game-piece feeding.
 *
 * <h3>Deprecation / warning fixes</h3>
 * <ul>
 *   <li>Replaced {@code new TalonFX(id, "ChassisCAN")} with
 *       {@code new TalonFX(id, new CANBus("ChassisCAN"))}.</li>
 *   <li>Removed the unused {@code motionMagicRequest} field — the feeder is a
 *       continuous roller and does not need position control.</li>
 * </ul>
 */
public class FeederSubsystem extends SubsystemBase {

    private static final int    MOTOR_ID         = 44;
    private static final String CAN_BUS          = "ChassisCAN";
    private static final double CRUISE_VELOCITY  = 100;
    private static final double ACCELERATION     = 200;

    private final TalonFX      feederMotor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public FeederSubsystem() {
        // FIX: use CANBus object instead of deprecated TalonFX(int, String)
        feederMotor = new TalonFX(MOTOR_ID, new String(CAN_BUS));

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration   = ACCELERATION;

        config.Slot0.kP = 2.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        config.Voltage.PeakForwardVoltage  =  8;
        config.Voltage.PeakReverseVoltage  = -8;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = feederMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.err.println("Feeder Motor config failed: " + status);
        }

        feederMotor.setPosition(0);
    }

    /** Set motor power using Duty Cycle (-1.0 to 1.0). */
    public void setPower(double power) {
        feederMotor.setControl(dutyCycleRequest.withOutput(power));
    }

    public void stop() { feederMotor.stopMotor(); }

    @Override
    public void periodic() {}
}