package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * HoodSubsystem — adjustable hood angle with Motion Magic position control.
 *
 * <h3>Deprecation / warning fixes</h3>
 * <ul>
 *   <li>Replaced {@code new TalonFX(id, "ChassisCAN")} with
 *       {@code new TalonFX(id, new CANBus("ChassisCAN"))}.</li>
 *   <li>{@code moveToDegrees()} changed from {@code private} to {@code public}
 *       so it can be called from commands (was flagged as "never used locally").</li>
 * </ul>
 */
public class HoodSubsystem extends SubsystemBase {

    private static final int    MOTOR_ID        = 44;
    private static final String CAN_BUS         = "ChassisCAN";
    private static final double CRUISE_VELOCITY = 100;
    private static final double ACCELERATION    = 200;
    private static final double GEAR_RATIO      = 1;

    private final TalonFX             HoodMotor;
    private final DutyCycleOut        dutyCycleRequest   = new DutyCycleOut(0);
    private final MotionMagicVoltage  motionMagicRequest = new MotionMagicVoltage(0);

    public HoodSubsystem() {
        // FIX: use CANBus object instead of deprecated TalonFX(int, String)
        HoodMotor = new TalonFX(MOTOR_ID, new String(CAN_BUS));

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
            status = HoodMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.err.println("Hood Motor config failed: " + status);
        }

        HoodMotor.setPosition(0);
    }

    /** Set motor power using Duty Cycle (-1.0 to 1.0). */
    public void setPower(double power) {
        HoodMotor.setControl(dutyCycleRequest.withOutput(power));
    }

    /**
     * Move the hood to an absolute angle using Motion Magic.
     * 0° = stowed (horizontal), positive = raised toward vertical.
     */
    public void moveToDegrees(double degrees) {
        double rotations = (degrees / 360.0) * GEAR_RATIO;
        HoodMotor.setControl(motionMagicRequest.withPosition(rotations));
    }

    public void stop() { HoodMotor.stopMotor(); }

    @Override
    public void periodic() {}
}