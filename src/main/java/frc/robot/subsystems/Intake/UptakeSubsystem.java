package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * UptakeSubsystem — intake uptake roller (CAN ID 42, ChassisCAN bus).
 */
public class UptakeSubsystem extends SubsystemBase {

    private static final int    LEADER_ID       = 42;
    private static final String CAN_BUS         = "ChassisCAN";
    private static final double CRUISE_VELOCITY = 100;
    private static final double ACCELERATION    = 200;

    private final TalonFX            uptakeMotor;
    private final DutyCycleOut       dutyCycleRequest   = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    public UptakeSubsystem() {
        // FIX: new CANBus(CAN_BUS) — not new String(CAN_BUS)
        uptakeMotor = new TalonFX(LEADER_ID, new CANBus(CAN_BUS));

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
            status = uptakeMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.err.println("Uptake Motor configuration failed: " + status);
        }

        uptakeMotor.setPosition(0);
    }

    /** Sets motor power using Duty Cycle (-1.0 to 1.0). */
    public void setPower(double power) {
        uptakeMotor.setControl(dutyCycleRequest.withOutput(power));
    }

    /** Move to a specific position using Motion Magic. */
    public void setPosition(double targetRotations) {
        uptakeMotor.setControl(motionMagicRequest.withPosition(targetRotations));
    }

    public void stopMotors() {
        uptakeMotor.stopMotor();
    }

    @Override
    public void periodic() {}
}