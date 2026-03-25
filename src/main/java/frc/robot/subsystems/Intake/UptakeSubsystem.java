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
import frc.robot.Constants.IntakeConstants;

/**
 * UptakeSubsystem — intake uptake roller (CAN ID 42, ChassisCAN bus).
 */
public class UptakeSubsystem extends SubsystemBase {

    private final TalonFX            uptakeMotor;
    private final DutyCycleOut       dutyCycleRequest   = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    public UptakeSubsystem() {
        uptakeMotor = new TalonFX(IntakeConstants.UPTAKE_MOTOR_ID,
            new CANBus(IntakeConstants.UPTAKE_CAN_BUS));

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.UPTAKE_CRUISE_VEL;
        config.MotionMagic.MotionMagicAcceleration   = IntakeConstants.UPTAKE_ACCELERATION;

        config.Slot0.kP = IntakeConstants.UPTAKE_kP;
        config.Slot0.kI = IntakeConstants.UPTAKE_kI;
        config.Slot0.kD = IntakeConstants.UPTAKE_kD;
        config.Voltage.PeakForwardVoltage  =  IntakeConstants.UPTAKE_PEAK_VOLTAGE;
        config.Voltage.PeakReverseVoltage  = -IntakeConstants.UPTAKE_PEAK_VOLTAGE;

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
}