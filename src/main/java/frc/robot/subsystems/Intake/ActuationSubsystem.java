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

public class ActuationSubsystem extends SubsystemBase {

    private final TalonFX actuationMotor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    public ActuationSubsystem() {
        actuationMotor = new TalonFX(IntakeConstants.ACTUATION_MOTOR_ID,
            new CANBus(IntakeConstants.ACTUATION_CAN_BUS));

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted    = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.ACTUATION_FORWARD_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.STOWED_ROTATIONS;

        config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.ACTUATION_CRUISE_VEL;
        config.MotionMagic.MotionMagicAcceleration   = IntakeConstants.ACTUATION_ACCELERATION;

        config.Slot0.kP = IntakeConstants.ACTUATION_kP;
        config.Slot0.kI = IntakeConstants.ACTUATION_kI;
        config.Slot0.kD = IntakeConstants.ACTUATION_kD;

        config.Voltage.PeakForwardVoltage =  IntakeConstants.ACTUATION_PEAK_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -IntakeConstants.ACTUATION_PEAK_VOLTAGE;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = actuationMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.err.println("Actuation Motor configuration failed: " + status);
        }

        actuationMotor.setPosition(0);
    }

    /**
     * Run the motor using direct percent output (Duty Cycle).
     * @param power range from -1.0 to 1.0
     */
    public void setPower(double power) {
        actuationMotor.setControl(dutyCycleRequest.withOutput(power));
    }

    /**
     * Move to a specific position using Motion Magic.
     * @param rotations Target position in rotations
     */
    public void setPosition(double rotations) {
        actuationMotor.setControl(motionMagicRequest.withPosition(rotations));
    }

    public void stop() {
        actuationMotor.stopMotor();
    }

    /** Returns current arm position in rotations. */
    public double getCurrentPosition() {
        return actuationMotor.getPosition().getValueAsDouble();
    }

}