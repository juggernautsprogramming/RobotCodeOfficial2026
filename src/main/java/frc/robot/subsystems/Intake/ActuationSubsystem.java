package frc.robot.subsystems.Intake; // Ensure your folder structure matches this

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ActuationSubsystem extends SubsystemBase {

    // Hardware Constants
    private static final int MOTOR_ID = 43;
    
    // Motion Magic settings (Phoenix 6 uses Rotations, not Ticks)
    private static final double CRUISE_VELOCITY = 100; // rotations/sec
    private static final double ACCELERATION = 200;    // rotations/sec^2

    // Member Variables (Declared here so all methods can see them)
    private final TalonFX actuationMotor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    public ActuationSubsystem() {
        actuationMotor = new TalonFX(MOTOR_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // 1. Motor Output & Neutral Mode
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 2. Soft Limits (Hard limits replacement)
        // Adjust these numbers based on your mechanism's physical range
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 50; // Rotations
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;  // Rotations

        // 3. Motion Magic tuning
        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION;

        // 4. Slot 0 Gains (Voltage Control)
        config.Slot0.kP = 2.0; 
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        
        config.Voltage.PeakForwardVoltage = 8;
        config.Voltage.PeakReverseVoltage = -8;

        // Apply configurations
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = actuationMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.err.println("Actuation Motor configuration failed: " + status);
        }

        // Zero the sensor on boot
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

    @Override
    public void periodic() {
        // This runs every 20ms. Useful for logging or dashboard updates.
    }
}