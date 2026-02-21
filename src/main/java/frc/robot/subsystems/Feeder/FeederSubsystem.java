package frc.robot.subsystems.Feeder;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {

    // --- Constants ---
    private static final int MOTOR_ID = 44; // Give it a unique ID
    private static final String CAN_BUS = "ChassisCAN";
    private static final double CRUISE_VELOCITY = 100; 
    private static final double ACCELERATION = 200;

    // --- Hardware and Control Objects ---
    // Declaring these here makes them accessible to ALL methods in the class
    private final TalonFX feederMotor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    public FeederSubsystem() {
        feederMotor = new TalonFX(MOTOR_ID, CAN_BUS);
        
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Basic Motor Settings
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Motion Magic tuning
        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION;

        // Slot 0 for Position/Voltage Control
        config.Slot0.kP = 2.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        config.Voltage.PeakForwardVoltage = 8;
        config.Voltage.PeakReverseVoltage = -8;

        // Apply config with retries
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

    /**
     * Set motor power using Duty Cycle (-1.0 to 1.0)
     */
    public void setPower(double power) {
        feederMotor.setControl(dutyCycleRequest.withOutput(power));
    }

    /**
     * Stop the motor safely
     */
    public void stop() {
        feederMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // This runs every 20ms - good for SmartDashboard updates
    }
}