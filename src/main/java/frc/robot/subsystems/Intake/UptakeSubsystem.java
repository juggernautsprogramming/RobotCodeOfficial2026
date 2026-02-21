package frc.robot.subsystems.Intake; // Fixed package declaration semicolon

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UptakeSubsystem extends SubsystemBase {

    // Hardware Constants
    private static final int LEADER_ID = 42;
    private static final String CAN_BUS = "ChassisCAN";

    // Motion Magic settings (Rotations based for Phoenix 6)
    private static final double CRUISE_VELOCITY = 100; // rotations/sec
    private static final double ACCELERATION = 200;    // rotations/sec^2

    // Hardware and Control Objects
    private final TalonFX uptakeMotor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    public UptakeSubsystem() {
        uptakeMotor = new TalonFX(LEADER_ID, CAN_BUS);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor Output Settings
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Motion Magic tuning
        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION;

        // Slot 0 for Motion Magic / Position Control
        config.Slot0.kP = 2.0; 
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        
        // Limits
        config.Voltage.PeakForwardVoltage = 8;
        config.Voltage.PeakReverseVoltage = -8;

        // Apply configurations with retries
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

    /** Sets motor power using Duty Cycle (-1.0 to 1.0) */
    public void setPower(double power) {
        uptakeMotor.setControl(dutyCycleRequest.withOutput(power));
    }

    /** Move to a specific position using Motion Magic */
    public void setPosition(double targetRotations) {
        uptakeMotor.setControl(motionMagicRequest.withPosition(targetRotations));
    }

    public void stopMotors() {
        uptakeMotor.stopMotor();
    }

    
    public void periodic() {
        // SmartDashboard.putNumber("Uptake Position", uptakeMotor.getPosition().getValueAsDouble());
    }
}