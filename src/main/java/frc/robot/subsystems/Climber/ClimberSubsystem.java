package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private static final int LEADER_ID = 15;
    private static final int FOLLOWER_ID = 16;

    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;

    // --- Gravity Compensation ---
    // kG is the amount of voltage (0 to 12) required to keep the elevator 
    // from falling. Start with 0.0 and increase by 0.1 until it stays still.
    private static final double kG = 0.2; 
    private static final double GEAR_RATIO = 1.0; 

    private static final double CRUISE_VELOCITY = 100; 
    private static final double ACCELERATION = 200;

    // The '.withSlot(0)' ensures it uses your PID gains to hold position
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    public ClimberSubsystem() {
        leaderMotor = new TalonFX(LEADER_ID, "ChassisCAN");
        followerMotor = new TalonFX(FOLLOWER_ID, "ChassisCAN");

        TalonFXConfiguration config = new TalonFXConfiguration();

        // 1. Brake Mode is essential for preventing back-driving
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // 2. Motion Magic Tuning
        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION;

        // 3. PID Gains
        config.Slot0.kP = 2.0; 
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        // Gravity Feedforward (Built into the motor controller)
        config.Slot0.kG = kG; 

        // Apply config
        leaderMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);

        // 4. Set follower AFTER applying config
        followerMotor.setControl(new Follower(LEADER_ID, MotorAlignmentValue.Opposed));
        
        leaderMotor.setPosition(0);
    }

    /**
     * Use this method to move and HOLD the climber.
     * By using MotionMagic, the motor will actively fight gravity to stay at this spot.
     */
    public void setPositionDegrees(double degrees) {
        double rotations = (degrees / 360.0) * GEAR_RATIO;
        leaderMotor.setControl(motionMagicRequest.withPosition(rotations));
    }

    /**
     * Warning: Calling stopMotor() or setPower(0) will likely let the elevator fall 
     * unless your kG is perfectly tuned or your gearbox is high-friction.
     */
    public void setPowerLevel(double power) {
        leaderMotor.set(power);
    }
    public void stopMotors() {
        leaderMotor.stopMotor();
    }
    /**
 * Returns the current position of the leader motor in rotations.
 */
    public double getCurrentPosition() {
        return leaderMotor.getPosition().getValueAsDouble();
    }
    @Override
    public void periodic() {
        // Log position to see if it's drifting
        // SmartDashboard.putNumber("Climber Rotations", leaderMotor.getPosition().getValueAsDouble());
    }
}