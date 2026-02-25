package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Shooter system that uses Motion Magic for angle control and 
 * compensates for robot velocity during aiming.
 */
public class ShooterSubsystem extends SubsystemBase {

    // CAN IDs for your motors
    private static final int LEADER_ID = 15;
    private static final int FOLLOWER_ID = 16;

    // Hardware
    private TalonFX leaderMotor;
    private TalonFX followerMotor;

    // Targeting Logic Helper
    private final AimToHub aimLogic;

    // Constants
    private static final double GEAR_RATIO = 1.0; // Adjust to your actual pivot gearing
    private static final double CRUISE_VELOCITY = 100; // rotations/sec
    private static final double ACCELERATION = 200;    // rotations/sec^2

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    public ShooterSubsystem() {
        this.aimLogic = new AimToHub();
        
        leaderMotor = new TalonFX(LEADER_ID, "ChassisCAN");
        followerMotor = new TalonFX(FOLLOWER_ID, "ChassisCAN");

        leaderMotor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);

        // Configure follower to mirror leader
        followerMotor.setControl(new Follower(LEADER_ID, MotorAlignmentValue.Aligned));

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Motion Magic tuning
        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION;

        // PID Tuning (Slot 0)
        config.Slot0.kP = 2.0; 
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;

        // Apply configs
        applyMotorConfigs(config);

        leaderMotor.setPosition(0);
    }

    /**
     * Helper to apply configs with retries for stability.
     */
    private void applyMotorConfigs(TalonFXConfiguration config) {
        StatusCode leaderStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            leaderStatus = leaderMotor.getConfigurator().apply(config);
            if (leaderStatus.isOK()) break;
        }
        
        if (!leaderStatus.isOK()) {
            System.out.println("Shooter Leader Config Failed: " + leaderStatus);
        }
    }

    /**
     * Commands the shooter pivot to a specific angle using Motion Magic.
     */
    public void moveToDegrees(double degrees) {
        double rotations = (degrees / 360.0) * GEAR_RATIO;
        leaderMotor.setControl(motionMagicRequest.withPosition(rotations));
    }

    /**
     * The main auto-aiming method. Call this in a command while holding a button.
     * @param robotSpeeds Current ChassisSpeeds (vx, vy) from the drivetrain.
     */
    public void runAutoAim(ChassisSpeeds robotSpeeds) {
        // 1. Get Limelight data
        var table = NetworkTableInstance.getDefault().getTable("limelight");
        double ty = table.getEntry("ty").getDouble(0.0);
        double tv = table.getEntry("tv").getDouble(0.0);

        // 2. Only calculate if we see the hub
        if (tv < 1.0) {
            return; // Stay at current position or go to home
        }

        // 3. Use AimToHub to find the 'moving' angle compensation
        // This accounts for the robot sliding left/right/forward while shooting
        double targetAngle = aimLogic.calculateMovingAimAngle(
            robotSpeeds.vyMetersPerSecond, 
            robotSpeeds.vxMetersPerSecond, 
            ty
        );

        // 4. Move pivot to that angle
        moveToDegrees(targetAngle);
    }

    public void setPowerLevel(double power) {
        leaderMotor.set(power);
    }

    public void stopMotors() {
        leaderMotor.stopMotor();
        followerMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // Standard monitoring (SmartDashboard, etc.) can go here
    }
}