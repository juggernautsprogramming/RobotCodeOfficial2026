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

//shooter system either on or off with buttons

public class ShooterSubsystem extends SubsystemBase {

    // CAN IDs for your motors (set these to match your robot's wiring)
    private static final int LEADER_ID = 15;
    private static final int FOLLOWER_ID = 16;

    // TalonFX objects (Kraken X60 motors)
    private TalonFX leaderMotor;
    private TalonFX followerMotor;


    // Encoder constants
    private static final double TICKS_PER_REV = 2048.0; // TalonFX integrated sensor
    private static final double GEAR_RATIO = 1.0;       // Adjust if using gearing

    // Motion Magic settings
    private static final double CRUISE_VELOCITY = 100; // rotations/sec
    private static final double ACCELERATION = 200;    // rotations/sec^2

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);


    public ShooterSubsystem() {
        // Create motor objects
        leaderMotor = new TalonFX(LEADER_ID, "ChassisCAN");
        followerMotor = new TalonFX(FOLLOWER_ID,"ChassisCAN");

        // Set neutral mode (Brake or Coast)  Brake for more accurate stopping
        leaderMotor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);

        // Configure follower
        // followerMotor will mirror leaderMotor's output automatically
        // Ensure you are using 'new' and the parameters are (int, boolean)
        followerMotor.setControl(new Follower(LEADER_ID, MotorAlignmentValue.Aligned));
        // The second parameter is "opposeMaster" — set to true if the motors are mounted in opposite directions


        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Motion Magic tuning
        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION;

        // Configure slot 0 for Voltage Control
        config.Slot0.kP = 2.0; // Example value, tune as needed
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        config.Voltage.withPeakForwardVoltage(8).withPeakReverseVoltage(-8);

        // Configure slot 1 for Torque Control
        config.Slot1.kP = 60.0; // Example value, tune as needed
        config.Slot1.kI = 0.0;
        config.Slot1.kD = 6.0;
        config.TorqueCurrent.withPeakForwardTorqueCurrent(120).withPeakReverseTorqueCurrent(-120);

        // Apply configurations with retries  Leader
        StatusCode leaderMotorstatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            leaderMotorstatus = leaderMotor.getConfigurator().apply(config);
            if (leaderMotorstatus.isOK()) break;
        }
        // Apply configurations with retries Follower
        StatusCode followerMotorstatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            followerMotorstatus = followerMotor.getConfigurator().apply(config);
            if (followerMotorstatus.isOK()) break;
        }


	
        if (!leaderMotorstatus.isOK()) {
            System.out.println("Motor configuration failed: " + leaderMotorstatus);
        }
  

        leaderMotor.setPosition(0); // Start at 0
    }



    
    public void disabledInit() {
        // Stop motors when disabled
        leaderMotor.stopMotor();
        followerMotor.stopMotor();
    }


    private void moveToDegrees(double degrees)
    {
     
        //Use encoder   forward 90 degrees back to 0 degree is backward
        // Convert degrees to rotations
        double rotations = (degrees / 360.0) * GEAR_RATIO;

        // Command Motion Magic
        leaderMotor.setControl(motionMagicRequest.withPosition(rotations));

    }


    public void setPowerLevel(double Power) 
    {
        // Example: run leader at 50% output
        leaderMotor.set(Power);
        // followerMotor automatically follows — no need to set it here
    }


    
    public void stopMotors() 
    {
        // Stop motors when disabled
        leaderMotor.stopMotor();
        followerMotor.stopMotor();

    }

    @Override
    public void periodic() 
    {
         //maybe display things to tab
    
    }

    public void teleopPeriodic() {
        // Example: run leader at 50% output
        leaderMotor.set(0.5);
        // followerMotor automatically follows — no need to set it here
    }

}

