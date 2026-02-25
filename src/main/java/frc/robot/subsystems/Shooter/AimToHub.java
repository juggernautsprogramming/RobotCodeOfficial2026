package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

public class AimToHub extends SubsystemBase {

    // --- TUNING CONSTANTS ---
    // Target Height (approx 2.64m) - Camera Height (approx 0.5m)
    private final double DELTA_HEIGHT = Units.inchesToMeters(104.0 - 20.0); 
    // The fixed angle of your camera relative to the ground
    private final double CAMERA_MOUNT_ANGLE_RAD = Units.degreesToRadians(35.0);
    // Approximate speed of the ball leaving the shooter (m/s)
    private final double BALL_EXIT_VELOCITY = 15.0; 

    private final TalonFX aimMotor;
    private final MotionMagicVoltage positionRequest;
    private final double GEAR_RATIO = 10.0; 

    public AimToHub() {
        aimMotor = new TalonFX(1);
        positionRequest = new MotionMagicVoltage(0).withSlot(0);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Slot0.kP = 2.4; 
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.1;
        configs.Slot0.kV = 0.12;

        configs.MotionMagic.MotionMagicCruiseVelocity = 10; 
        configs.MotionMagic.MotionMagicAcceleration = 20; 
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        StatusCode status = aimMotor.getConfigurator().apply(configs);
        if (!status.isOK()) {
            System.out.println("Could not configure AimMotor! Error: " + status);
        }
    }

    /**
     * Estimates distance to the hub using camera vertical offset (ty).
     */
    public double getEstimateDistanceToTarget(double ty) {
        double totalAngleRad = CAMERA_MOUNT_ANGLE_RAD + Units.degreesToRadians(ty);
        return DELTA_HEIGHT / Math.tan(totalAngleRad);
    }

    /**
     * Main logic called by the Shooter Subsystem.
     * Calculates the required horizontal angle (yaw/pivot) to lead the target.
     * * @param vry Robot velocity in Y (m/s) - lateral/strafe
     * @param vrx Robot velocity in X (m/s) - forward/backward
     * @param ty Vertical offset from vision camera
     * @return The angle in degrees the robot/turret should aim
     */
    public double calculateMovingAimAngle(double vry, double vxMetersPerSecond, double ty) {
        double distance = getEstimateDistanceToTarget(ty);
        
        // Time of flight = Distance / Velocity
        double t = distance / BALL_EXIT_VELOCITY;

        // Xtarget: Where the goal will be relative to the robot when the ball arrives
        // If we move forward (positive vx), the goal "moves closer"
        double xTarget = distance - (vxMetersPerSecond * t);
        
        // YTarget: Side-to-side drift
        // If we strafe right (positive vy), the goal "moves left"
        double yTarget = 0.0 - (vry * t);
        
        // Calculate the yaw angle to hit that virtual point
        double virtualAimAngleRadians = Math.atan2(yTarget, xTarget);
        
        return Math.toDegrees(virtualAimAngleRadians);
    }

    /**
     * Optional: Returns the distance the shooter should actually "act" like it's at.
     * If you are driving away from the goal, you need to shoot harder (longer virtual distance).
     */
    public double getVirtualShotDistance(double vrx, double ty) {
        double distance = getEstimateDistanceToTarget(ty);
        double t = distance / BALL_EXIT_VELOCITY;
        double xTarget = distance - (vrx * t);
        double yTarget = 0.0; // Assuming centering handles Y, but you could include it
        
        return Math.sqrt(Math.pow(xTarget, 2) + Math.pow(yTarget, 2));
    }

    public void setAimAngle(double targetAngleDegrees) {
        double mechanismRotations = targetAngleDegrees / 360.0;
        double motorRotations = mechanismRotations * GEAR_RATIO;
        aimMotor.setControl(positionRequest.withPosition(motorRotations));
    }
}