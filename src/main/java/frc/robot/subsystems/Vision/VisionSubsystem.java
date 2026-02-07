// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase
{
    private final PhotonCamera camera;
    private PhotonTrackedTarget bestTarget;
    private final PIDController strafePID;
    private final SimpleMotorFeedforward strafeFeedforward;
    private final ReadAprilTag m_vision = new ReadAprilTag();
      
    public VisionSubsystem(String string) {
        camera = new PhotonCamera(Constants.VisionHardware.CAMERA_OBJECT_NAME);

        strafePID = new PIDController(VisionConstants.kP, VisionConstants.kI, VisionConstants.kD);
        strafePID.setTolerance(VisionConstants.angleTolerance);

        strafeFeedforward = new SimpleMotorFeedforward(VisionConstants.kS, VisionConstants.kV, VisionConstants.kA);
        
    }
    
    public boolean hasValidTarget() {
        return bestTarget != null;
    }
    public int getBestTagId() {
        return (bestTarget != null) ? bestTarget.getFiducialId() : -1;
    }
    public double getYawError()
    {
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            return result.getBestTarget().getYaw();
        }
        return 0.0;
        }
    


    public double getStrafeSpeedWithOffset(double offsetMeters) { 
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            bestTarget = result.getBestTarget();
    
            double yawError = bestTarget.getYaw(); // Use angle instead of X position
        double pidOutput = strafePID.calculate(yawError, 0);
        double desiredVelocity = strafePID.getVelocityError();
        double feedforwardOutput = strafeFeedforward.calculate(desiredVelocity);
        double strafeSpeed = pidOutput;

        if (Math.abs(strafeSpeed) < 0.05) {
            strafeSpeed = 0;
        }

        // Debugging prints
        System.out.println("Yaw Error: " + yawError);
        System.out.println("PID Output: " + pidOutput);
        System.out.println("Desired Velocity: " + desiredVelocity);
        System.out.println("Feedforward Output: " + feedforwardOutput);
        System.out.println("Final Strafe Speed: " + strafeSpeed);

        return strafeSpeed;
    } else {
        System.out.println("No target detected!");
    }
    return 0.0;
}

    @Override
public void periodic() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
        bestTarget = result.getBestTarget();

        // Push all target data to the SmartDashboard/Shuffleboard
        SmartDashboard.putNumber("Vision/Tag ID", bestTarget.getFiducialId());
        SmartDashboard.putNumber("Vision/Yaw (Left-Right)", bestTarget.getYaw());
        SmartDashboard.putNumber("Vision/Pitch (Up-Down)", bestTarget.getPitch());
        SmartDashboard.putNumber("Vision/Area (Size)", bestTarget.getArea());
        SmartDashboard.putBoolean("Vision/Has Target", true);
    } else {
        bestTarget = null;
        
        // Reset values when target is lost
        SmartDashboard.putNumber("Vision/Tag ID", -1);
        SmartDashboard.putBoolean("Vision/Has Target", false);
    }
}

    /** Returns the current best target object for detailed data access. */
    public org.photonvision.targeting.PhotonTrackedTarget getBestTarget() {
        return bestTarget;
    }
    
}


// package frc.robot.subsystems;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonTrackedTarget;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.VisionConstants;

// public class VisionSubsystem extends SubsystemBase {
//     private final PhotonCamera camera;
//     private PhotonTrackedTarget bestTarget;
//     private final PIDController strafePID;
//     private final SimpleMotorFeedforward strafeFeedforward;

//     private static final double CAMERA_YAW_OFFSET = 0 ; // Adjust based on camera mounting angle

//     public VisionSubsystem(String cameraName) {
//         camera = new PhotonCamera(cameraName);
//         strafePID = new PIDController(VisionConstants.kP, VisionConstants.kI, VisionConstants.kD);
//         strafePID.setTolerance(VisionConstants.angleTolerance);
//         strafeFeedforward = new SimpleMotorFeedforward(VisionConstants.kS, VisionConstants.kV, VisionConstants.kA);
//     }

//     /** Checks if a valid target is detected */
//     public boolean hasValidTarget() {
//         return bestTarget != null;
//     }

//     /** Computes strafing speed with an optional offset */
//     public double getStrafeSpeedWithOffset(double offsetMeters) {
//         if (!hasValidTarget()) {
//             return 0.0;
//         }

//         double yawError = bestTarget.getYaw() - CAMERA_YAW_OFFSET; // Adjust for side-mounted camera
//         double pidOutput = strafePID.calculate(yawError, offsetMeters);
//         double feedforwardOutput = strafeFeedforward.calculate(pidOutput);
//         double strafeSpeed = pidOutput + feedforwardOutput;

//         // Enforce minimum movement speed
//         double minSpeed = 0.12; // Prevent slow creeping
//         if (Math.abs(strafeSpeed) < minSpeed && Math.abs(yawError) > VisionConstants.angleTolerance) {
//             strafeSpeed = Math.copySign(minSpeed, strafeSpeed);
//         }

//         return strafeSpeed;
//     }

//     @Override
//     public void periodic() {
//         var result = camera.getLatestResult();
//         bestTarget = result.hasTargets() ? result.getBestTarget() : null;
//     }
// }




// // package frc.robot.subsystems;

// // import org.photonvision.PhotonCamera;
// // import org.photonvision.targeting.PhotonTrackedTarget;

// // import edu.wpi.first.math.controller.PIDController;
// // import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// // import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// // import edu.wpi.first.wpilibj2.command.SubsystemBase;
// // import frc.robot.Constants.VisionConstants;

// // public class VisionSubsystem extends SubsystemBase {
// //     private final PhotonCamera camera;
// //     private PhotonTrackedTarget bestTarget;
// //     private final PIDController strafePID;
// //     private final SimpleMotorFeedforward strafeFeedforward;

// //     // Camera and AprilTag properties
// //     private static final double CAMERA_HEIGHT_METERS = 0.2;  // Camera height from the ground in meters
// //     private static final double CAMERA_ANGLE_DEGREES = 30.0;  // Camera tilt angle (adjust if necessary)
// //     private static final double TAG_HEIGHT_METERS = 0.33; // Height of the AprilTag in meters (e.g., 25.4 cm)

// //     public VisionSubsystem(String FrontCenter) {
// //         camera = new PhotonCamera(FrontCenter);

// //         strafePID = new PIDController(VisionConstants.kP, VisionConstants.kI, VisionConstants.kD);
// //         strafePID.setTolerance(VisionConstants.angleTolerance);

// //         strafeFeedforward = new SimpleMotorFeedforward(VisionConstants.kS, VisionConstants.kV, VisionConstants.kA);
// //     }

// //     public boolean hasValidTarget() {
// //         return bestTarget != null;
// //     }

// //     // Get strafe speed with an offset
// //     public double getStrafeSpeedWithOffset(double offsetMeters) {
// //         var result = camera.getLatestResult();
// //         if (result.hasTargets()) {
// //             bestTarget = result.getBestTarget();

// //             double yawError = bestTarget.getYaw(); // Use yaw for alignment
// //             double pidOutput = strafePID.calculate(yawError, offsetMeters);
// //             double desiredVelocity = strafePID.getVelocityError();
// //             double feedforwardOutput = strafeFeedforward.calculate(desiredVelocity);
// //             double strafeSpeed = pidOutput + feedforwardOutput;

// //             if (Math.abs(strafeSpeed) < 0.1) {
// //                 strafeSpeed = Math.signum(strafeSpeed) * 0.1;
// //             }

// //             // Debugging prints
// //             System.out.println("Yaw Error: " + yawError);
// //             System.out.println("PID Output: " + pidOutput);
// //             System.out.println("Desired Velocity: " + desiredVelocity);
// //             System.out.println("Feedforward Output: " + feedforwardOutput);
// //             System.out.println("Final Strafe Speed: " + strafeSpeed);

// //             return strafeSpeed;
// //         } else {
// //             System.out.println("No target detected!");
// //         }
// //         return 0.0;
// //     }

// //     // Calculate distance to target using camera's vertical angle and target height
// //     public double getDistanceToTarget() {
// //         var result = camera.getLatestResult();
// //         if (result.hasTargets()) {
// //             bestTarget = result.getBestTarget();

// //             // Get the vertical angle to the target (ty)
// //             double verticalAngle = bestTarget.getPitch();  // Assuming pitch angle is used

// //             // Convert angle to radians
// //             double angleRadians = Math.toRadians(CAMERA_ANGLE_DEGREES + verticalAngle);

// //             // Calculate the distance using basic trigonometry
// //             double distance = TAG_HEIGHT_METERS / Math.tan(angleRadians);

// //             // Optionally, display the distance for debugging
// //             SmartDashboard.putNumber("Distance to Target", distance);

// //             return distance;
// //         } else {
// //             System.out.println("No target detected!");
// //         }
// //         return 0.0;
// //     }

// //     @Override
// //     public void periodic() {
// //         var result = camera.getLatestResult();
// //         bestTarget = result.hasTargets() ? result.getBestTarget() : null;
// //     }
// // }
