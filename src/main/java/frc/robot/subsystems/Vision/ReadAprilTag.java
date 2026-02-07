package frc.robot.subsystems.Vision;

import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.Constants.VisionHardware.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class ReadAprilTag extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;

    public ReadAprilTag() {
        // Initialize the camera using the name from Constants
        camera = new PhotonCamera(CAMERA_OBJECT_NAME);

        // Set up the Pose Estimator to transform "Camera Space" to "Field Space"
        photonPoseEstimator = new PhotonPoseEstimator(
            kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            kRobotToCam
        );

        // Fallback strategy if only one tag is visible
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * The main function to call from your Drivetrain or Commands.
     * It looks at the camera and tries to figure out where the robot is on the field.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        // 1. Get the latest result from the camera hardware
        var latestResult = camera.getLatestResult();
    
        // 2. Set the reference pose (where the robot thinks it is)
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        
        // 3. Pass that 'latestResult' into the update method
        return photonPoseEstimator.update(latestResult); 
    }

    /**
     * Helper method to check if the camera sees any targets at all.
     */
    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();
    
        // This tells you if the camera is actually seeing a valid tag
        SmartDashboard.putBoolean("Vision/Has Target", result.hasTargets());
    
        if (result.hasTargets()) {
            SmartDashboard.putNumber("Vision/Best Tag ID", result.getBestTarget().getFiducialId());
            // Ambiguity: 0.0 is perfect, 1.0 is "I have no idea which way this tag is facing"
            SmartDashboard.putNumber("Vision/Ambiguity", result.getBestTarget().getPoseAmbiguity());
        }
    }
}