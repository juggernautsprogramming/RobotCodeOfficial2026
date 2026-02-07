package frc.robot.subsystems.Vision;

import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.Constants.VisionHardware.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget; // Added import

public class ReadAprilTag extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private PhotonTrackedTarget bestTarget; // Added so FindRobotPosition can see it

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
     * Calculates the Robot's position on the field using a single tag and PhotonUtils.
     */
    public Pose3d FindRobotPosition() {
        // 1. Check if we actually have a target saved in our periodic loop
        if (bestTarget != null) {
            var tagPoseOptional = kTagLayout.getTagPose(bestTarget.getFiducialId());

            // 2. Check if the ID we see actually exists on the official field map
            if (tagPoseOptional.isPresent()) {
                // 3. Calculate the Robot's position on the field
                return PhotonUtils.estimateFieldToRobotAprilTag(
                    bestTarget.getBestCameraToTarget(), 
                    tagPoseOptional.get(),              
                    kRobotToCam 
                );
            }
        }
        return null; 
    }

    /**
     * The main function for high-accuracy localization.
     * Combines multiple tags to find the robot's pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        var latestResult = camera.getLatestResult();
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(latestResult); 
    }

    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();
        
        if (result.hasTargets()) {
            bestTarget = result.getBestTarget(); // Update class variable
            
            SmartDashboard.putBoolean("Vision/Has Target", true);
            SmartDashboard.putNumber("Vision/Best Tag ID", bestTarget.getFiducialId());
            SmartDashboard.putNumber("Vision/Ambiguity", bestTarget.getPoseAmbiguity());
        } else {
            bestTarget = null;
            SmartDashboard.putBoolean("Vision/Has Target", false);
        }
    }
}