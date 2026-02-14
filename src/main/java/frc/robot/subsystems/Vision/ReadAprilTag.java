package frc.robot.subsystems.Vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * A wrapper class for a single PhotonVision camera.
 * Handles both high-level Pose Estimation (Multi-tag) and simple target tracking.
 */
public class ReadAprilTag extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final Transform3d robotToCam; 
    
    private PhotonTrackedTarget bestTarget;

    public ReadAprilTag(String cameraName, Transform3d robotToCam) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCam = robotToCam;

    // Remove 'camera' from this list of arguments
    this.photonPoseEstimator = new PhotonPoseEstimator(
        kTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCam 
    );

    this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * The "Pro" localization method. Uses Multi-tag PNP to find exactly where 
     * the robot is on the field.
     * @param prevEstimatedRobotPose The last known position of the robot to help seed the math.
     */
   public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        var latestResult = camera.getLatestResult(); // You get the result here...
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(latestResult); // ...and pass it to the estimator here.
    }

    /**
     * A simpler method to find the robot's position based on a single target.
     * Useful for checking against the Multi-tag result.
     */
    public Pose3d findRobotPosition() {
        if (bestTarget != null) {
            var tagPoseOptional = kTagLayout.getTagPose(bestTarget.getFiducialId());

            if (tagPoseOptional.isPresent()) {
                return PhotonUtils.estimateFieldToRobotAprilTag(
                    bestTarget.getBestCameraToTarget(), 
                    tagPoseOptional.get(),               
                    robotToCam 
                );
            }
        }
        return null; 
    }

    /**
     * @return The raw PhotonCamera object for low-level result access.
     */
    public PhotonCamera getCamera() {
        return camera;
    }

    /**
     * @return True if the camera sees any valid AprilTags.
     */
    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    /**
     * @return The best target currently seen by this specific camera.
     */
    public PhotonTrackedTarget getBestTarget() {
        return bestTarget;
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();
        
        if (result.hasTargets()) {
            bestTarget = result.getBestTarget(); 
            
            // Logging to help diagnose which camera is doing the heavy lifting
            SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Target", true);
            SmartDashboard.putNumber("Vision/" + camera.getName() + "/Best Tag ID", bestTarget.getFiducialId());
            SmartDashboard.putNumber("Vision/" + camera.getName() + "/Ambiguity", bestTarget.getPoseAmbiguity());
        } else {
            bestTarget = null;
            SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Target", false);
        }
    }
}