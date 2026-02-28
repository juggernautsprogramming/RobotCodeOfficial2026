package frc.robot.subsystems.Vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class ReadAprilTag extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final Transform3d robotToCam; 
    private PhotonPipelineResult lastResult;

    public ReadAprilTag(String cameraName, Transform3d robotToCam) {
        this.camera = new PhotonCamera(cameraName);
        this.robotToCam = robotToCam;

        // FIXED: Added PhotonCamera to the constructor for v2026 API
        this.photonPoseEstimator = new PhotonPoseEstimator(
            kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCam // The 3-argument constructor is common in 2025+
        );

        this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (lastResult == null || !lastResult.hasTargets()) return Optional.empty();
        
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        // FIXED: The update method now requires the result to be passed
        return photonPoseEstimator.update(lastResult);
    }

    public PhotonCamera getCamera() { return camera; }
    public String getName() { return camera.getName(); }
    public PhotonPipelineResult getLatestResult() { return lastResult; }
    /**
     * Calculates the average 3D distance to all currently tracked AprilTags.
     * @return Average distance in meters. Returns 0.0 if no targets are found.
     */
    // Inside ReadAprilTag.java

/**
 * Returns the average distance to all visible tags. 
 * Used to scale "trust" in VisionSubsystem.
 */
public double getAverageDistanceToTags() {
    if (lastResult == null || !lastResult.hasTargets()) return 0.0;

    double totalDistance = 0.0;
    var targets = lastResult.getTargets();
    for (var target : targets) {
        // getBestCameraToTarget() gives the Transform3d from lens to tag center
        totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    return totalDistance / targets.size();
}

/**
 * Returns the ambiguity of the best target. 
 * High ambiguity (near 1.0) means the camera is confused about the tag's orientation.
 */
public double getAmbiguity() {
    if (lastResult == null || !lastResult.hasTargets()) return 1.0;
    return lastResult.getBestTarget().getPoseAmbiguity();
}

    /**
     * Helper to get distance to the single best target (largest area).
     */
    public double getBestTargetDistance() {
        if (lastResult == null || !lastResult.hasTargets()) return 0.0;
        return lastResult.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
    }
   @Override
    public void periodic() {
        lastResult = camera.getLatestResult();
        
        String prefix = "Vision/" + camera.getName() + "/";
        boolean hasTargets = lastResult.hasTargets();
        SmartDashboard.putBoolean(prefix + "Has Target", hasTargets);
        
        if (hasTargets) {
            SmartDashboard.putNumber(prefix + "Best Tag ID", lastResult.getBestTarget().getFiducialId());
            SmartDashboard.putNumber(prefix + "Target Count", lastResult.getTargets().size());
            // This will show up as a number in your Elastic tree
            SmartDashboard.putNumber(prefix + "Avg Distance M", getAverageDistanceToTags());
        }
    }
}