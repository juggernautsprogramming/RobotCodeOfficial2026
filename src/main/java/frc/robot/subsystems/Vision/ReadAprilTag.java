package frc.robot.subsystems.Vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
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
    private PhotonPipelineResult lastResult;

    public ReadAprilTag(String cameraName, Transform3d robotToCam) {
        this.camera = new PhotonCamera(cameraName);
        this.robotToCam = robotToCam;

        // Constructor for v2026 PhotonPoseEstimator
        this.photonPoseEstimator = new PhotonPoseEstimator(
            kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCam 
        );

        this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * The "Pro" localization method.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (lastResult == null || !lastResult.hasTargets()) return Optional.empty();
        
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(lastResult);
    }

    /**
     * Simple field-to-robot estimate for a single tag.
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
        return new Pose3d(); 
    }

    /**
     * @return The average distance to all visible tags in meters.
     */
    public double getAverageDistanceToTags() {
        if (lastResult == null || !lastResult.hasTargets()) return Double.MAX_VALUE;
        
        double totalDistance = 0;
        int count = 0;
        for (var target : lastResult.getTargets()) {
            if (target.getFiducialId() > 0) {
                var tagPose = kTagLayout.getTagPose(target.getFiducialId());
                if (tagPose.isPresent()) {
                    // FIXED: Using calculateDistanceToTargetMeters instead of getDistanceToTarget
                    totalDistance += PhotonUtils.calculateDistanceToTargetMeters(
                        CAMERA_HEIGHT_METERS,
                        tagPose.get().getZ(), // Target height (Z)
                        CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(target.getPitch())
                    );
                    count++;
                }
            }
        }
        return count > 0 ? totalDistance / count : Double.MAX_VALUE;
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public boolean hasTargets() {
        return bestTarget != null;
    }

    public PhotonTrackedTarget getBestTarget() {
        return bestTarget;
    }

    public int getTargetCount() {
        return (lastResult != null && lastResult.hasTargets()) ? lastResult.getTargets().size() : 0;
    }

    /**
     * @return The pose ambiguity (0.0 = certain, 1.0 = uncertain). Returns Double.MAX_VALUE if no targets.
     */
    public double getAmbiguity() {
        if (bestTarget != null) {
            return bestTarget.getPoseAmbiguity();
        }
        return Double.MAX_VALUE;
    }

    /**
     * @return All currently tracked targets for multi-tag analysis
     */
    public List<PhotonTrackedTarget> getAllTargets() {
        if (lastResult != null && lastResult.hasTargets()) {
            return lastResult.getTargets();
        }
        return new ArrayList<>();
    }

    /**
     * @return The timestamp of the last result in seconds
     */
    public double getTimestampSeconds() {
        if (lastResult != null) {
            return lastResult.getTimestampSeconds();
        }
        return 0.0;
    }

    @Override
    public void periodic() {
        lastResult = camera.getLatestResult();
        
        if (lastResult.hasTargets()) {
            bestTarget = lastResult.getBestTarget(); 
            
            SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Target", true);
            SmartDashboard.putNumber("Vision/" + camera.getName() + "/Best Tag ID", bestTarget.getFiducialId());
            SmartDashboard.putNumber("Vision/" + camera.getName() + "/Target Count", lastResult.getTargets().size());
            SmartDashboard.putNumber("Vision/" + camera.getName() + "/Avg Distance", getAverageDistanceToTags());
            SmartDashboard.putNumber("Vision/" + camera.getName() + "/Ambiguity", getAmbiguity());
        } else {
            bestTarget = null;
            SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Target", false);
            SmartDashboard.putNumber("Vision/" + camera.getName() + "/Target Count", 0);
            SmartDashboard.putNumber("Vision/" + camera.getName() + "/Ambiguity", Double.MAX_VALUE);
        }
    }
    /**
 * @return The raw PhotonPipelineResult from the camera.
 */
    public PhotonPipelineResult getLatestResult() {
        return lastResult;
    }

/**
 * @return The camera name for identification.
 */
    public String getName() {
        return camera.getName();
    }
}
