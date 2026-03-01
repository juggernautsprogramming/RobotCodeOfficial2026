package frc.robot.subsystems.Vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * ReadAprilTag — single-camera wrapper using the PhotonVision 2026 API.
 *
 * <h3>Deprecation fixes vs the previous version</h3>
 * <table>
 *   <tr><th>Old (deprecated)</th><th>New (2026)</th></tr>
 *   <tr><td>{@code camera.getLatestResult()}</td>
 *       <td>{@code camera.getAllUnreadResults()} — returns every frame since
 *           the last call; we keep the freshest one</td></tr>
 *   <tr><td>{@code new PhotonPoseEstimator(layout, strategy, Transform3d)}</td>
 *       <td>{@code new PhotonPoseEstimator(layout, strategy, camera, robotToCam)}
 *           — 4-arg constructor that owns the camera reference</td></tr>
 *   <tr><td>{@code estimator.setReferencePose()} +
 *           {@code estimator.update(result)}</td>
 *       <td>{@code estimator.update(result)} — reference pose no longer needed;
 *           the estimator resolves ambiguity internally</td></tr>
 *   <tr><td>{@code estimator.setMultiTagFallbackStrategy()}</td>
 *       <td>Not needed — LOWEST_AMBIGUITY is the default fallback in 2026</td></tr>
 * </table>
 */
public class ReadAprilTag extends SubsystemBase {

    private final PhotonCamera        m_camera;
    private final PhotonPoseEstimator m_poseEstimator;
    private final Transform3d         m_robotToCam;

    /** Most recent pipeline result — updated exactly once per loop tick. */
    private PhotonPipelineResult m_lastResult = null;

    // ── Constructor ───────────────────────────────────────────────────────────

    public ReadAprilTag(String cameraName, Transform3d robotToCam) {
        m_camera     = new PhotonCamera(cameraName);
        m_robotToCam = robotToCam;

        // 2026 non-deprecated constructor: camera + robotToCam are passed directly.
        m_poseEstimator = new PhotonPoseEstimator(
            kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            m_robotToCam);
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Estimate the robot's global pose from the latest accepted frame.
     * Returns empty when no result, no targets, or ambiguity is too high.
     *
     * @param prevEstimatedPose Kept for API compatibility with VisionSubsystem;
     *                          the 2026 estimator no longer requires a reference pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedPose) {
        if (m_lastResult == null || !m_lastResult.hasTargets()) return Optional.empty();

        // Reject ambiguous single-tag frames before running the expensive PnP solve
        if (m_lastResult.getTargets().size() == 1
                && m_lastResult.getBestTarget().getPoseAmbiguity() > kAmbiguityThreshold) {
            return Optional.empty();
        }

        // 2026 API: update(result) — no setReferencePose() call needed
        return m_poseEstimator.update(m_lastResult);
    }

    /** Average 3D distance to all currently tracked tags (metres). 0 if none visible. */
    public double getAverageDistanceToTags() {
        if (m_lastResult == null || !m_lastResult.hasTargets()) return 0.0;
        var targets = m_lastResult.getTargets();
        double total = 0.0;
        for (var t : targets) total += t.getBestCameraToTarget().getTranslation().getNorm();
        return total / targets.size();
    }

    /** Pose ambiguity of the best visible target [0, 1]. Returns 1.0 when none visible. */
    public double getAmbiguity() {
        if (m_lastResult == null || !m_lastResult.hasTargets()) return 1.0;
        return m_lastResult.getBestTarget().getPoseAmbiguity();
    }

    /** Distance to the single best target (metres). 0 if none visible. */
    public double getBestTargetDistance() {
        if (m_lastResult == null || !m_lastResult.hasTargets()) return 0.0;
        return m_lastResult.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
    }

    /**
     * FPGA timestamp (seconds) of the most recent pipeline result.
     * Returns 0.0 if no result has been received yet.
     */
    public double getLatestResultTimestamp() {
        return m_lastResult != null ? m_lastResult.getTimestampSeconds() : 0.0;
    }

    public PhotonCamera         getCamera()       { return m_camera; }
    public String               getName()         { return m_camera.getName(); }
    public PhotonPipelineResult getLatestResult() { return m_lastResult; }

    // ── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // getAllUnreadResults() returns every new frame since the last call.
        // Taking the last element gives us the freshest frame with no skipped frames.
        // This replaces the deprecated getLatestResult() which could return stale data.
        List<PhotonPipelineResult> unread = m_camera.getAllUnreadResults();
        if (!unread.isEmpty()) {
            m_lastResult = unread.get(unread.size() - 1);
        }

        String  prefix    = "Vision/" + m_camera.getName() + "/";
        boolean hasTarget = m_lastResult != null && m_lastResult.hasTargets();

        SmartDashboard.putBoolean(prefix + "Has Target", hasTarget);
        if (hasTarget) {
            SmartDashboard.putNumber(prefix + "Best Tag ID",
                m_lastResult.getBestTarget().getFiducialId());
            SmartDashboard.putNumber(prefix + "Target Count",
                m_lastResult.getTargets().size());
            SmartDashboard.putNumber(prefix + "Avg Distance M",
                getAverageDistanceToTags());
            SmartDashboard.putNumber(prefix + "Ambiguity",
                m_lastResult.getBestTarget().getPoseAmbiguity());
        }
    }
}