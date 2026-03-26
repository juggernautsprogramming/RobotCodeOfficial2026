package frc.robot.subsystems.Vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * ReadAprilTag — single-camera AprilTag pose estimator wrapper.
 *
 * <p><b>Design:</b> This is a plain (non-Subsystem) data-provider class.
 * {@link VisionSubsystem} calls {@link #update()} explicitly at the top of its
 * periodic, which guarantees that fresh data is always available when
 * VisionSubsystem processes it — no CommandScheduler ordering dependency.
 *
 * <h3>Key behaviours</h3>
 * <ul>
 *   <li><b>All-frames processing</b> — {@link #update()} drains
 *       {@code getAllUnreadResults()} and runs the pose estimator on <em>every</em>
 *       new frame, not just the latest. Each result retains its own FPGA timestamp,
 *       so latency compensation in the Kalman filter is fully utilised.</li>
 *   <li><b>Primary strategy</b> — {@code MULTI_TAG_PNP_ON_COPROCESSOR} for the
 *       highest geometric accuracy when ≥ 2 tags are visible.</li>
 *   <li><b>Fallback strategy</b> — {@code LOWEST_AMBIGUITY} for single-tag frames,
 *       with a hard ambiguity ceiling defined by {@code kAmbiguityThreshold}.</li>
 * </ul>
 */
public class ReadAprilTag {

    private final PhotonCamera        m_camera;
    private final PhotonPoseEstimator m_poseEstimator;
    private final Transform3d         m_robotToCam;

    /** Most recent pipeline result — used by accessors. */
    private PhotonPipelineResult m_latestResult = null;

    /**
     * All new pose estimates produced during the last {@link #update()} call.
     * Cleared and repopulated on every update.
     */
    private final List<EstimatedRobotPose> m_newEstimates = new ArrayList<>();

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * @param cameraName  Must match the name configured in the PhotonVision web UI.
     * @param robotToCam  3-D transform from robot-center to camera lens
     *                    (forward=+X, left=+Y, up=+Z in WPILib convention).
     */
    public ReadAprilTag(String cameraName, Transform3d robotToCam) {
        m_camera     = new PhotonCamera(cameraName);
        m_robotToCam = robotToCam;

        m_poseEstimator = new PhotonPoseEstimator(
            kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            m_robotToCam);

        // Single-tag fallback: pick the candidate with the lowest ambiguity ratio
        m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    // ── Update (called by VisionSubsystem) ───────────────────────────────────

    /**
     * Drains all unread camera frames and runs the pose estimator on each.
     * Must be called once at the start of VisionSubsystem's periodic loop.
     *
     * <p>Processing every frame (rather than discarding intermediates) lets the
     * Kalman filter apply full latency compensation using each frame's FPGA
     * timestamp.
     */
    public void update() {
        m_newEstimates.clear();

        List<PhotonPipelineResult> unread = m_camera.getAllUnreadResults();
        if (unread.isEmpty()) return;

        for (PhotonPipelineResult result : unread) {
            if (!result.hasTargets()) continue;

            // Hard ambiguity gate for single-tag frames before running PnP solve
            if (result.getTargets().size() == 1
                    && result.getBestTarget().getPoseAmbiguity() > kAmbiguityThreshold) {
                continue;
            }

            Optional<EstimatedRobotPose> estimate = m_poseEstimator.update(result);
            estimate.ifPresent(m_newEstimates::add);
        }

        // Keep the chronologically latest result for accessors (distance, ambiguity, etc.)
        m_latestResult = unread.get(unread.size() - 1);

        publishTelemetry();
    }

    // ── Public accessors ──────────────────────────────────────────────────────

    /**
     * All new pose estimates from the last {@link #update()} call, in chronological
     * order. Returns an unmodifiable view — do not mutate.
     */
    public List<EstimatedRobotPose> getNewEstimates() {
        return Collections.unmodifiableList(m_newEstimates);
    }

    /**
     * Average 3-D camera-to-tag distance across all targets in the latest
     * pipeline result (metres). Returns 0.0 when no targets are visible.
     */
    public double getAverageDistanceToTags() {
        if (m_latestResult == null || !m_latestResult.hasTargets()) return 0.0;
        var targets = m_latestResult.getTargets();
        double total = 0.0;
        for (var t : targets) total += t.getBestCameraToTarget().getTranslation().getNorm();
        return total / targets.size();
    }

    /**
     * Pose ambiguity of the best target in the latest result, range [0, 1].
     * Returns 1.0 when no targets are visible (worst-case, maximum penalty).
     */
    public double getAmbiguity() {
        if (m_latestResult == null || !m_latestResult.hasTargets()) return 1.0;
        return m_latestResult.getBestTarget().getPoseAmbiguity();
    }

    /**
     * FPGA timestamp of the most recently received pipeline result (seconds).
     * Returns 0.0 before the first frame arrives.
     */
    public double getLatestResultTimestamp() {
        return m_latestResult != null ? m_latestResult.getTimestampSeconds() : 0.0;
    }

    public PhotonCamera         getCamera()       { return m_camera;       }
    public String               getName()         { return m_camera.getName(); }
    public PhotonPipelineResult getLatestResult() { return m_latestResult; }

    /**
     * Updates the robot-to-camera transform used by the pose estimator.
     * Call each loop for a turret-mounted camera to track the live turret angle.
     */
    public void setRobotToCam(Transform3d newTransform) {
        m_poseEstimator.setRobotToCameraTransform(newTransform);
    }

    /**
     * All 3D robot pose estimates from the last {@link #update()} call, as a
     * {@code Pose3d[]} array ready for AdvantageScope 3D field visualization.
     */
    public Pose3d[] getNewEstimatedPoses3d() {
        Pose3d[] poses = new Pose3d[m_newEstimates.size()];
        for (int i = 0; i < m_newEstimates.size(); i++) {
            poses[i] = m_newEstimates.get(i).estimatedPose;
        }
        return poses;
    }

    /**
     * Camera-to-tag transforms for every target in the latest pipeline result.
     * Useful for visualizing each detected tag's position relative to the camera
     * in AdvantageScope's 3D view.
     * Returns an empty array when no targets are visible.
     */
    public Transform3d[] getCameraToTagTransforms() {
        if (m_latestResult == null || !m_latestResult.hasTargets()) return new Transform3d[0];
        var targets = m_latestResult.getTargets();
        Transform3d[] transforms = new Transform3d[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            transforms[i] = targets.get(i).getBestCameraToTarget();
        }
        return transforms;
    }

    // ── Telemetry ─────────────────────────────────────────────────────────────

    private void publishTelemetry() {
        String  prefix    = "Vision/" + m_camera.getName() + "/";
        boolean hasTarget = m_latestResult != null && m_latestResult.hasTargets();

        SmartDashboard.putBoolean(prefix + "Has Target", hasTarget);
        if (hasTarget) {
            SmartDashboard.putNumber(prefix + "Best Tag ID",
                m_latestResult.getBestTarget().getFiducialId());
            SmartDashboard.putNumber(prefix + "Target Count",
                m_latestResult.getTargets().size());
            SmartDashboard.putNumber(prefix + "Avg Distance M",
                getAverageDistanceToTags());
            SmartDashboard.putNumber(prefix + "Ambiguity",
                m_latestResult.getBestTarget().getPoseAmbiguity());
            SmartDashboard.putNumber(prefix + "New Estimates",
                m_newEstimates.size());
        }
    }
}
