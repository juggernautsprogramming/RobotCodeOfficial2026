package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionHardware;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.ShotCalculator;

/**
 * VisionSubsystem — multi-camera PhotonVision pose fusion for FRC 2026.
 *
 * <h3>Accuracy + latency optimisations</h3>
 * <ul>
 *   <li><b>Single result fetch per sensor per loop</b> — {@code periodic()} reads
 *       each camera's cached result from {@code ReadAprilTag}. There are no duplicate
 *       JNI calls anywhere in this class.</li>
 *   <li><b>Ambiguity guard in {@code ReadAprilTag}</b> — frames with high pose
 *       ambiguity are rejected before the expensive PnP solve runs.</li>
 *   <li><b>Distance-squared std-dev model</b> — the Kalman filter trusts
 *       measurements more when the robot is close:
 *       {@code stdDev = 0.1 + dist² × 0.05}.</li>
 *   <li><b>Vision freshness check</b> — measurements older than
 *       {@link #VISION_STALENESS_THRESHOLD_S} are silently dropped, preventing
 *       the estimator from being poisoned by stale frames after a camera dropout.</li>
 *   <li><b>Multi-tag bonus</b> — when ≥ 2 tags are visible the std-dev is halved,
 *       giving the Kalman filter higher confidence in multi-tag solves.</li>
 *   <li><b>Alliance origin set once</b> — tag layout origin is updated exactly
 *       once when the DS connects, not every loop.</li>
 * </ul>
 */
public class VisionSubsystem extends SubsystemBase {

    /** Drop vision measurements older than this (seconds). */
    private static final double VISION_STALENESS_THRESHOLD_S = 0.150;

    private final CommandSwerveDrivetrain m_drivetrain;
    private final List<ReadAprilTag>      m_sensors = new ArrayList<>();

    /** Kalman-fused robot pose visualisation. */
    private final Field2d m_field        = new Field2d();
    /** Per-camera raw pose estimate visualisation ("ghosts"). */
    private final Field2d m_fieldCameras = new Field2d();

    /** Best overall target (highest pixel area) across all cameras. */
    private PhotonTrackedTarget m_bestTarget    = null;
    /** Best hub-specific AprilTag target. */
    private PhotonTrackedTarget m_bestHubTarget = null;
    /** Distance to hub derived from best hub tag (metres). */
    private double m_hubDistMeters = 0.0;
    /** FPGA timestamp of the most recently accepted hub measurement (seconds). */
    private double m_lastHubResultTimestamp = 0.0;

    /** Most recent accepted vision pose estimate (for ghost overlay). */
    private Pose2d m_lastVisionPose   = null;
    /** FPGA timestamp (s) of {@link #m_lastVisionPose} — used for ghost fade. */
    private double m_lastVisionTimeS  = 0.0;
    /** Hide vision ghost after this many seconds with no accepted estimate. */
    private static final double GHOST_FADE_S = 0.5;

    private boolean m_allianceSet = false;

    /** PID for raw yaw-alignment rotation commands (teleop AlignToTag use). */
    private final PIDController m_strafePID;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * @param cameraNames Names matching the PhotonVision web UI — use
     *                    {@link VisionHardware} constants.
     * @param drivetrain  Swerve drivetrain whose pose estimator will be fed.
     */
    public VisionSubsystem(String[] cameraNames, CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_strafePID  = new PIDController(
            VisionConstants.kP, VisionConstants.kI, VisionConstants.kD);
        m_strafePID.setTolerance(VisionConstants.angleTolerance);

        for (String name : cameraNames) {
            Transform3d robotToCam = VisionHardware.kCameraOffsets.getOrDefault(
                name, VisionHardware.kDefaultRobotToCam);
            ReadAprilTag sensor = new ReadAprilTag(name, robotToCam);
            m_sensors.add(sensor);
            // Cap frame rate on each camera to reduce CPU load during vision fusion
            sensor.getCamera().setFPSLimit(20);
        }

        SmartDashboard.putData("Vision/Field Map",     m_field);
        SmartDashboard.putData("Vision/Field Cameras", m_fieldCameras);

        // Pre-register Field2d objects so Elastic sees them before any command runs
        m_field.getObject("Vision Ghost") .setPose(new Pose2d(-10, -10, new Rotation2d()));
        m_field.getObject("Target Pose")  .setPose(new Pose2d(-10, -10, new Rotation2d()));
        m_field.getObject("hub-line-ready")   .setPoses(
            new Pose2d(-10, -10, new Rotation2d()), new Pose2d(-10, -10, new Rotation2d()));
        m_field.getObject("hub-line-notready").setPoses(
            new Pose2d(-10, -10, new Rotation2d()), new Pose2d(-10, -10, new Rotation2d()));
        SmartDashboard.putNumber("Vision/Tag Count",  0);
        SmartDashboard.putNumber("Vision/Latency ms", -1.0);
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Reset per-loop tracking
        m_bestTarget    = null;
        m_bestHubTarget = null;
        m_hubDistMeters = 0.0;

        // Set alliance origin exactly once after DS connects
        if (!m_allianceSet && DriverStation.getAlliance().isPresent()) {
            updateAllianceOrigin();
            m_allianceSet = true;
        }

        // Current fused pose — read once, reused for every camera this tick
        Pose2d robotPose = m_drivetrain.getState().Pose;
        // FPGA time used for freshness checks
        double now = RobotController.getFPGATime() * 1e-6; // µs → s

        for (ReadAprilTag sensor : m_sensors) {
            // Estimate called ONCE per sensor per loop
            var estimateOpt = sensor.getEstimatedGlobalPose(robotPose);

            if (estimateOpt.isPresent()) {
                EstimatedRobotPose estimate = estimateOpt.get();
                Pose2d             estPose  = estimate.estimatedPose.toPose2d();

                // Freshness guard: drop stale frames
                double ageSec = now - estimate.timestampSeconds;
                if (ageSec > VISION_STALENESS_THRESHOLD_S) {
                    m_fieldCameras.getObject("Ghost-" + sensor.getName())
                        .setPose(new Pose2d(-10, -10, new Rotation2d()));
                    continue;
                }

                // ── Adaptive trust scaling (6328 approach) ───────────────────
                // Base: less trust (higher stdDev) at greater distance.
                double dist   = sensor.getAverageDistanceToTags();
                double stdDev = 0.1 + (dist * dist * 0.05);

                // Speed penalty: fast motion blurs tag detections, so above
                // 3 m/s we scale stdDev proportionally — odometry carries more
                // weight and prevents the pose from "teleporting".
                var chassisSpeeds = m_drivetrain.getState().Speeds;
                double speed = Math.hypot(
                    chassisSpeeds.vxMetersPerSecond,
                    chassisSpeeds.vyMetersPerSecond);
                if (speed > 3.0) stdDev *= (speed / 3.0);

                // Ambiguity soft-penalty: instead of hard-rejecting ambiguous
                // frames (already done in ReadAprilTag for >0.2), we further
                // scale stdDev so high-ambiguity measurements contribute less.
                // Scale: 1.0 at ambiguity=0 → 2.0 at kAmbiguityThreshold (0.2).
                double ambiguity = sensor.getAmbiguity();
                stdDev *= (1.0 + (ambiguity / VisionConstants.kAmbiguityThreshold));

                // Multi-tag bonus: halve stdDev when ≥ 2 tags visible.
                var result = sensor.getLatestResult();
                if (result != null && result.getTargets().size() >= 2) {
                    stdDev *= 0.5;
                }

                m_drivetrain.addVisionMeasurement(
                    estPose,
                    estimate.timestampSeconds,
                    VecBuilder.fill(stdDev, stdDev, Units.degreesToRadians(30)));

                m_fieldCameras.getObject("Ghost-" + sensor.getName()).setPose(estPose);

                // Track the most recently accepted vision pose for the ghost overlay
                if (m_lastVisionPose == null || estimate.timestampSeconds > m_lastVisionTimeS) {
                    m_lastVisionPose  = estPose;
                    m_lastVisionTimeS = now;
                }
            } else {
                m_fieldCameras.getObject("Ghost-" + sensor.getName())
                    .setPose(new Pose2d(-10, -10, new Rotation2d()));
            }

            // Track best overall and best hub-specific targets
            var result = sensor.getLatestResult();
            if (result != null && result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (m_bestTarget == null || target.getArea() > m_bestTarget.getArea()) {
                        m_bestTarget = target;
                    }
                    if (isHubTag(target.getFiducialId())) {
                        if (m_bestHubTarget == null
                                || target.getArea() > m_bestHubTarget.getArea()) {
                            m_bestHubTarget = target;
                            m_lastHubResultTimestamp = sensor.getLatestResultTimestamp();
                        }
                    }
                }
            }
        }

        // Compute hub distance from best hub tag
        if (m_bestHubTarget != null) {
            m_hubDistMeters = PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.CAMERA_HEIGHT_METERS,
                ShooterConstants.HUB_TARGET_HEIGHT_METERS,
                VisionConstants.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(m_bestHubTarget.getPitch()));
        }

        // Update main field2d AFTER fusion so it shows the corrected pose
        m_field.setRobotPose(m_drivetrain.getState().Pose);

        // Live distance to hub — published every loop so Elastic always shows it
        double distToHub = m_drivetrain.getState().Pose.getTranslation()
            .getDistance(ShooterConstants.HUB_CENTER);
        SmartDashboard.putNumber("DTHS/CurrentDist_m", distToHub);
        SmartDashboard.putNumber("DTHS/DistError_m",   distToHub - ShotCalculator.OPTIMAL_STANDOFF_M);

        updateDashboard();
    }

    // ── Public accessors ──────────────────────────────────────────────────────

    /** @return true if any AprilTag is visible. */
    public boolean hasValidTarget()  { return m_bestTarget    != null; }
    /** @return true if a hub AprilTag is visible. */
    public boolean hasHubTarget()    { return m_bestHubTarget != null; }

    /** Best overall target (highest area). May be {@code null}. */
    public PhotonTrackedTarget getBestTarget()    { return m_bestTarget;    }
    /** Best hub-specific AprilTag target. May be {@code null}. */
    public PhotonTrackedTarget getBestHubTarget() { return m_bestHubTarget; }

    /**
     * FPGA timestamp (seconds) of the most recently seen hub tag result.
     * Compare against {@code RobotController.getFPGATime() * 1e-6} to determine
     * how stale the measurement is.
     */
    public double getLatestHubResultTimestamp() { return m_lastHubResultTimestamp; }

    /**
     * Camera-measured distance to the hub (metres).
     * For shooting decisions prefer the odometry-derived distance in
     * {@code DriveToHubAndShootCommand}, which is smoother.
     */
    public double getHubDistanceMeters() { return m_hubDistMeters; }

    /** Yaw to best hub tag from camera (degrees). 0.0 when none visible. */
    public double getHubTagYawDeg() {
        return m_bestHubTarget != null ? m_bestHubTarget.getYaw() : 0.0;
    }

    /**
     * Distance to best (any) target in <b>inches</b>.
     * Kept for AlignToTag compatibility.
     */
    public double getDistanceToTarget() {
        if (m_bestTarget == null) return 0.0;
        return Units.metersToInches(PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.CAMERA_HEIGHT_METERS,
            VisionConstants.TARGET_HEIGHT_METERS,
            VisionConstants.CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(m_bestTarget.getPitch())));
    }

    /**
     * PID rotation speed toward the best visible tag (rad/s), clamped to ±3.
     * Used by teleop yaw-alignment commands.
     */
    public double getAlignmentRotationSpeed() {
        if (m_bestTarget == null) return 0.0;
        return Math.max(-3.0, Math.min(3.0,
            m_strafePID.calculate(m_bestTarget.getYaw(), 0.0)));
    }

    /** Field2d showing the Kalman-fused robot pose. */
    public Field2d getField()        { return m_field;        }
    /** Field2d showing individual camera raw estimates. */
    public Field2d getFieldCameras() { return m_fieldCameras; }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private static boolean isHubTag(int id) {
        for (int hubId : ShooterConstants.HUB_APRIL_TAG_IDS) {
            if (id == hubId) return true;
        }
        return false;
    }

    private void updateAllianceOrigin() {
        DriverStation.getAlliance().ifPresent(alliance ->
            VisionConstants.kTagLayout.setOrigin(
                alliance == DriverStation.Alliance.Red
                    ? edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide
                    : edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide));
    }

    /**
     * 6328-style diagnostic dashboard update — called every 20 ms from {@link #periodic()}.
     *
     * <h3>Field2d objects on "Vision/Field Map"</h3>
     * <ul>
     *   <li><b>Robot</b>         — Kalman-fused pose (set by {@code m_field.setRobotPose()}).</li>
     *   <li><b>Vision Ghost</b>  — Most recent accepted PhotonVision estimate, hidden after
     *                              {@link #GHOST_FADE_S} seconds of no accepted measurement.</li>
     *   <li><b>Target Pose</b>   — Ghost at the physics-optimal shooting standoff on the
     *                              current robot-to-hub bearing. Shows where to drive.</li>
     *   <li><b>hub-line-ready</b>    — Robot→hub line drawn when all fire gates are green.</li>
     *   <li><b>hub-line-notready</b> — Robot→hub line drawn when gates are not all green.</li>
     * </ul>
     *
     * <h3>SmartDashboard keys</h3>
     * <pre>
     *   Vision/Tag Count   — total AprilTags visible across all cameras this tick
     *   Vision/Latency ms  — pipeline latency of the freshest frame (-1 if none)
     * </pre>
     */
    private void updateDashboard() {
        Pose2d       robotPose = m_drivetrain.getState().Pose;
        Translation2d hub      = ShooterConstants.HUB_CENTER;
        double        now      = RobotController.getFPGATime() * 1e-6;

        // ── 1. Vision Ghost — single aggregate, fades after GHOST_FADE_S ─────────
        if (m_lastVisionPose != null && (now - m_lastVisionTimeS) < GHOST_FADE_S) {
            m_field.getObject("Vision Ghost").setPose(m_lastVisionPose);
        } else {
            m_field.getObject("Vision Ghost").setPose(new Pose2d(-10, -10, new Rotation2d()));
        }

        // ── 2. Target Pose — ideal shooting standoff on current robot-to-hub axis ─
        // Bearing from hub → robot so the robot stands on the far side of the hub.
        double bearingRad = Math.atan2(
            robotPose.getY() - hub.getY(),
            robotPose.getX() - hub.getX());
        double targetX = hub.getX() + ShotCalculator.OPTIMAL_STANDOFF_M * Math.cos(bearingRad);
        double targetY = hub.getY() + ShotCalculator.OPTIMAL_STANDOFF_M * Math.sin(bearingRad);
        // Face toward hub (reverse of bearing)
        Rotation2d targetRot = new Rotation2d(bearingRad + Math.PI);
        m_field.getObject("Target Pose").setPose(new Pose2d(targetX, targetY, targetRot));

        // ── 3. Robot→hub trajectory line, color-split by fire-gate readiness ─────
        // Field2d has no native color API, so we use two named objects.
        // Only the active one gets real poses; the other is hidden off-field.
        Pose2d hubPose2d = new Pose2d(hub, new Rotation2d());
        boolean fireReady = SmartDashboard.getBoolean("DTHS2/Gate/FIRE", false);
        if (fireReady) {
            m_field.getObject("hub-line-ready")   .setPoses(robotPose, hubPose2d);
            m_field.getObject("hub-line-notready").setPoses(
                new Pose2d(-10, -10, new Rotation2d()),
                new Pose2d(-10, -10, new Rotation2d()));
        } else {
            m_field.getObject("hub-line-notready").setPoses(robotPose, hubPose2d);
            m_field.getObject("hub-line-ready")   .setPoses(
                new Pose2d(-10, -10, new Rotation2d()),
                new Pose2d(-10, -10, new Rotation2d()));
        }

        // ── 4. Vision metadata — aggregate across all cameras ────────────────────
        int    totalTags       = 0;
        double bestLatencyMs   = -1.0;
        for (ReadAprilTag sensor : m_sensors) {
            var result = sensor.getLatestResult();
            if (result != null && result.hasTargets()) {
                totalTags += result.getTargets().size();
                // Latency = time from image capture to now (ms)
                double latMs = (now - result.getTimestampSeconds()) * 1000.0;
                if (bestLatencyMs < 0 || latMs < bestLatencyMs) bestLatencyMs = latMs;
            }
        }
        SmartDashboard.putNumber("Vision/Tag Count",  totalTags);
        SmartDashboard.putNumber("Vision/Latency ms", bestLatencyMs);

        // ── 5. Standard target telemetry ─────────────────────────────────────────
        SmartDashboard.putBoolean("Vision/Has Target",     m_bestTarget    != null);
        SmartDashboard.putBoolean("Vision/Has Hub Target", m_bestHubTarget != null);

        if (m_bestTarget != null) {
            SmartDashboard.putNumber("Vision/Target ID",        m_bestTarget.getFiducialId());
            SmartDashboard.putNumber("Vision/Target Yaw (deg)", m_bestTarget.getYaw());
            SmartDashboard.putNumber("Vision/Distance (in)",    getDistanceToTarget());
        }
        if (m_bestHubTarget != null) {
            SmartDashboard.putNumber("Vision/Hub Tag ID",        m_bestHubTarget.getFiducialId());
            SmartDashboard.putNumber("Vision/Hub Tag Yaw (deg)", m_bestHubTarget.getYaw());
            SmartDashboard.putNumber("Vision/Hub Distance (m)",  m_hubDistMeters);
        }
    }
}