package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionHardware;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * VisionSubsystem — multi-camera PhotonVision → Kalman filter pose fusion.
 *
 * <h3>Architecture</h3>
 * <ul>
 *   <li>Each physical camera is wrapped in a {@link ReadAprilTag} instance.
 *       {@code ReadAprilTag} is a plain (non-Subsystem) class; this subsystem
 *       drives its update loop explicitly to guarantee correct execution order.</li>
 *   <li>Every camera frame (not just the latest) is fed to the drivetrain's
 *       {@code SwerveDrivePoseEstimator} using the frame's own FPGA timestamp,
 *       enabling full latency compensation.</li>
 * </ul>
 *
 * <h3>Measurement trust (std-dev) model</h3>
 * <pre>
 *   xyBase      = XY_BASE + dist² × XY_DIST_COEFF   (quadratic distance scaling)
 *   xyFinal     = xyBase × tagFactor × ambiguityFactor
 *
 *   tagFactor   = MULTI_TAG_FACTOR (< 1)  if tagCount ≥ 2, else 1.0
 *   ambiguity   = 1.0 + ambiguity × (AMBIGUITY_SCALE − 1)   range [1, AMBIGUITY_SCALE]
 *
 *   rotStdDev   = 5°   when tagCount ≥ 2 AND robot speed < SPEED_THRESHOLD
 *               = 9999 in all other cases (gyro is trusted instead)
 * </pre>
 *
 * <h3>Sanity filter</h3>
 * Each measurement is silently dropped when ANY of the following is true:
 * <ul>
 *   <li>Frame timestamp is older than {@code VISION_STALENESS_S}.</li>
 *   <li>Estimated pose is outside the field + {@code FIELD_MARGIN_M} border.</li>
 *   <li>Pose translation jumped more than {@code MAX_POSE_JUMP_M} from current odometry.</li>
 * </ul>
 *
 * <h3>AdvantageScope keys</h3>
 * <pre>
 *   Drive/RobotPose3d           — fused 3-D robot pose (for 3D field view)
 *   Drive/VisionPose            — latest accepted vision pose (ghost robot)
 *   Vision/DetectedTags3d       — 3D field poses of all currently visible tags
 *   Vision/LayoutTags3d         — all tag poses from the loaded layout (reference)
 *   Vision/AcceptedPoses        — all poses injected into Kalman this loop
 *   Vision/VisionEnabled        — whether Kalman injection is active
 *   Vision/Rejected/Stale       — frames dropped for being too old
 *   Vision/Rejected/OutOfBounds — frames dropped for impossible field position
 *   Vision/Rejected/Teleport    — frames dropped for large pose jump
 *   Vision/{camera}/XY_StdDev  — per-camera trust applied this loop
 *   Vision/{camera}/TagCount    — tags used in the accepted estimate
 *   Vision/{camera}/Distance_m  — average camera-to-tag distance
 *   Vision/{camera}/Ambiguity   — best-target pose ambiguity ratio
 *   Vision/{camera}/JumpDist_m  — translation delta vs current odometry
 * </pre>
 */
public class VisionSubsystem extends SubsystemBase {

    // ── Sanity filter ─────────────────────────────────────────────────────────

    /** 2026 Reefscape field dimensions (metres). */
    private static final double FIELD_LENGTH_M = 17.548;
    private static final double FIELD_WIDTH_M  =  8.052;

    /**
     * Tolerance outside the field boundary where a pose is still accepted (metres).
     * Accounts for robot overhanging the edge and minor layout inaccuracies.
     */
    private static final double FIELD_MARGIN_M = 0.5;

    /**
     * Maximum XY distance between the current odometry pose and the incoming
     * vision estimate. Measurements beyond this are rejected as "teleportation."
     */
    private static final double MAX_POSE_JUMP_M = 1.0;

    /** Discard measurements older than this many seconds (stale-frame guard). */
    private static final double VISION_STALENESS_S = 0.150;

    // ── Std-dev trust model ───────────────────────────────────────────────────

    /** Minimum XY std-dev (metres) at zero distance, zero ambiguity, multi-tag. */
    private static final double XY_BASE_STDDEV   = 0.05;

    /** Quadratic distance coefficient (m std-dev per m² of range). */
    private static final double XY_DIST_COEFF    = 0.04;

    /**
     * XY std-dev multiplier when ≥ 2 tags are visible.
     * Values below 1.0 increase trust. 0.4 ≈ 2.5× more weight for multi-tag solves.
     */
    private static final double MULTI_TAG_FACTOR = 0.4;

    /**
     * Maximum ambiguity-based std-dev multiplier.
     * At ambiguity = 0: factor = 1.0 (no penalty).
     * At ambiguity = 1: factor = AMBIGUITY_SCALE (maximum penalty).
     */
    private static final double AMBIGUITY_SCALE  = 4.0;

    /**
     * Rotation std-dev (radians) used when vision heading is trusted.
     * Only applies to multi-tag + slow-robot scenarios.
     */
    private static final double ROT_STDDEV_TRUSTED_RAD = Units.degreesToRadians(5.0);

    /**
     * Rotation std-dev sentinel that effectively disables heading fusion.
     * The gyro is almost always more accurate for heading.
     */
    private static final double ROT_STDDEV_IGNORE_RAD  = 9999.0;

    /**
     * Robot translational speed below which vision rotation is trusted (m/s).
     */
    private static final double SPEED_TRUST_THRESHOLD_MPS = 0.3;

    // ── Logging ───────────────────────────────────────────────────────────────

    /** Hide the AdvantageScope ghost pose after no accepted estimate for this long. */
    private static final double GHOST_FADE_S = 0.5;

    // ── State ─────────────────────────────────────────────────────────────────

    private final CommandSwerveDrivetrain m_drivetrain;
    private final List<ReadAprilTag>      m_sensors = new ArrayList<>();

    /** Kalman-fused robot pose for dashboard visualisation. */
    private final Field2d m_field        = new Field2d();
    /** Per-camera raw pose ghosts for debugging. */
    private final Field2d m_fieldCameras = new Field2d();

    /** Best overall AprilTag target (highest pixel area) across all cameras. */
    private PhotonTrackedTarget m_bestTarget    = null;
    /** Best hub-specific AprilTag target. */
    private PhotonTrackedTarget m_bestHubTarget = null;
    /** Camera-derived distance to hub (metres). */
    private double m_hubDistMeters          = 0.0;
    /** FPGA timestamp of the most recently accepted hub measurement. */
    private double m_lastHubResultTimestamp = 0.0;

    private boolean  m_allianceSet    = false;
    private Pose2d   m_lastVisionPose = null;
    private double   m_lastVisionTimeS = 0.0;

    /**
     * Cached 3-D poses of all tags in the loaded layout.
     * Populated once after DS alliance is known (tag layout origin may flip).
     * Logged every loop so late-joining AdvantageScope instances see the data.
     */
    private Pose3d[] m_layoutTags3d = new Pose3d[0];

    /**
     * When false, vision measurements are NOT injected into the Kalman filter.
     * Set to false by DriveToPose so the robot navigates on pure odometry during
     * a snap-to-target maneuver, eliminating mid-motion pose jumps / jitter.
     * Always reset to true when DriveToPose ends.
     */
    private volatile boolean m_visionEnabled = true;

    /** PID for raw yaw-alignment commands (used by AlignToTag). */
    private final PIDController m_alignPID;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * @param cameraNames Camera names matching the PhotonVision web UI (use
     *                    {@link VisionHardware} constants).
     * @param drivetrain  Swerve drivetrain whose SwerveDrivePoseEstimator is fed.
     */
    public VisionSubsystem(String[] cameraNames, CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;

        m_alignPID = new PIDController(
            VisionConstants.kP, VisionConstants.kI, VisionConstants.kD);
        m_alignPID.setTolerance(VisionConstants.angleTolerance);

        for (String name : cameraNames) {
            Transform3d robotToCam = VisionHardware.kCameraOffsets.getOrDefault(
                name, VisionHardware.kDefaultRobotToCam);
            m_sensors.add(new ReadAprilTag(name, robotToCam));
        }

        SmartDashboard.putData("Vision/Field Map",     m_field);
        SmartDashboard.putData("Vision/Field Cameras", m_fieldCameras);
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {

        // Set field-relative AprilTag layout origin once after DS connects
        if (!m_allianceSet && DriverStation.getAlliance().isPresent()) {
            updateAllianceOrigin();
            m_allianceSet = true;
        }

        // ── Per-loop AdvantageScope accumulators ──────────────────────────────
        List<Pose3d> detectedTags3d = new ArrayList<>();
        List<Pose2d> acceptedPoses  = new ArrayList<>();
        Set<Integer> seenTagIds     = new HashSet<>();
        int rejStale = 0, rejOutOfBounds = 0, rejTeleport = 0;

        // Reset per-loop target state
        m_bestTarget    = null;
        m_bestHubTarget = null;
        m_hubDistMeters = 0.0;

        Pose2d robotPose = m_drivetrain.getState().Pose;
        double now       = RobotController.getFPGATime() * 1e-6; // µs → s

        var    chassisSpeeds = m_drivetrain.getState().Speeds;
        double speed = Math.hypot(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond);

        for (ReadAprilTag sensor : m_sensors) {

            // Step 1 — fetch all new frames from this camera (clears the camera buffer)
            sensor.update();

            // Step 2 — process every new estimate; each carries its own timestamp
            for (EstimatedRobotPose estimate : sensor.getNewEstimates()) {

                Pose2d estPose   = estimate.estimatedPose.toPose2d();
                double timestamp = estimate.timestampSeconds;

                // ── Sanity Filter ─────────────────────────────────────────────

                // (a) Staleness: drop frames that arrived too late to be useful
                if ((now - timestamp) > VISION_STALENESS_S) { rejStale++; continue; }

                // (b) Field boundaries: robot cannot be outside the field
                if (!isWithinField(estPose)) { rejOutOfBounds++; continue; }

                // (c) Teleportation: reject if pose jumped too far from odometry
                double jumpDist = estPose.getTranslation()
                    .getDistance(robotPose.getTranslation());
                if (jumpDist > MAX_POSE_JUMP_M) { rejTeleport++; continue; }

                // ── Dynamic Trust Factor ──────────────────────────────────────

                int    tagCount  = estimate.targetsUsed.size();
                double dist      = sensor.getAverageDistanceToTags();
                double ambiguity = sensor.getAmbiguity();

                // Base XY std-dev scales quadratically with range
                double xyStdDev = XY_BASE_STDDEV + dist * dist * XY_DIST_COEFF;

                // Multi-tag bonus: geometric constraint greatly reduces error
                if (tagCount >= 2) xyStdDev *= MULTI_TAG_FACTOR;

                // Ambiguity penalty: linearly scale up std-dev for higher ambiguity
                xyStdDev *= (1.0 + ambiguity * (AMBIGUITY_SCALE - 1.0));

                // Rotation: only trust heading for multi-tag + near-stationary robot
                double rotStdDev = (tagCount >= 2 && speed < SPEED_TRUST_THRESHOLD_MPS)
                    ? ROT_STDDEV_TRUSTED_RAD
                    : ROT_STDDEV_IGNORE_RAD;

                // ── Feed to Kalman filter at the historical timestamp ──────────
                // Skipped when vision is paused (e.g. DriveToPose is active) so
                // mid-motion Kalman corrections never jitter the drivetrain output.
                if (!m_visionEnabled) continue;

                // SwerveDrivePoseEstimator buffers past odometry states and
                // retroactively applies the vision correction at the correct time.
                m_drivetrain.addVisionMeasurement(
                    estPose,
                    timestamp,
                    VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev));

                // Track for ghost pose display and AdvantageScope
                acceptedPoses.add(estPose);
                if (m_lastVisionPose == null || timestamp > m_lastVisionTimeS) {
                    m_lastVisionPose  = estPose;
                    m_lastVisionTimeS = now;
                }

                m_fieldCameras.getObject("Ghost-" + sensor.getName()).setPose(estPose);

                // Log per-camera trust metrics for tuning
                String logPrefix = "Vision/" + sensor.getName() + "/";
                Logger.recordOutput(logPrefix + "XY_StdDev",  xyStdDev);
                Logger.recordOutput(logPrefix + "TagCount",   (double) tagCount);
                Logger.recordOutput(logPrefix + "Distance_m", dist);
                Logger.recordOutput(logPrefix + "Ambiguity",  ambiguity);
                Logger.recordOutput(logPrefix + "JumpDist_m", jumpDist);
            }

            // Step 3 — track best targets + collect detected tag 3D poses
            var result = sensor.getLatestResult();
            if (result != null && result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    // Best-target tracking
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

                    // AdvantageScope: collect 3D layout pose of each visible tag (deduplicated)
                    int tagId = target.getFiducialId();
                    if (seenTagIds.add(tagId)) {
                        VisionConstants.kTagLayout.getTagPose(tagId)
                            .ifPresent(detectedTags3d::add);
                    }
                }
            } else {
                // Move ghost off-field when camera has no targets
                m_fieldCameras.getObject("Ghost-" + sensor.getName())
                    .setPose(new Pose2d(-10, -10, new Rotation2d()));
            }
        }

        // ── Hub distance ──────────────────────────────────────────────────────
        if (m_bestHubTarget != null) {
            m_hubDistMeters = PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.CAMERA_HEIGHT_METERS,
                VisionConstants.HUB_TAG_HEIGHT_METERS,
                VisionConstants.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(m_bestHubTarget.getPitch()));
        }

        // ── AdvantageScope bulk logging ───────────────────────────────────────

        // Robot pose in 3D for the AdvantageScope 3D field view
        Logger.recordOutput("Drive/RobotPose3d", new Pose3d(m_drivetrain.getState().Pose));

        // Vision ghost: parks off-field when stale or absent
        if (m_lastVisionPose != null && (now - m_lastVisionTimeS) < GHOST_FADE_S) {
            Logger.recordOutput("Drive/VisionPose", m_lastVisionPose);
        } else {
            Logger.recordOutput("Drive/VisionPose", new Pose2d(-10, -10, new Rotation2d()));
        }

        // 3D tag poses currently detected (from layout — exact field positions)
        Logger.recordOutput("Vision/DetectedTags3d",
            detectedTags3d.toArray(new Pose3d[0]));

        // All tag poses from the loaded layout (reference overlay in 3D view)
        Logger.recordOutput("Vision/LayoutTags3d", m_layoutTags3d);

        // All vision poses accepted into the Kalman filter this loop
        Logger.recordOutput("Vision/AcceptedPoses",
            acceptedPoses.toArray(new Pose2d[0]));

        // Vision pause gate state (true = Kalman corrections active)
        Logger.recordOutput("Vision/VisionEnabled", m_visionEnabled);

        // Sanity filter rejection counters
        Logger.recordOutput("Vision/Rejected/Stale",       (double) rejStale);
        Logger.recordOutput("Vision/Rejected/OutOfBounds", (double) rejOutOfBounds);
        Logger.recordOutput("Vision/Rejected/Teleport",    (double) rejTeleport);

        // ── Dashboard ─────────────────────────────────────────────────────────
        m_field.setRobotPose(m_drivetrain.getState().Pose);
        updateUI();
    }

    // ── Public accessors ──────────────────────────────────────────────────────

    /**
     * Enable or disable vision injection into the Kalman filter.
     *
     * <p>Call {@code setVisionEnabled(false)} at the start of a snap-to-target
     * maneuver (e.g. DriveToPose.initialize()) so the robot navigates on pure
     * wheel odometry + gyro — no mid-motion pose jumps from the Kalman filter.
     * Always restore with {@code setVisionEnabled(true)} when the maneuver ends.
     */
    public void setVisionEnabled(boolean enabled) {
        m_visionEnabled = enabled;
    }

    /** @return {@code true} if any AprilTag is currently visible. */
    public boolean hasValidTarget()  { return m_bestTarget    != null; }

    /** @return {@code true} if a hub-specific AprilTag is currently visible. */
    public boolean hasHubTarget()    { return m_bestHubTarget != null; }

    /** Best overall target by pixel area across all cameras. May be {@code null}. */
    public PhotonTrackedTarget getBestTarget()    { return m_bestTarget;    }

    /** Best hub-specific AprilTag target. May be {@code null}. */
    public PhotonTrackedTarget getBestHubTarget() { return m_bestHubTarget; }

    /**
     * FPGA timestamp (seconds) of the most recently accepted hub-tag result.
     * Compare against {@code RobotController.getFPGATime() * 1e-6} to assess staleness.
     */
    public double getLatestHubResultTimestamp() { return m_lastHubResultTimestamp; }

    /**
     * Camera-derived distance to the hub (metres).
     * For shooting decisions, prefer the odometry distance from
     * {@code DriveToHubAndShootCommand}, which is smoother and lower-latency.
     */
    public double getHubDistanceMeters() { return m_hubDistMeters; }

    /** Yaw to the best hub tag (degrees, camera-relative). 0.0 when none visible. */
    public double getHubTagYawDeg() {
        return m_bestHubTarget != null ? m_bestHubTarget.getYaw() : 0.0;
    }

    /**
     * Distance to the best visible target (any tag) in <b>inches</b>.
     * Retained for {@code AlignToTag} compatibility.
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
     * PID-computed rotation speed toward the best visible tag (rad/s), clamped ±3.
     * Used by teleop yaw-alignment commands ({@code AlignToTag}).
     */
    public double getAlignmentRotationSpeed() {
        if (m_bestTarget == null) return 0.0;
        return Math.max(-3.0, Math.min(3.0,
            m_alignPID.calculate(m_bestTarget.getYaw(), 0.0)));
    }

    /** Field2d displaying the Kalman-fused robot pose. */
    public Field2d getField()        { return m_field;        }

    /** Field2d displaying individual camera raw pose ghosts. */
    public Field2d getFieldCameras() { return m_fieldCameras; }

    // ── Helpers ───────────────────────────────────────────────────────────────

    /**
     * Returns {@code true} if the pose lies within the field boundary plus the
     * configured margin. Rejects clearly impossible poses (e.g. (-5, -5)).
     */
    private static boolean isWithinField(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        return x >= -FIELD_MARGIN_M && x <= FIELD_LENGTH_M + FIELD_MARGIN_M
            && y >= -FIELD_MARGIN_M && y <= FIELD_WIDTH_M  + FIELD_MARGIN_M;
    }

    private static boolean isHubTag(int id) {
        for (int hubId : ShooterConstants.HUB_APRIL_TAG_IDS) {
            if (id == hubId) return true;
        }
        return false;
    }

    /** Called once after the DriverStation alliance is known. */
    private void updateAllianceOrigin() {
        DriverStation.getAlliance().ifPresent(alliance ->
            VisionConstants.kTagLayout.setOrigin(
                alliance == DriverStation.Alliance.Red
                    ? edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide
                    : edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide));

        // Cache 3D poses of all tags in the layout — logged every loop so late-joining
        // AdvantageScope instances always see the reference overlay.
        var tags = VisionConstants.kTagLayout.getTags();
        m_layoutTags3d = new Pose3d[tags.size()];
        for (int i = 0; i < tags.size(); i++) {
            m_layoutTags3d[i] = tags.get(i).pose;
        }
    }

    private void updateUI() {
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
