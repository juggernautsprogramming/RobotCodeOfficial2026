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
import edu.wpi.first.math.geometry.Rotation3d;
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
 *   rotStdDev   = 12°  when tagCount ≥ 2  (multi-tag: gentle heading nudge)
 *               = 30°  when tagCount = 1  (single-tag: weaker heading nudge)
 *   Gyro/odometry always dominates rotation; vision prevents long-term drift.
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
 *   Drive/RobotPose3d                  — Kalman-fused 3D pose (logged by CommandSwerveDrivetrain)
 *   Drive/VisionPose                   — latest accepted vision pose 2D (ghost robot)
 *   Vision/AcceptedPoses               — all Kalman-injected poses this loop (Pose2d[])
 *   Vision/AcceptedPoses3d             — same as AcceptedPoses as Pose3d[] for 3D field view
 *   Vision/DetectedTags3d              — 3D field poses of all currently visible tags
 *   Vision/LayoutTags3d                — all tag poses from the loaded layout (reference)
 *   Vision/VisionEnabled               — whether Kalman injection is active
 *   Vision/Rejected/Stale              — frames dropped for being too old
 *   Vision/Rejected/OutOfBounds        — frames dropped for impossible field position
 *   Vision/Rejected/Teleport           — frames dropped for large pose jump
 *   Vision/{camera}/XY_StdDev         — per-camera trust applied this loop
 *   Vision/{camera}/TagCount           — tags used in the accepted estimate
 *   Vision/{camera}/Distance_m         — average camera-to-tag distance
 *   Vision/{camera}/Ambiguity          — best-target pose ambiguity ratio
 *   Vision/{camera}/JumpDist_m         — translation delta vs current odometry
 *   Vision/{camera}/EstimatedPoses3d          — all 3D pose estimates from this camera this loop
 *   Vision/{camera}/CamToTagTransforms        — camera-to-tag Transform3d for each detected tag
 *   Vision/ConsensusPose                      — weighted average of odometry + all vision estimates (Pose2d)
 *   Vision/ConsensusPose3d                    — same as above as Pose3d for 3D field view
 *   Vision/SourceWeights/OdometryFraction     — fraction [0-1] of total weight from odometry
 *   Vision/SourceWeights/VisionFraction       — fraction [0-1] of total weight from vision
 *   Vision/SourceWeights/VisionEstimateCount  — number of vision estimates this loop
 * </pre>
 */
public class VisionSubsystem extends SubsystemBase {

    // ── Sanity filter ─────────────────────────────────────────────────────────

    /**
     * Field dimensions pulled from the loaded AprilTag layout (metres).
     * Automatically matches whatever field JSON is deployed (custom hub, full field, etc.)
     * so the out-of-bounds rejection always matches the actual playing field.
     */
    private static final double FIELD_LENGTH_M = VisionConstants.kTagLayout.getFieldLength();
    private static final double FIELD_WIDTH_M  = VisionConstants.kTagLayout.getFieldWidth();

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
     * Rotation std-dev for all vision pose estimates.
     * Set to a very large value so vision NEVER influences heading — gyro owns it.
     * Vision only corrects XY position.
     */
    private static final double ROT_STDDEV_RAD = 9999.0;

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

    /**
     * Most recent weighted consensus pose — a manual weighted average of odometry
     * and all accepted vision estimates this loop.  Logged as
     * {@code Vision/ConsensusPose} and {@code Vision/ConsensusPose3d}.
     * Null until the first loop completes.
     */
    private Pose2d m_consensusPose = null;

    /**
     * Std-dev used for the odometry source in the consensus XY calculation (metres).
     * Raised so cameras (XY_BASE_STDDEV ≈ 0.05 m) dominate the XY consensus pose
     * whenever at least one tag is visible.  Odometry still wins for rotation
     * through the Kalman filter's gyro model.
     */
    private static final double ODOMETRY_STDDEV_M = 0.25;

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
        List<Pose3d> detectedTags3d  = new ArrayList<>();
        List<Pose2d> acceptedPoses   = new ArrayList<>();
        List<Double> acceptedStdDevs = new ArrayList<>(); // parallel to acceptedPoses
        Set<Integer> seenTagIds      = new HashSet<>();
        int rejStale = 0, rejOutOfBounds = 0, rejTeleport = 0;

        // Reset per-loop target state
        m_bestTarget    = null;
        m_bestHubTarget = null;
        m_hubDistMeters = 0.0;

        Pose2d robotPose = m_drivetrain.getState().Pose;
        double now       = RobotController.getFPGATime() * 1e-6; // µs → s

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

                // ── Tag metrics (needed for jump handling AND trust model) ─────
                int    tagCount  = estimate.targetsUsed.size();
                double dist      = sensor.getAverageDistanceToTags();
                double ambiguity = sensor.getAmbiguity();

                // (c) Large pose jump handling
                // Normal Kalman updates can't bridge a multi-metre gap (e.g. robot
                // starts at 0,0 but the tags say it's at 2,3).  If the fix is
                // reasonable confidence (low ambiguity) we RESET the pose so
                // odometry immediately starts from the correct field position.
                // High-ambiguity large jumps are still rejected to guard against
                // bad single-tag solves corrupting the estimator.
                double jumpDist = estPose.getTranslation()
                    .getDistance(robotPose.getTranslation());
                if (jumpDist > MAX_POSE_JUMP_M) {
                    if (ambiguity < 0.3 && m_visionEnabled) {
                        m_drivetrain.resetPose(estPose);
                        robotPose = estPose; // update local ref so rest of loop uses new pose
                        Logger.recordOutput("Vision/PoseReset", true);
                    } else {
                        rejTeleport++;
                    }
                    continue; // don't also add as vision measurement after reset
                }
                Logger.recordOutput("Vision/PoseReset", false);

                // Base XY std-dev scales quadratically with range
                double xyStdDev = XY_BASE_STDDEV + dist * dist * XY_DIST_COEFF;

                // Multi-tag bonus: geometric constraint greatly reduces error
                if (tagCount >= 2) xyStdDev *= MULTI_TAG_FACTOR;

                // Ambiguity penalty: linearly scale up std-dev for higher ambiguity
                xyStdDev *= (1.0 + ambiguity * (AMBIGUITY_SCALE - 1.0));

                // Rotation: gyro owns heading. Vision only corrects XY.
                double rotStdDev = ROT_STDDEV_RAD;

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

                // Track for ghost pose display, consensus calc, and AdvantageScope
                acceptedPoses.add(estPose);
                acceptedStdDevs.add(xyStdDev);
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

            // Per-camera 3D estimated poses — for AdvantageScope 3D field view.
            // Shows exactly what each camera thinks the robot's pose is.
            Logger.recordOutput(
                "Vision/" + sensor.getName() + "/EstimatedPoses3d",
                sensor.getNewEstimatedPoses3d());

            // Camera-to-tag transforms — visualize tag detections in 3D.
            Logger.recordOutput(
                "Vision/" + sensor.getName() + "/CamToTagTransforms",
                sensor.getCameraToTagTransforms());

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

        // ── Multi-source weighted consensus pose ──────────────────────────────
        // Combines odometry + every accepted vision estimate into a single
        // manually-averaged pose.  Weight = 1/σ² so lower std-dev = more pull.
        //
        // This is NOT fed to the Kalman filter — it is a transparency/debug signal
        // so you can see in AdvantageScope how much each source is contributing vs
        // what the Kalman filter (Drive/RobotPose3d) actually decided.
        {
            double sumW    = 0.0;
            double sumWx   = 0.0, sumWy   = 0.0;
            double sumWSin = 0.0, sumWCos = 0.0;

            // Odometry contribution
            double odomW = 1.0 / (ODOMETRY_STDDEV_M * ODOMETRY_STDDEV_M);
            sumW    += odomW;
            sumWx   += odomW * robotPose.getX();
            sumWy   += odomW * robotPose.getY();
            sumWSin += odomW * Math.sin(robotPose.getRotation().getRadians());
            sumWCos += odomW * Math.cos(robotPose.getRotation().getRadians());

            // Vision estimates contribution
            double visionTotalW = 0.0;
            for (int i = 0; i < acceptedPoses.size(); i++) {
                double w = 1.0 / (acceptedStdDevs.get(i) * acceptedStdDevs.get(i));
                Pose2d p = acceptedPoses.get(i);
                sumW    += w;
                sumWx   += w * p.getX();
                sumWy   += w * p.getY();
                sumWSin += w * Math.sin(p.getRotation().getRadians());
                sumWCos += w * Math.cos(p.getRotation().getRadians());
                visionTotalW += w;
            }

            // Compute the weighted average
            double avgX     = sumWx / sumW;
            double avgY     = sumWy / sumW;
            double avgTheta = Math.atan2(sumWSin / sumW, sumWCos / sumW);
            m_consensusPose = new Pose2d(avgX, avgY, new Rotation2d(avgTheta));

            Logger.recordOutput("Vision/ConsensusPose",   m_consensusPose);
            Logger.recordOutput("Vision/ConsensusPose3d", new Pose3d(m_consensusPose));

            // Source-weight fractions [0-1] — tells you how much each source is
            // pulling the consensus.  OdometryFraction + VisionFraction = 1.0.
            Logger.recordOutput("Vision/SourceWeights/OdometryFraction",
                sumW > 0 ? odomW / sumW : 1.0);
            Logger.recordOutput("Vision/SourceWeights/VisionFraction",
                sumW > 0 ? visionTotalW / sumW : 0.0);
            Logger.recordOutput("Vision/SourceWeights/VisionEstimateCount",
                (double) acceptedPoses.size());
        }

        // ── AdvantageScope bulk logging ───────────────────────────────────────

        // NOTE: Drive/RobotPose3d is logged by CommandSwerveDrivetrain — not duplicated here.

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

        // All vision poses accepted into the Kalman filter this loop — 2D
        Logger.recordOutput("Vision/AcceptedPoses",
            acceptedPoses.toArray(new Pose2d[0]));

        // Same accepted poses as Pose3d[] for the AdvantageScope 3D field view.
        // Lets you see ghost robots at each raw camera estimate vs the fused pose.
        Pose3d[] acceptedPoses3d = new Pose3d[acceptedPoses.size()];
        for (int i = 0; i < acceptedPoses.size(); i++) {
            acceptedPoses3d[i] = new Pose3d(acceptedPoses.get(i).getX(),
                acceptedPoses.get(i).getY(), 0.0,
                new Rotation3d(0, 0, acceptedPoses.get(i).getRotation().getRadians()));
        }
        Logger.recordOutput("Vision/AcceptedPoses3d", acceptedPoses3d);

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

    /**
     * Most recent weighted consensus pose — a manual weighted average of odometry
     * and all accepted vision estimates from the last periodic() loop.
     * Returns {@code null} before the first loop completes.
     * Logged as {@code Vision/ConsensusPose} and {@code Vision/ConsensusPose3d}.
     */
    public Pose2d getConsensusPose() { return m_consensusPose; }

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
        // Custom hub layout uses absolute coordinates — do NOT flip by alliance.
        // Alliance origin flipping is only correct for the full FRC field layout.
        VisionConstants.kTagLayout.setOrigin(
            edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

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
