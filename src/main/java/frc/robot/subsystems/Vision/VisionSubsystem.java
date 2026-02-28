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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionHardware;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * VisionSubsystem — PhotonVision multi-camera pose fusion for FRC 2026.
 *
 * <h3>Design notes</h3>
 * <ul>
 *   <li>{@code getEstimatedGlobalPose()} is called exactly <em>once</em> per
 *       sensor per loop and the result is reused for both fusion and telemetry,
 *       eliminating redundant CPU work from the previous double-call pattern.</li>
 *   <li>Hub-specific AprilTags are tracked separately so
 *       {@link DriveToHubAndShootCommand} can query hub distance/yaw directly.</li>
 *   <li>Vision fusion uses a distance-squared std-dev model: the Kalman filter
 *       trusts the camera more when it is close to the target.</li>
 *   <li>Drive outputs in {@code DriveToHubAndShootCommand} are computed from the
 *       <em>fused odometry pose</em> — never from raw camera angles — so this
 *       subsystem's periodic() never causes jitter in drive commands.</li>
 * </ul>
 */
public class VisionSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final List<ReadAprilTag>      m_sensors = new ArrayList<>();

    /** Full-field2d showing the Kalman-fused robot pose. */
    private final Field2d m_field        = new Field2d();
    /** Full-field2d showing each camera's raw pose estimate ("ghost"). */
    private final Field2d m_fieldCameras = new Field2d();

    /** Best target across all cameras (highest area = most confident). */
    private PhotonTrackedTarget m_bestTarget    = null;
    /** Best hub-specific AprilTag target. */
    private PhotonTrackedTarget m_bestHubTarget = null;
    /** Distance to hub derived from the best hub tag (meters). */
    private double m_hubDistMeters = 0.0;

    private boolean m_allianceSet = false;

    /** PID for raw yaw-alignment rotation commands (AlignToTag / teleop use). */
    private final PIDController m_strafePID;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * @param cameraNames Names that exactly match what is configured in the
     *                    PhotonVision web UI. Use the constants in
     *                    {@link VisionHardware} — e.g.
     *                    {@code new String[]{VisionHardware.CAMERA_BACK_LEFT,
     *                                       VisionHardware.CAMERA_BACK_RIGHT}}.
     * @param drivetrain  The swerve drivetrain whose pose estimator will be fed.
     */
    public VisionSubsystem(String[] cameraNames, CommandSwerveDrivetrain drivetrain) {
        this.m_drivetrain = drivetrain;
        this.m_strafePID  = new PIDController(
            VisionConstants.kP, VisionConstants.kI, VisionConstants.kD);
        this.m_strafePID.setTolerance(VisionConstants.angleTolerance);

        for (String name : cameraNames) {
            Transform3d robotToCam = VisionHardware.kCameraOffsets.getOrDefault(
                name, VisionHardware.kDefaultRobotToCam);
            ReadAprilTag sensor = new ReadAprilTag(name, robotToCam);
            m_sensors.add(sensor);
            sensor.getCamera().setFPSLimit(20);
        }

        SmartDashboard.putData("Vision/Field Map",     m_field);
        SmartDashboard.putData("Vision/Field Cameras", m_fieldCameras);
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Reset per-loop tracking
        m_bestTarget    = null;
        m_bestHubTarget = null;
        m_hubDistMeters = 0.0;

        // Set alliance origin once DS is connected
        if (!m_allianceSet && DriverStation.getAlliance().isPresent()) {
            updateAllianceOrigin();
            m_allianceSet = true;
        }

        // Get the current fused pose once — used for both estimation AND ghost telemetry
        Pose2d robotPose = m_drivetrain.getState().Pose;

        for (ReadAprilTag sensor : m_sensors) {
            // Call getEstimatedGlobalPose ONCE and cache the result
            var estimateOpt = sensor.getEstimatedGlobalPose(robotPose);

            if (estimateOpt.isPresent()) {
                EstimatedRobotPose estimate = estimateOpt.get();
                Pose2d             estPose  = estimate.estimatedPose.toPose2d();

                // Distance-squared std-dev: less trust at greater distance
                double dist   = sensor.getAverageDistanceToTags();
                double stdDev = 0.1 + (dist * dist * 0.05);

                m_drivetrain.addVisionMeasurement(
                    estPose,
                    estimate.timestampSeconds,
                    VecBuilder.fill(stdDev, stdDev, Units.degreesToRadians(30)));

                // Update this camera's ghost pose on the Field2d
                m_fieldCameras.getObject("Ghost-" + sensor.getName()).setPose(estPose);
            } else {
                // Push ghost off-field when no estimate available
                m_fieldCameras.getObject("Ghost-" + sensor.getName())
                    .setPose(new Pose2d(-10, -10, new Rotation2d()));
            }

            // Track best overall and best hub-specific targets
            var result = sensor.getLatestResult();
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    // Best overall target
                    if (m_bestTarget == null || target.getArea() > m_bestTarget.getArea()) {
                        m_bestTarget = target;
                    }
                    // Best hub target
                    if (isHubTag(target.getFiducialId())) {
                        if (m_bestHubTarget == null || target.getArea() > m_bestHubTarget.getArea()) {
                            m_bestHubTarget = target;
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

        // Update main field2d with the latest fused pose (AFTER vision fusion above)
        m_field.setRobotPose(m_drivetrain.getState().Pose);
        updateUI();
    }

    // ── Public accessors ──────────────────────────────────────────────────────

    /** True if any AprilTag is currently visible. */
    public boolean hasValidTarget()  { return m_bestTarget    != null; }
    /** True if a hub AprilTag is currently visible. */
    public boolean hasHubTarget()    { return m_bestHubTarget != null; }

    /** Best overall target (highest pixel area). May be {@code null}. */
    public PhotonTrackedTarget getBestTarget()    { return m_bestTarget;    }
    /** Best hub AprilTag target. May be {@code null}. */
    public PhotonTrackedTarget getBestHubTarget() { return m_bestHubTarget; }

    /**
     * Camera-measured distance to the hub (meters). Returns 0.0 when no hub
     * tag is visible.
     *
     * <p><b>Note:</b> for shooting and alignment decisions prefer the
     * odometry-derived distance in {@code DriveToHubAndShootCommand}, which is
     * smoother and jitter-free.
     */
    public double getHubDistanceMeters() { return m_hubDistMeters; }

    /**
     * Yaw to the best hub tag as seen by the camera (degrees).
     * Positive = tag is to the left of camera center. Returns 0.0 when none visible.
     */
    public double getHubTagYawDeg() {
        return m_bestHubTarget != null ? m_bestHubTarget.getYaw() : 0.0;
    }

    /**
     * Distance to the best (any) target in <em>inches</em>.
     * Kept for compatibility with existing commands such as AlignToTag.
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
     * PID-based rotation speed toward the best visible tag (rad/s), clamped to ±3.
     * Used by teleop yaw-alignment commands.
     */
    public double getAlignmentRotationSpeed() {
        if (m_bestTarget == null) return 0.0;
        return Math.max(-3.0, Math.min(3.0,
            m_strafePID.calculate(m_bestTarget.getYaw(), 0.0)));
    }

    /** Field2d showing the Kalman-fused robot pose (drag into Elastic). */
    public Field2d getField()        { return m_field;        }
    /** Field2d showing individual camera raw pose estimates. */
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

    private void updateUI() {
        SmartDashboard.putBoolean("Vision/Has Target",     m_bestTarget    != null);
        SmartDashboard.putBoolean("Vision/Has Hub Target", m_bestHubTarget != null);

        if (m_bestTarget != null) {
            SmartDashboard.putNumber("Vision/Target ID",       m_bestTarget.getFiducialId());
            SmartDashboard.putNumber("Vision/Target Yaw (deg)", m_bestTarget.getYaw());
            SmartDashboard.putNumber("Vision/Distance (in)",   getDistanceToTarget());
        }
        if (m_bestHubTarget != null) {
            SmartDashboard.putNumber("Vision/Hub Tag ID",         m_bestHubTarget.getFiducialId());
            SmartDashboard.putNumber("Vision/Hub Tag Yaw (deg)",  m_bestHubTarget.getYaw());
            SmartDashboard.putNumber("Vision/Hub Distance (m)",   m_hubDistMeters);
        }
    }
}