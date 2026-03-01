package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.VelocityCompensator;

/**
 * HubAlignController — closed-loop chassis output for hub alignment.
 *
 * <h3>Key design decisions</h3>
 * <ul>
 *   <li><b>Continuous heading input</b> — {@code enableContinuousInput(-180, 180)}
 *       ensures the PID always takes the shortest arc through zero, never spinning
 *       350° the wrong way.</li>
 *   <li><b>Pose-estimator heading, not raw camera yaw</b> — the Kalman-fused
 *       odometry runs at 250 Hz and is latency-compensated; camera yaw is 20–100 ms
 *       stale. Vision corrects the estimator asynchronously via
 *       {@code addVisionMeasurement()}.</li>
 *   <li><b>Velocity-compensated desired heading</b> — we aim at the
 *       <em>virtual</em> hub (look-ahead for ball ToF), not the raw hub bearing.</li>
 *   <li><b>Exponential moving-average output filter</b> — smooths drive commands
 *       to prevent wheel-slip on hard lateral corrections.</li>
 * </ul>
 */
public class HubAlignController {

    // ── Controllers ───────────────────────────────────────────────────────────
    private final PIDController m_rotPID;
    private final PIDController m_drivePID;

    // ── Low-pass filter state ─────────────────────────────────────────────────
    private double m_filteredVx  = 0.0;
    private double m_filteredVy  = 0.0;
    private double m_filteredRot = 0.0;

    // ── Cached telemetry value ────────────────────────────────────────────────
    private double m_desiredHeadingDeg = 0.0;

    /** Result: field-relative chassis speeds to apply this loop tick. */
    public record DriveOutput(double vxMps, double vyMps, double omegaRadS) {}

    // ── Constructor ───────────────────────────────────────────────────────────

    public HubAlignController() {
        // Rotation PID — CRITICAL: continuous input prevents wrong-way wrap
        m_rotPID = new PIDController(
            ShooterConstants.ROTATION_kP, 0.0, ShooterConstants.ROTATION_kD);
        m_rotPID.enableContinuousInput(-180.0, 180.0);
        m_rotPID.setTolerance(ShooterConstants.YAW_TOLERANCE_DEG);

        // Drive (range) PID — drives robot to desired standoff distance
        m_drivePID = new PIDController(
            ShooterConstants.DRIVE_kP, 0.0, ShooterConstants.DRIVE_kD);
        m_drivePID.setTolerance(ShooterConstants.DISTANCE_TOLERANCE_METERS);
    }

    // ── Main compute method ───────────────────────────────────────────────────

    /**
     * Compute drive output for one 20 ms loop tick.
     *
     * @param drivetrain      Live drivetrain for pose and velocity.
     * @param hubPose         Hub centre, field-relative (metres).
     * @param desiredStandoff Target distance from hub to robot centre (metres).
     * @return Field-relative chassis speeds to apply via SwerveRequest.
     */
    public DriveOutput compute(
            CommandSwerveDrivetrain drivetrain,
            Translation2d hubPose,
            double desiredStandoff) {

        Pose2d robotPose = drivetrain.getState().Pose;
        var    speeds    = drivetrain.getState().Speeds;

        // 1. Velocity-compensated desired heading (look-ahead) ─────────────────
        var comp = VelocityCompensator.getInstance().compensate(
            robotPose.getTranslation(), hubPose,
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond);

        m_desiredHeadingDeg = comp.aimHeadingDeg();
        double currentHeadingDeg = robotPose.getRotation().getDegrees();

        // 2. Rotation output ───────────────────────────────────────────────────
        double rawRot = m_rotPID.calculate(currentHeadingDeg, m_desiredHeadingDeg);
        rawRot = MathUtil.clamp(rawRot,
            -ShooterConstants.ROTATION_MAX_RAD_S,
             ShooterConstants.ROTATION_MAX_RAD_S);

        // 3. Range drive: project correction along bearing to hub ─────────────
        double bearingRad = Math.atan2(
            hubPose.getY() - robotPose.getY(),
            hubPose.getX() - robotPose.getX());

        double currentDist = robotPose.getTranslation().getDistance(hubPose);
        double rawDrive    = m_drivePID.calculate(currentDist, desiredStandoff);
        rawDrive = MathUtil.clamp(rawDrive,
            -ShooterConstants.DRIVE_MAX_SPEED_MPS,
             ShooterConstants.DRIVE_MAX_SPEED_MPS);

        double rawVx = rawDrive * Math.cos(bearingRad);
        double rawVy = rawDrive * Math.sin(bearingRad);

        // 4. Exponential moving-average filter ────────────────────────────────
        double alpha     = ShooterConstants.OUTPUT_FILTER_ALPHA;
        m_filteredVx  = alpha * rawVx  + (1.0 - alpha) * m_filteredVx;
        m_filteredVy  = alpha * rawVy  + (1.0 - alpha) * m_filteredVy;
        m_filteredRot = alpha * rawRot + (1.0 - alpha) * m_filteredRot;

        // 5. Deadband ──────────────────────────────────────────────────────────
        double vx    = deadband(m_filteredVx,  ShooterConstants.VELOCITY_DEADBAND);
        double vy    = deadband(m_filteredVy,  ShooterConstants.VELOCITY_DEADBAND);
        double omega = deadband(m_filteredRot, ShooterConstants.VELOCITY_DEADBAND);

        // 6. Telemetry ─────────────────────────────────────────────────────────
        SmartDashboard.putNumber ("HubAlign/DesiredHeading",  m_desiredHeadingDeg);
        SmartDashboard.putNumber ("HubAlign/CurrentHeading",  currentHeadingDeg);
        SmartDashboard.putNumber ("HubAlign/HeadingError",    m_rotPID.getError());
        SmartDashboard.putNumber ("HubAlign/DistanceError",   currentDist - desiredStandoff);
        SmartDashboard.putNumber ("HubAlign/VirtDist_m",      comp.virtualDistanceMeters());
        SmartDashboard.putNumber ("HubAlign/ToF_s",           comp.timeOfFlight());
        SmartDashboard.putBoolean("HubAlign/RotAtTarget",     m_rotPID.atSetpoint());
        SmartDashboard.putBoolean("HubAlign/DistAtTarget",    m_drivePID.atSetpoint());

        return new DriveOutput(vx, vy, omega);
    }

    // ── Accessors ─────────────────────────────────────────────────────────────

    /** @return Last computed desired heading (degrees). Used by {@link ReadyToFireCondition}. */
    public double getDesiredHeadingDeg() { return m_desiredHeadingDeg; }

    /** @return true when heading is within {@link ShooterConstants#YAW_TOLERANCE_DEG}. */
    public boolean isAligned() { return m_rotPID.atSetpoint(); }

    /** @return true when range is within {@link ShooterConstants#DISTANCE_TOLERANCE_METERS}. */
    public boolean isAtDistance() { return m_drivePID.atSetpoint(); }

    /** Reset filter state and PID integral. Call from command {@code initialize()}. */
    public void reset() {
        m_rotPID.reset();
        m_drivePID.reset();
        m_filteredVx  = 0.0;
        m_filteredVy  = 0.0;
        m_filteredRot = 0.0;
        m_desiredHeadingDeg = 0.0;
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private static double deadband(double value, double band) {
        return Math.abs(value) < band ? 0.0 : value;
    }
}