package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.VelocityCompensator;

/**
 * HubAlignController — closed-loop chassis output for hub alignment.
 *
 * <h3>Key design decisions</h3>
 * <ul>
 *   <li><b>ProfiledPIDController for rotation (2910 / Jack in the Bot approach)</b> —
 *       generates a smooth trapezoidal velocity profile toward the hub heading so the
 *       robot never jerks or saturates. Continuous input is enabled in radians so
 *       the shortest arc is always taken through ±π.</li>
 *   <li><b>Pose-estimator heading, not raw camera yaw</b> — Kalman-fused odometry
 *       runs at 250 Hz and is latency-compensated; vision corrects it asynchronously
 *       via {@code addVisionMeasurement()}.</li>
 *   <li><b>Velocity-compensated desired heading</b> — we aim at the <em>virtual</em>
 *       hub (look-ahead for ball ToF), not the raw hub bearing.</li>
 *   <li><b>Exponential moving-average output filter</b> — smooths drive commands to
 *       prevent wheel-slip on hard lateral corrections.</li>
 *   <li><b>Citrus (1678) alignment gate</b> — {@link #isAligned()} requires the
 *       true angular error (goal vs. measurement) to be &lt; 1.5°, not just the
 *       profile-tracking error reported by {@code atSetpoint()}.</li>
 * </ul>
 *
 * <h3>ProfiledPIDController gain note</h3>
 * Both measurement and setpoint are supplied in <em>radians</em>.  The {@code kP}
 * constant therefore has effective units of (rad/s) per radian.  With the default
 * {@link ShooterConstants#ROTATION_kP} = 5.5 a 10° (0.175 rad) tracking error
 * produces ≈ 0.96 rad/s of corrective output — enough to keep up with the
 * trapezoidal profile without oscillating.  Retune if the robot lags or oscillates.
 */
public class HubAlignController {

    // ── Controllers ───────────────────────────────────────────────────────────

    /**
     * 2910 approach: ProfiledPIDController generates a trapezoidal velocity
     * profile so the robot ramps smoothly to and from the target heading rather
     * than saturating immediately and overshooting.
     *
     * <p>Operates in <b>radians</b>. {@code enableContinuousInput(-π, π)} ensures
     * the shortest arc is always chosen — no 350° wrong-way spins.
     */
    private final ProfiledPIDController m_rotPID;

    /** Plain PID — drives robot to desired standoff distance. */
    private final PIDController m_drivePID;

    // ── Low-pass filter state ─────────────────────────────────────────────────
    private double m_filteredVx  = 0.0;
    private double m_filteredVy  = 0.0;
    private double m_filteredRot = 0.0;

    // ── Cached values ─────────────────────────────────────────────────────────
    private double  m_desiredHeadingDeg = 0.0;
    private double  m_lastHeadingDeg    = 0.0;   // actual robot heading from last tick
    private boolean m_hasComputed       = false;

    /** Result: field-relative chassis speeds to apply this loop tick. */
    public record DriveOutput(double vxMps, double vyMps, double omegaRadS) {}

    // ── Constructor ───────────────────────────────────────────────────────────

    public HubAlignController() {
        // Rotation — ProfiledPIDController in radians.
        // Constraints govern the shape of the motion profile; kP/kD tune tracking.
        m_rotPID = new ProfiledPIDController(
            ShooterConstants.ROTATION_kP,
            0.0,
            ShooterConstants.ROTATION_kD,
            new TrapezoidProfile.Constraints(
                ShooterConstants.ROTATION_MAX_RAD_S,      // max ω  (rad/s)
                ShooterConstants.ROTATION_MAX_ACCEL_RAD_S2 // max α  (rad/s²)
            ));
        // CRITICAL: shortest-arc across the ±π seam
        m_rotPID.enableContinuousInput(-Math.PI, Math.PI);

        // Drive (range) PID — drives robot to desired standoff distance
        m_drivePID = new PIDController(
            ShooterConstants.DRIVE_kP, 0.0, ShooterConstants.DRIVE_kD);
        m_drivePID.setTolerance(ShooterConstants.DISTANCE_TOLERANCE_METERS);
    }

    // ── Main compute method ───────────────────────────────────────────────────

    /**
     * Compute drive output for one 20 ms loop tick.
     *
     * @param drivetrain      Live drivetrain for pose + velocity.
     * @param hubPose         Hub centre, field-relative (metres).
     * @param desiredStandoff Target distance from hub to robot centre (metres).
     * @return Field-relative chassis speeds to apply via {@code SwerveRequest}.
     */
    public DriveOutput compute(
            CommandSwerveDrivetrain drivetrain,
            Translation2d           hubPose,
            double                  desiredStandoff) {

        Pose2d robotPose = drivetrain.getState().Pose;
        var    speeds    = drivetrain.getState().Speeds;

        // 1. Velocity-compensated desired heading (look-ahead for ball ToF) ─────
        var comp = VelocityCompensator.getInstance().compensate(
            robotPose.getTranslation(), hubPose,
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond);

        m_desiredHeadingDeg = comp.aimHeadingDeg();
        double currentHeadingDeg = robotPose.getRotation().getDegrees();
        m_lastHeadingDeg         = currentHeadingDeg;

        // 2. Seed the profiled PID with the actual heading on the first call ─────
        // This prevents the profile from starting at position 0 and suddenly
        // generating full-speed rotation commands on the first tick.
        double currentHeadingRad = Math.toRadians(currentHeadingDeg);
        double desiredHeadingRad = Math.toRadians(m_desiredHeadingDeg);

        if (!m_hasComputed) {
            m_rotPID.reset(currentHeadingRad);
        }
        m_hasComputed = true;

        // 3. Profiled rotation PID output (rad/s) ─────────────────────────────
        double rawRot = m_rotPID.calculate(currentHeadingRad, desiredHeadingRad);
        rawRot = MathUtil.clamp(rawRot,
            -ShooterConstants.ROTATION_MAX_RAD_S,
             ShooterConstants.ROTATION_MAX_RAD_S);

        // 4. Range drive: project correction along bearing to hub ─────────────
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

        // 5. Exponential moving-average (EMA) output filter ───────────────────
        double alpha  = ShooterConstants.OUTPUT_FILTER_ALPHA;
        m_filteredVx  = alpha * rawVx  + (1.0 - alpha) * m_filteredVx;
        m_filteredVy  = alpha * rawVy  + (1.0 - alpha) * m_filteredVy;
        m_filteredRot = alpha * rawRot + (1.0 - alpha) * m_filteredRot;

        // 6. Deadband ──────────────────────────────────────────────────────────
        double vx    = deadband(m_filteredVx,  ShooterConstants.VELOCITY_DEADBAND);
        double vy    = deadband(m_filteredVy,  ShooterConstants.VELOCITY_DEADBAND);
        double omega = deadband(m_filteredRot, ShooterConstants.VELOCITY_DEADBAND);

        // 7. Telemetry ─────────────────────────────────────────────────────────
        double headingErrorDeg = ((currentHeadingDeg - m_desiredHeadingDeg + 180.0) % 360.0) - 180.0;
        SmartDashboard.putNumber ("HubAlign/DesiredHeading",  m_desiredHeadingDeg);
        SmartDashboard.putNumber ("HubAlign/CurrentHeading",  currentHeadingDeg);
        SmartDashboard.putNumber ("HubAlign/HeadingError",    headingErrorDeg);
        SmartDashboard.putNumber ("HubAlign/DistanceError",   currentDist - desiredStandoff);
        SmartDashboard.putNumber ("HubAlign/VirtDist_m",      comp.virtualDistanceMeters());
        SmartDashboard.putNumber ("HubAlign/ToF_s",           comp.timeOfFlight());
        SmartDashboard.putNumber ("HubAlign/RawVx",           rawVx);
        SmartDashboard.putNumber ("HubAlign/RawVy",           rawVy);
        SmartDashboard.putNumber ("HubAlign/FilteredVx",      m_filteredVx);
        SmartDashboard.putNumber ("HubAlign/FilteredVy",      m_filteredVy);
        SmartDashboard.putNumber ("HubAlign/ProfileOmega",    m_rotPID.getSetpoint().velocity);
        SmartDashboard.putBoolean("HubAlign/RotAligned",      isAligned());
        SmartDashboard.putBoolean("HubAlign/DistAtTarget",    m_drivePID.atSetpoint());

        return new DriveOutput(vx, vy, omega);
    }

    // ── Accessors ─────────────────────────────────────────────────────────────

    /** @return Last computed desired heading (degrees). Used by {@link ReadyToFireCondition}. */
    public double getDesiredHeadingDeg() { return m_desiredHeadingDeg; }

    /**
     * Citrus (1678) approach: returns {@code true} only when the true angular
     * error between the robot heading and the desired heading is &lt; 1.5°.
     *
     * <p>This is stricter than {@code atSetpoint()} on the profiled PID, which
     * checks profile-tracking error, not the goal error directly.
     * Returns {@code false} before the first {@link #compute} call.
     */
    public boolean isAligned() {
        if (!m_hasComputed) return false;
        // Wrap to [-180, 180] so the comparison is always in the shortest-arc space
        double err = ((m_lastHeadingDeg - m_desiredHeadingDeg + 180.0) % 360.0) - 180.0;
        return Math.abs(err) < 1.5;
    }

    /**
     * @return {@code true} when range is within
     *         {@link ShooterConstants#DISTANCE_TOLERANCE_METERS}.
     *         Returns {@code false} before the first {@link #compute} call.
     */
    public boolean isAtDistance() {
        return m_hasComputed && m_drivePID.atSetpoint();
    }

    /**
     * Reset filter state, PID integrals, and the "has computed" flag.
     * Must be called from the parent command's {@code initialize()}.
     *
     * <p>The profiled PID is reset to position 0; it will be re-seeded with
     * the actual heading on the first {@link #compute} call.
     */
    public void reset() {
        m_rotPID.reset(0.0);   // re-seeded with true heading on first compute()
        m_drivePID.reset();
        m_filteredVx        = 0.0;
        m_filteredVy        = 0.0;
        m_filteredRot       = 0.0;
        m_desiredHeadingDeg = 0.0;
        m_lastHeadingDeg    = 0.0;
        m_hasComputed       = false;
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private static double deadband(double value, double band) {
        return Math.abs(value) < band ? 0.0 : value;
    }
}
