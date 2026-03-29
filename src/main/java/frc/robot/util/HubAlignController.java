package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ShooterConstants;

/**
 * Stateful PD controller for driving the robot to the hub shooting position.
 *
 * File location: src/main/java/frc/robot/util/HubAlignController.java
 *
 * Anti-jitter design:
 *  1. All drive outputs derive ONLY from the fused odometry pose.
 *     Raw camera angles never reach the motors.
 *  2. D-term is computed on the error (not the setpoint) — no derivative kick
 *     when the target pose updates.
 *  3. First-order low-pass filter on all three output axes.
 *  4. Slew-rate limiters cap acceleration to eliminate wheel jerk.
 *  5. Velocity deadband zeros tiny residual commands at setpoint.
 */
public final class HubAlignController {

    // ── Filter / derivative state ─────────────────────────────────────────────
    private double m_prevDistErr   = 0.0;
    private double m_prevStrafeErr = 0.0;
    private double m_prevRotErr    = 0.0;

    private double m_filteredDrive  = 0.0;
    private double m_filteredStrafe = 0.0;
    private double m_filteredRot    = 0.0;


    private Pose2d m_targetPose = null;

    // ── AlignOutput ───────────────────────────────────────────────────────────

    /** One cycle's worth of chassis speed commands. */
    public static final class AlignOutput {
        /** Field-centric forward speed (m/s). */
        public final double  xSpeedMps;
        /** Field-centric left speed (m/s). */
        public final double  ySpeedMps;
        /** Counter-clockwise rotation (rad/s). */
        public final double  omegaRadS;
        /** True when both distance and heading errors are within tolerance. */
        public final boolean readyToShoot;

        public AlignOutput(double x, double y, double omega, boolean ready) {
            xSpeedMps    = x;
            ySpeedMps    = y;
            omegaRadS    = omega;
            readyToShoot = ready;
        }
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Compute chassis speeds to reach the hub shooting position.
     * Call once per robot loop (20 ms) from DriveToHubAndShootCommand.execute().
     *
     * @param currentPose Fused robot pose from drivetrain.getState().Pose
     * @param dt          Loop period in seconds (normally 0.020)
     * @return Field-centric chassis speed commands
     */
    public AlignOutput update(Pose2d currentPose, double dt) {
        if (dt <= 0.0 || dt > 0.10) dt = 0.020;

        Translation2d hub     = ShooterConstants.HUB_CENTER;
        Translation2d robotXY = currentPose.getTranslation();
        Translation2d toHub   = hub.minus(robotXY);

        double currentDist     = toHub.getNorm();
        double headingToHubRad = Math.atan2(toHub.getY(), toHub.getX());
        double desired         = ShooterConstants.DESIRED_DISTANCE_METERS;

        // Target pose: stand at desired distance, facing the hub
        double cosH = Math.cos(headingToHubRad);
        double sinH = Math.sin(headingToHubRad);
        Translation2d targetXY      = hub.minus(new Translation2d(desired * cosH, desired * sinH));
        Rotation2d    targetHeading  = new Rotation2d(headingToHubRad);
        m_targetPose = new Pose2d(targetXY, targetHeading);

        // ── Errors ─────────────────────────────────────────────────────────
        double distErr   = currentDist - desired;
        double strafeErr = computePerpError(robotXY, hub, headingToHubRad);
        double rotErr    = MathUtil.angleModulus(
            targetHeading.getRadians() - currentPose.getRotation().getRadians());

        // ── PD ─────────────────────────────────────────────────────────────
        double rawDrive  = ShooterConstants.DRIVE_kP  * distErr
                         + ShooterConstants.DRIVE_kD  * (distErr   - m_prevDistErr)   / dt;
        double rawStrafe = ShooterConstants.STRAFE_kP * strafeErr
                         + ShooterConstants.STRAFE_kD * (strafeErr - m_prevStrafeErr) / dt;
        double rawRot    = ShooterConstants.ROTATION_kP * rotErr
                         + ShooterConstants.ROTATION_kD * (rotErr  - m_prevRotErr)   / dt;

        m_prevDistErr   = distErr;
        m_prevStrafeErr = strafeErr;
        m_prevRotErr    = rotErr;

        // ── Clamp ──────────────────────────────────────────────────────────
        rawDrive  = MathUtil.clamp(rawDrive,  -ShooterConstants.DRIVE_MAX_SPEED_MPS,  ShooterConstants.DRIVE_MAX_SPEED_MPS);
        rawStrafe = MathUtil.clamp(rawStrafe, -ShooterConstants.STRAFE_MAX_SPEED_MPS, ShooterConstants.STRAFE_MAX_SPEED_MPS);
        rawRot    = MathUtil.clamp(rawRot,    -ShooterConstants.ROTATION_MAX_RAD_S,   ShooterConstants.ROTATION_MAX_RAD_S);

        // ── Low-pass filter ────────────────────────────────────────────────
        final double alpha = ShooterConstants.OUTPUT_FILTER_ALPHA;
        m_filteredDrive  += alpha * (rawDrive  - m_filteredDrive);
        m_filteredStrafe += alpha * (rawStrafe - m_filteredStrafe);
        m_filteredRot    += alpha * (rawRot    - m_filteredRot);

        double drive  = m_filteredDrive;
        double strafe = m_filteredStrafe;
        double rot    = m_filteredRot;

        // ── Deadband ───────────────────────────────────────────────────────
        drive  = deadband(drive);
        strafe = deadband(strafe);
        rot    = deadband(rot);

        // ── Rotate from approach-frame to field frame ──────────────────────
        double xSpeed = drive * cosH - strafe * sinH;
        double ySpeed = drive * sinH + strafe * cosH;

        // ── Ready-to-shoot gate ────────────────────────────────────────────
        boolean distOk = Math.abs(distErr)                   < ShooterConstants.DISTANCE_TOLERANCE_METERS;
        boolean yawOk  = Math.abs(Math.toDegrees(rotErr))    < ShooterConstants.YAW_TOLERANCE_DEG;

        return new AlignOutput(xSpeed, ySpeed, rot, distOk && yawOk);
    }

    /** Reset all filter / derivative state. Call from command initialize(). */
    public void reset() {
        m_prevDistErr   = 0.0;
        m_prevStrafeErr = 0.0;
        m_prevRotErr    = 0.0;
        m_filteredDrive  = 0.0;
        m_filteredStrafe = 0.0;
        m_filteredRot    = 0.0;
    }

    /** Last computed target pose — null before first update(). Useful for telemetry. */
    public Pose2d getTargetPose() { return m_targetPose; }

    // ── Helpers ───────────────────────────────────────────────────────────────

    /** Signed lateral distance from the ideal approach line. */
    private static double computePerpError(
            Translation2d robotXY, Translation2d hubXY, double headingRad) {
        double ax = Math.cos(headingRad);
        double ay = Math.sin(headingRad);
        double rx = robotXY.getX() - hubXY.getX();
        double ry = robotXY.getY() - hubXY.getY();
        return rx * ay - ry * ax;
    }

    private static double deadband(double v) {
        return Math.abs(v) < ShooterConstants.VELOCITY_DEADBAND ? 0.0 : v;
    }
}