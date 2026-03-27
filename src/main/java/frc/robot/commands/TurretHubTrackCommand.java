package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

/**
 * TurretHubTrackCommand — continuously aims the turret at the hub using the
 * turret camera's AprilTag yaw as the primary source, falling back to
 * Kalman-fused odometry geometry when no hub tag is visible.
 *
 * <h3>Source priority</h3>
 * <ol>
 *   <li><b>Camera (primary)</b> — when a hub AprilTag is visible the turret is
 *       driven directly to the camera-relative yaw. Highest accuracy; immune to
 *       odometry drift.</li>
 *   <li><b>Odometry (fallback)</b> — when the tag is lost the turret is driven
 *       to the geometrically computed angle from the robot's Kalman pose, with
 *       omega feedforward to keep it locked while the robot rotates.</li>
 * </ol>
 *
 * <p>Use as the <b>default turret command</b> for always-on, shoot-on-the-move
 * hub tracking.
 */
public class TurretHubTrackCommand extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final TurretSubsystem         m_turret;
    private final VisionSubsystem         m_vision;

    public TurretHubTrackCommand(
            CommandSwerveDrivetrain drivetrain,
            TurretSubsystem turret,
            VisionSubsystem vision) {
        m_drivetrain = drivetrain;
        m_turret     = turret;
        m_vision     = vision;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        // Snap immediately to avoid a large first-frame jump
        m_turret.setAngleDeg(computeOdometryTurretDeg());
        SmartDashboard.putBoolean("TurretTrack/Active", true);
    }

    @Override
    public void execute() {
        var hubTarget = m_vision.getBestHubTarget();
        boolean hasTag = hubTarget != null && !Double.isNaN(hubTarget.getYaw());

        if (hasTag) {
            // ── Camera path: drive directly to camera-relative yaw ───────────
            double turretDeg = MathUtil.inputModulus(
                hubTarget.getYaw() + TurretConstants.TURRET_AIM_TRIM_DEG, -180.0, 180.0);
            m_turret.setAngleDeg(turretDeg);

            SmartDashboard.putString("TurretTrack/Source",     "Camera");
            SmartDashboard.putNumber("TurretTrack/CamYaw_deg", hubTarget.getYaw());
            SmartDashboard.putNumber("TurretTrack/Target_deg", turretDeg);
        } else {
            // ── Odometry fallback: geometry + omega feedforward ───────────────
            double turretDeg = computeOdometryTurretDeg();
            double omega     = m_drivetrain.getState().Speeds.omegaRadiansPerSecond;
            m_turret.setAngleDegWithFF(turretDeg, omega);

            SmartDashboard.putString("TurretTrack/Source",     "Odometry");
            SmartDashboard.putNumber("TurretTrack/Target_deg", turretDeg);
        }

        SmartDashboard.putBoolean("TurretTrack/HasTag",    hasTag);
        SmartDashboard.putNumber ("TurretTrack/Actual_deg", m_turret.getAngleDeg());
        SmartDashboard.putNumber ("TurretTrack/Error_deg",
            SmartDashboard.getNumber("TurretTrack/Target_deg", 0)
                - m_turret.getAngleDeg());
        SmartDashboard.putBoolean("TurretTrack/AtTarget",  m_turret.isAtTarget());
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean interrupted) {
        m_turret.setAngleDeg(m_turret.getAngleDeg()); // hold on handoff
        SmartDashboard.putBoolean("TurretTrack/Active", false);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private double computeOdometryTurretDeg() {
        var robot          = m_drivetrain.getState().Pose;
        double headingDeg  = robot.getRotation().getDegrees();
        double headingRad  = Math.toRadians(headingDeg);

        double shooterX = robot.getX()
            + ShooterConstants.SHOOTER_X_OFFSET_METERS * Math.cos(headingRad);
        double shooterY = robot.getY()
            + ShooterConstants.SHOOTER_X_OFFSET_METERS * Math.sin(headingRad);

        double dx         = ShooterConstants.HUB_CENTER.getX() - shooterX;
        double dy         = ShooterConstants.HUB_CENTER.getY() - shooterY;
        double fieldAimDeg = Math.toDegrees(Math.atan2(dy, dx));

        return MathUtil.inputModulus(
            fieldAimDeg - headingDeg + TurretConstants.TURRET_AIM_TRIM_DEG,
            -180.0, 180.0);
    }
}
