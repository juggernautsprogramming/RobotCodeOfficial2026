package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.TurretSubsystem;

/**
 * TurretGimbalModeCommand — continuously aims the turret at the 2026 hub
 * using Kalman-fused robot pose. Tracks x, y, and θ in the horizontal plane.
 *
 * <p>Replaces TurretOdometryAimCommand. Constructor signature is identical.
 *
 * <h3>Offset tuning</h3>
 * If the turret consistently points N° away from the hub, set
 * {@link TurretConstants#TURRET_AIM_TRIM_DEG} in Constants.java.
 * Positive = shift right, negative = shift left.
 *
 * <h3>Diagnosing the root cause of a fixed offset</h3>
 * <pre>
 *   Rotate robot 90° — if error stays the same:
 *     → turret was zeroed off-center (mechanical zero issue)
 *     → fix: re-zero with turret physically straight, or adjust trim
 *   Rotate robot 90° — if error changes:
 *     → gyro is not zeroed to field frame
 *     → fix: press driver left bumper (seedFieldCentric) while robot
 *            is square to a field wall, then re-enable gimbal mode
 * </pre>
 *
 * <h3>Convention (inherits from TurretSubsystem)</h3>
 * <pre>
 *   0° = straight ahead,  +° = right,  −° = left
 *   Hard stops: right +279.75°, left −266.22°
 *   Soft limits: right +260°,   left −260°
 * </pre>
 *
 * <h3>Pre-match requirement</h3>
 * Point turret straight ahead, press operator Y to zero before enabling.
 */
public class TurretGimbalModeCommand extends Command {

    private static final Translation2d HUB = ShooterConstants.HUB_CENTER;

    private final CommandSwerveDrivetrain m_drivetrain;
    private final TurretSubsystem         m_turret;

    public TurretGimbalModeCommand(
            CommandSwerveDrivetrain drivetrain,
            TurretSubsystem turret) {
        m_drivetrain = drivetrain;
        m_turret     = turret;
        addRequirements(turret);
    }

    // ── Geometry ──────────────────────────────────────────────────────────────

    /**
     * Computes the robot-frame turret angle needed to aim at the hub, plus
     * applies {@link TurretConstants#TURRET_AIM_TRIM_DEG} to correct for
     * mechanical zero error or gyro frame mismatch.
     */
    private double computeTurretDeg(Pose2d robot) {
        double robotHeadingDeg = robot.getRotation().getDegrees();
        double headingRad      = Math.toRadians(robotHeadingDeg);

        double shooterX = robot.getX()
                + ShooterConstants.SHOOTER_X_OFFSET_METERS * Math.cos(headingRad);
        double shooterY = robot.getY()
                + ShooterConstants.SHOOTER_X_OFFSET_METERS * Math.sin(headingRad);

        double dx          = HUB.getX() - shooterX;
        double dy          = HUB.getY() - shooterY;
        double fieldAimDeg = Math.toDegrees(Math.atan2(dy, dx));

        double raw = MathUtil.inputModulus(fieldAimDeg - robotHeadingDeg, -180.0, 180.0);

        // Apply trim — positive shifts turret right, negative shifts left.
        // Wrap again after trim so the result stays in (−180, +180].
        return MathUtil.inputModulus(raw + TurretConstants.TURRET_AIM_TRIM_DEG, -180.0, 180.0);
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        if (!m_turret.hasBeenZeroed()) {
            SmartDashboard.putBoolean("TurretGimbal/WARNING_NotZeroed", true);
        }
        // Snap to correct angle immediately — no FF on first frame
        m_turret.setAngleDeg(computeTurretDeg(m_drivetrain.getState().Pose));
        SmartDashboard.putBoolean("TurretGimbal/Active", true);
    }

    @Override
    public void execute() {
        Pose2d robot           = m_drivetrain.getState().Pose;
        double robotHeadingDeg = robot.getRotation().getDegrees();
        double headingRad      = Math.toRadians(robotHeadingDeg);

        double shooterX = robot.getX()
                + ShooterConstants.SHOOTER_X_OFFSET_METERS * Math.cos(headingRad);
        double shooterY = robot.getY()
                + ShooterConstants.SHOOTER_X_OFFSET_METERS * Math.sin(headingRad);

        double dx          = HUB.getX() - shooterX;
        double dy          = HUB.getY() - shooterY;
        double fieldAimDeg = Math.toDegrees(Math.atan2(dy, dx));

        double rawTurretDeg = MathUtil.inputModulus(fieldAimDeg - robotHeadingDeg, -180.0, 180.0);
        double turretDeg    = MathUtil.inputModulus(
                rawTurretDeg + TurretConstants.TURRET_AIM_TRIM_DEG, -180.0, 180.0);

        // Pass robot omega as-is — setAngleDegWithFF negates it internally
        // so the turret counter-rotates to stay locked. Do NOT negate here.
        double robotOmegaRadPerSec = m_drivetrain.getState().Speeds.omegaRadiansPerSecond;
        m_turret.setAngleDegWithFF(turretDeg, robotOmegaRadPerSec);

        SmartDashboard.putNumber ("TurretGimbal/FieldAimDeg",       fieldAimDeg);
        SmartDashboard.putNumber ("TurretGimbal/RobotHeadingDeg",   robotHeadingDeg);
        SmartDashboard.putNumber ("TurretGimbal/RawTargetDeg",      rawTurretDeg);   // pre-trim
        SmartDashboard.putNumber ("TurretGimbal/TargetDeg",         turretDeg);      // post-trim
        SmartDashboard.putNumber ("TurretGimbal/TrimDeg",           TurretConstants.TURRET_AIM_TRIM_DEG);
        SmartDashboard.putNumber ("TurretGimbal/ActualDeg",         m_turret.getAngleDeg());
        SmartDashboard.putNumber ("TurretGimbal/ErrorDeg",          turretDeg - m_turret.getAngleDeg());
        SmartDashboard.putNumber ("TurretGimbal/HorizDist_m",       Math.sqrt(dx * dx + dy * dy));
        SmartDashboard.putBoolean("TurretGimbal/AtTarget",          m_turret.isAtTarget());
        SmartDashboard.putBoolean("TurretGimbal/WARNING_NotZeroed", !m_turret.hasBeenZeroed());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.setAngleDeg(m_turret.getAngleDeg()); // hold on handoff
        SmartDashboard.putBoolean("TurretGimbal/Active", false);
    }
}