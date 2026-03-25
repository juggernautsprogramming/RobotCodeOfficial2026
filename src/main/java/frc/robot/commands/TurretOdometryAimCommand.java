package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.TurretSubsystem;

/**
 * TurretOdometryAimCommand — continuously aims the turret at the hub center
 * using only the Kalman-fused robot pose from odometry. No vision required.
 *
 * Pre-match requirement: point turret straight ahead, press operator Y to zero.
 */
public class TurretOdometryAimCommand extends Command {

    private static final Translation2d HUB = ShooterConstants.HUB_CENTER;

    private final CommandSwerveDrivetrain m_drivetrain;
    private final TurretSubsystem         m_turret;

    public TurretOdometryAimCommand(
            CommandSwerveDrivetrain drivetrain,
            TurretSubsystem turret) {
        m_drivetrain = drivetrain;
        m_turret     = turret;
        addRequirements(turret);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private double computeTurretDeg() {
        Pose2d robot           = m_drivetrain.getState().Pose;
        double robotHeadingDeg = robot.getRotation().getDegrees();
        double headingRad      = Math.toRadians(robotHeadingDeg);

        // Correct for shooter exit offset from robot centre
        double shooterX = robot.getX() + ShooterConstants.SHOOTER_X_OFFSET_METERS * Math.cos(headingRad);
        double shooterY = robot.getY() + ShooterConstants.SHOOTER_X_OFFSET_METERS * Math.sin(headingRad);

        double dx          = HUB.getX() - shooterX;
        double dy          = HUB.getY() - shooterY;
        double fieldAimDeg = Math.toDegrees(Math.atan2(dy, dx));

        return MathUtil.inputModulus(fieldAimDeg - robotHeadingDeg, -180.0, 180.0);
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        if (!m_turret.hasBeenZeroed()) {
            // Warn loudly — aim will be offset but don't block movement
            SmartDashboard.putBoolean("TurretOdomAim/WARNING_NotZeroed", true);
        }
        // Snap immediately to correct angle with no FF on the first frame
        m_turret.setAngleDeg(computeTurretDeg());
        SmartDashboard.putBoolean("TurretOdomAim/Active", true);
    }

    @Override
    public void execute() {
        Pose2d robot           = m_drivetrain.getState().Pose;
        double robotHeadingDeg = robot.getRotation().getDegrees();
        double headingRad      = Math.toRadians(robotHeadingDeg);

        // Correct for shooter exit offset from robot centre
        double shooterX = robot.getX() + ShooterConstants.SHOOTER_X_OFFSET_METERS * Math.cos(headingRad);
        double shooterY = robot.getY() + ShooterConstants.SHOOTER_X_OFFSET_METERS * Math.sin(headingRad);

        double dx          = HUB.getX() - shooterX;
        double dy          = HUB.getY() - shooterY;
        double fieldAimDeg = Math.toDegrees(Math.atan2(dy, dx));

        double turretDeg = MathUtil.inputModulus(fieldAimDeg - robotHeadingDeg, -180.0, 180.0);

        // Pass robot angular velocity as-is.
        // setAngleDegWithFF negates it internally so the turret counter-rotates
        // to stay locked on the hub. Do NOT negate here — double negation was
        // previously cancelling the correction entirely.
        double robotAngularVelRadPerSec = m_drivetrain.getState().Speeds.omegaRadiansPerSecond;
        m_turret.setAngleDegWithFF(turretDeg, robotAngularVelRadPerSec);

        // Telemetry
        SmartDashboard.putNumber ("TurretOdomAim/FieldAimDeg",       fieldAimDeg);
        SmartDashboard.putNumber ("TurretOdomAim/RobotHeadingDeg",   robotHeadingDeg);
        SmartDashboard.putNumber ("TurretOdomAim/TurretTargetDeg",   turretDeg);
        SmartDashboard.putNumber ("TurretOdomAim/TurretActualDeg",   m_turret.getAngleDeg());
        SmartDashboard.putNumber ("TurretOdomAim/ErrorDeg",          turretDeg - m_turret.getAngleDeg());
        SmartDashboard.putBoolean("TurretOdomAim/AtTarget",          m_turret.isAtTarget());
        SmartDashboard.putBoolean("TurretOdomAim/WARNING_NotZeroed", !m_turret.hasBeenZeroed());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.setAngleDeg(m_turret.getAngleDeg()); // hold on handoff
        SmartDashboard.putBoolean("TurretOdomAim/Active", false);
    }
}