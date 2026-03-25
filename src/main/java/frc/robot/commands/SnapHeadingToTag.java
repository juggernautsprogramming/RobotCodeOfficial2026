package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveToPoseConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionSubsystem;

/**
 * SnapHeadingToTag — continuous rotation alignment toward the best visible AprilTag.
 *
 * <p>While the button is held:
 * <ol>
 *   <li>Every execute() cycle queries the best tag visible from VisionSubsystem.</li>
 *   <li>Looks up that tag's field position from the 3D AprilTag layout.</li>
 *   <li>Computes the field-relative heading (robot centre → tag centre) using the
 *       Kalman-fused robot pose — field-space geometry, not raw camera yaw.</li>
 *   <li>Drives the robot toward that heading with a ProfiledPIDController while
 *       the driver keeps full X/Y translation control.</li>
 *   <li>If the tag temporarily disappears, the last known heading is held until the
 *       tag re-appears or the safety timeout fires.</li>
 * </ol>
 *
 * <p>Works with <b>any</b> AprilTag in the loaded layout — hub tags, reef tags,
 * or any custom tag — and can be re-triggered as many times as needed.
 * Bind with {@code .whileTrue()} for the best experience.
 *
 * <p>AdvantageScope keys:
 * <pre>
 *   Snap/State                — RUNNING / INTERRUPTED / NO_TARGET
 *   Snap/TagID                — fiducial ID of the tag being tracked
 *   Snap/TargetDeg            — field-relative target heading (degrees)
 *   Snap/HeadingError_deg     — current heading error (degrees)
 *   Snap/AtGoal               — true when within tolerance
 *   Snap/TargetHeadingPose    — 2D ghost pose at current XY, facing the tag
 *   Snap/TargetHeadingPose3d  — same as above as Pose3d for the 3D field view
 * </pre>
 */
public class SnapHeadingToTag extends Command {

    private static final double HEADING_TOLERANCE_RAD = Units.degreesToRadians(1.0);
    /** Safety timeout — command auto-cancels if held longer than this (seconds). */
    private static final double MAX_RUNTIME_S          = 30.0;

    private final CommandSwerveDrivetrain m_drivetrain;
    private final VisionSubsystem         m_vision;
    private final DoubleSupplier          m_translationX;
    private final DoubleSupplier          m_translationY;

    private final ProfiledPIDController m_thetaController;
    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();

    // ── Per-run state ─────────────────────────────────────────────────────────

    /** Field-relative heading (radians) to face the tracked tag. Updated every loop. */
    private double  m_targetHeadingRad;
    /** True once any tag has been seen at least once since initialize(). */
    private boolean m_hasTarget;
    private double  m_startTimeS;
    /** Fiducial ID of the tag locked onto. -1 = not yet locked. */
    private int     m_lockedTagId;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * @param drivetrain   Swerve drivetrain.
     * @param vision       VisionSubsystem — queried once on initialize().
     * @param translationX Driver forward/back axis (m/s, already scaled to max speed).
     * @param translationY Driver left/right axis (m/s, already scaled to max speed).
     */
    public SnapHeadingToTag(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            DoubleSupplier translationX,
            DoubleSupplier translationY) {

        m_drivetrain    = drivetrain;
        m_vision        = vision;
        m_translationX  = translationX;
        m_translationY  = translationY;

        m_thetaController = new ProfiledPIDController(
            DriveToPoseConstants.kP_THETA,
            DriveToPoseConstants.kI_THETA,
            DriveToPoseConstants.kD_THETA,
            new TrapezoidProfile.Constraints(
                DriveToPoseConstants.MAX_OMEGA_RAD_S,
                DriveToPoseConstants.MAX_ALPHA_RAD_S2));

        // Shortest-arc rotation always
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_thetaController.setTolerance(HEADING_TOLERANCE_RAD);

        addRequirements(drivetrain);
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_startTimeS = Timer.getFPGATimestamp();
        m_hasTarget  = false;
        m_targetHeadingRad = 0.0;
        m_lockedTagId = -1;

        // Seed theta controller from current state for a smooth start
        Pose2d robotPose = m_drivetrain.getState().Pose;
        var    speeds    = m_drivetrain.getState().Speeds;
        m_thetaController.reset(
            robotPose.getRotation().getRadians(),
            speeds.omegaRadiansPerSecond);

        SmartDashboard.putString("Snap/State", "RUNNING");
    }

    @Override
    public void execute() {
        Pose2d current = m_drivetrain.getState().Pose;

        // ── Continuously update target heading from the best visible tag ───────
        // Uses the Kalman-fused robot pose and the tag's 3D field position,
        // so alignment is based on actual field geometry — not raw camera yaw.
        // If the tag disappears, m_targetHeadingRad holds the last known value.
        var bestTarget = m_vision.getBestTarget();
        if (bestTarget != null) {
            int targetId = bestTarget.getFiducialId();

            // Lock onto the first tag seen — never switch mid-command.
            // This prevents sudden heading jumps when a different tag becomes "best."
            if (m_lockedTagId == -1) {
                m_lockedTagId = targetId;
            }

            // Only update heading when we see the locked tag
            if (targetId == m_lockedTagId) {
                // Hub tags → aim at hub center for a clean shot angle.
                // Any other tag → aim at the tag's field position.
                Translation2d aimPoint = VisionSubsystem.isHubTag(targetId)
                    ? ShooterConstants.HUB_CENTER
                    : VisionConstants.kTagLayout.getTagPose(targetId)
                        .map(p -> p.toPose2d().getTranslation())
                        .orElse(null);

                if (aimPoint != null) {
                    Translation2d toTarget = aimPoint.minus(current.getTranslation());
                    m_targetHeadingRad = Math.atan2(toTarget.getY(), toTarget.getX());
                    m_hasTarget        = true;

                    SmartDashboard.putNumber("Snap/TagID",     targetId);
                    SmartDashboard.putNumber("Snap/TargetDeg", Math.toDegrees(m_targetHeadingRad));
                }
            }
        }

        // No tag ever seen — stand by without moving
        if (!m_hasTarget) {
            SmartDashboard.putString("Snap/State", "NO_TARGET");
            return;
        }

        // ── Heading PID — ProfiledPID for smooth, non-jerky rotation ──────────
        double omegaCmd = MathUtil.clamp(
            m_thetaController.calculate(
                current.getRotation().getRadians(),
                m_targetHeadingRad),
            -DriveToPoseConstants.MAX_OMEGA_RAD_S,
             DriveToPoseConstants.MAX_OMEGA_RAD_S);

        // Driver keeps full X/Y control throughout alignment
        m_drivetrain.setControl(m_driveRequest
            .withVelocityX(m_translationX.getAsDouble())
            .withVelocityY(m_translationY.getAsDouble())
            .withRotationalRate(omegaCmd));

        // ── Telemetry ─────────────────────────────────────────────────────────
        double errDeg = Math.toDegrees(MathUtil.angleModulus(
            m_targetHeadingRad - current.getRotation().getRadians()));
        SmartDashboard.putNumber ("Snap/HeadingError_deg", errDeg);
        SmartDashboard.putBoolean("Snap/AtGoal", m_thetaController.atGoal());

        // AdvantageScope 2D — ghost robot at current XY facing the snap target.
        Pose2d targetPose2d = new Pose2d(current.getTranslation(), new Rotation2d(m_targetHeadingRad));
        Logger.recordOutput("Snap/TargetHeadingPose",   targetPose2d);
        // AdvantageScope 3D — same pose lifted to Z=0 for the 3D field view.
        Logger.recordOutput("Snap/TargetHeadingPose3d", new Pose3d(targetPose2d));
    }

    @Override
    public boolean isFinished() {
        // Hold until button released (command interrupted) or safety timeout
        return Timer.getFPGATimestamp() - m_startTimeS > MAX_RUNTIME_S;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(new SwerveRequest.Idle());
        SmartDashboard.putString("Snap/State", interrupted ? "INTERRUPTED" : "COMPLETE");
        // Park ghost poses off-field so stale overlays don't confuse the 2D/3D views
        Logger.recordOutput("Snap/TargetHeadingPose",   new Pose2d(-10, -10, new Rotation2d()));
        Logger.recordOutput("Snap/TargetHeadingPose3d", new Pose3d(-10, -10, 0,
            new edu.wpi.first.math.geometry.Rotation3d()));
    }
}
