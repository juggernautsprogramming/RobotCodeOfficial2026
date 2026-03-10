package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveToPoseConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionSubsystem;

/**
 * SnapHeadingToTag — single-press rotation snap to face the best visible AprilTag.
 *
 * <p>On button press:
 * <ol>
 *   <li>Reads the best tag currently visible from VisionSubsystem.</li>
 *   <li>Looks up that tag's field position from the loaded layout.</li>
 *   <li>Computes the field-relative heading that points the robot at the tag.</li>
 *   <li>Holds that heading with a ProfiledPIDController while the driver
 *       keeps full control of X/Y translation.</li>
 *   <li>Finishes automatically once within ±1° for 0.1 s.</li>
 * </ol>
 *
 * <p>If no tag is visible at the moment the button is pressed, the command
 * exits immediately without moving the robot.
 */
public class SnapHeadingToTag extends Command {

    private static final double HEADING_TOLERANCE_RAD = Units.degreesToRadians(1.0);
    private static final double DEBOUNCE_S             = 0.10;
    private static final double MAX_RUNTIME_S          = 3.0;

    private final CommandSwerveDrivetrain m_drivetrain;
    private final VisionSubsystem         m_vision;
    private final DoubleSupplier          m_translationX;
    private final DoubleSupplier          m_translationY;

    private final ProfiledPIDController m_thetaController;
    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();

    // ── Per-run state ─────────────────────────────────────────────────────────

    /** Field-relative heading (radians) the robot should snap to. */
    private double  m_targetHeadingRad;
    /** False if no tag was visible at initialize() — command exits immediately. */
    private boolean m_hasTarget;
    private double  m_atGoalStartS;
    private double  m_startTimeS;

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
        m_startTimeS   = Timer.getFPGATimestamp();
        m_atGoalStartS = Double.NaN;
        m_hasTarget    = false;

        // ── Snapshot: find the best visible tag right now ─────────────────────
        var bestTarget = m_vision.getBestTarget();
        if (bestTarget == null) {
            SmartDashboard.putString("Snap/State", "NO_TARGET");
            return;
        }

        // ── Look up the tag's field position from the layout ──────────────────
        var tagPoseOpt = VisionConstants.kTagLayout.getTagPose(bestTarget.getFiducialId());
        if (tagPoseOpt.isEmpty()) {
            SmartDashboard.putString("Snap/State", "TAG_NOT_IN_LAYOUT");
            return;
        }

        // ── Compute heading: robot centre → tag centre ────────────────────────
        Pose2d       robotPose = m_drivetrain.getState().Pose;
        Translation2d tagXY   = tagPoseOpt.get().toPose2d().getTranslation();
        Translation2d toTag   = tagXY.minus(robotPose.getTranslation());
        m_targetHeadingRad    = Math.atan2(toTag.getY(), toTag.getX());
        m_hasTarget           = true;

        // Seed controller with current heading + angular velocity for smooth start
        var speeds = m_drivetrain.getState().Speeds;
        m_thetaController.reset(
            robotPose.getRotation().getRadians(),
            speeds.omegaRadiansPerSecond);

        SmartDashboard.putString("Snap/State",    "RUNNING");
        SmartDashboard.putNumber("Snap/TagID",     bestTarget.getFiducialId());
        SmartDashboard.putNumber("Snap/TargetDeg", Math.toDegrees(m_targetHeadingRad));
    }

    @Override
    public void execute() {
        if (!m_hasTarget) return;

        Pose2d current = m_drivetrain.getState().Pose;

        // Heading PID — ProfiledPID gives smooth, non-jerky rotation
        double omegaCmd = MathUtil.clamp(
            m_thetaController.calculate(
                current.getRotation().getRadians(),
                m_targetHeadingRad),
            -DriveToPoseConstants.MAX_OMEGA_RAD_S,
             DriveToPoseConstants.MAX_OMEGA_RAD_S);

        // Driver keeps full X/Y control throughout the snap
        m_drivetrain.setControl(m_driveRequest
            .withVelocityX(m_translationX.getAsDouble())
            .withVelocityY(m_translationY.getAsDouble())
            .withRotationalRate(omegaCmd));

        // Debounce: must stay within tolerance for DEBOUNCE_S continuously
        if (m_thetaController.atGoal()) {
            if (Double.isNaN(m_atGoalStartS)) m_atGoalStartS = Timer.getFPGATimestamp();
        } else {
            m_atGoalStartS = Double.NaN;
        }

        // Telemetry
        double errDeg = Math.toDegrees(MathUtil.angleModulus(
            m_targetHeadingRad - current.getRotation().getRadians()));
        SmartDashboard.putNumber ("Snap/HeadingError_deg", errDeg);
        SmartDashboard.putBoolean("Snap/AtGoal",
            !Double.isNaN(m_atGoalStartS));

        // AdvantageScope: ghost robot at current XY but facing the snap target.
        // Add "Snap/TargetHeadingPose" as a robot overlay in the 2D field view.
        Logger.recordOutput("Snap/TargetHeadingPose",
            new Pose2d(current.getTranslation(), new Rotation2d(m_targetHeadingRad)));
    }

    @Override
    public boolean isFinished() {
        // No tag visible at start — exit immediately
        if (!m_hasTarget) return true;
        // Safety timeout
        if (Timer.getFPGATimestamp() - m_startTimeS > MAX_RUNTIME_S) return true;
        // Debounced at-goal
        return !Double.isNaN(m_atGoalStartS)
            && (Timer.getFPGATimestamp() - m_atGoalStartS) >= DEBOUNCE_S;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(new SwerveRequest.Idle());
        SmartDashboard.putString("Snap/State", interrupted ? "INTERRUPTED" : "COMPLETE");
    }
}
