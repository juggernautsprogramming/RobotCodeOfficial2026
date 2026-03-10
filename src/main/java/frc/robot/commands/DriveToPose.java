package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.DriveToPoseConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionSubsystem;

/**
 * DriveToPose — jitter-free snap-to-target command for swerve drive.
 *
 * <h3>Anti-jitter design</h3>
 * Jitter in a drive-to-pose command has two sources:
 * <ol>
 *   <li><b>Kalman filter mid-motion corrections</b> — while the robot is moving,
 *       {@code VisionSubsystem} continuously injects new pose estimates into the
 *       {@code SwerveDrivePoseEstimator}. Each correction shifts the robot's
 *       perceived position by a few millimetres, the PID sees this as a new error,
 *       and the motor outputs stutter accordingly.</li>
 *   <li><b>Encoder quantisation noise</b> — even without vision, wheel odometry
 *       has sub-millimetre sample-to-sample noise that the D-term amplifies into
 *       micro-corrections every loop.</li>
 * </ol>
 *
 * <h3>Fixes applied</h3>
 * <ul>
 *   <li><b>Snapshot target</b> — the target {@link Pose2d} is resolved
 *       <em>once</em> in {@code initialize()} from the Kalman-fused pose (which
 *       already incorporates the last vision frame). The target never changes
 *       after that, so no moving-setpoint transients.</li>
 *   <li><b>Vision paused during motion</b> — {@code initialize()} calls
 *       {@link VisionSubsystem#setVisionEnabled(boolean) setVisionEnabled(false)},
 *       which stops the Kalman filter from receiving new measurements while the
 *       maneuver is in progress. The robot navigates on pure wheel odometry + gyro
 *       for the duration — deterministic, smooth, no mid-motion pose jumps.
 *       {@code end()} always restores vision.</li>
 *   <li><b>Pose input low-pass filter</b> — a 2-tap moving average is applied to
 *       the X and Y pose readings fed into the PID controllers, eliminating
 *       single-loop encoder quantisation spikes from the D-term output.
 *       Theta is not filtered because the gyro is already noiseless at 250 Hz.</li>
 *   <li><b>Velocity-seeded profiles</b> — all three {@link ProfiledPIDController}s
 *       are reset with the robot's current velocity in {@code initialize()}, so
 *       the trapezoidal profiler plans a smooth deceleration from whatever speed
 *       the robot is already moving. No overshoot, no tuned feedforward constant.</li>
 * </ul>
 *
 * <h3>Tag-space → field-space</h3>
 * Use {@link #poseInFrontOfTag} to compute the target pose relative to a tag.
 * The tag's {@code +X} axis points into the field; the robot's heading is set to
 * {@code tagHeading + 180°} so it faces the tag.
 *
 * <h3>RobotContainer usage</h3>
 * <pre>{@code
 * // Snap to 6 inches in front of tag 7; driver can cancel at any time
 * m_driverStick.a().onTrue(
 *     DriveToPose.toTag(drivetrain, visionSubsystem, 7,
 *         Units.inchesToMeters(6), 0.0,
 *         () -> -m_driverStick.getLeftY(),
 *         () -> -m_driverStick.getLeftX(),
 *         () -> -m_driverStick.getRightX())
 * );
 * }</pre>
 *
 * <h3>Dashboard keys ({@code DTP/*})</h3>
 * <pre>
 *   DTP/State            — RUNNING / COMPLETE / INTERRUPTED / NO_TAG
 *   DTP/XError_m         — field X error (metres)
 *   DTP/YError_m         — field Y error (metres)
 *   DTP/ThetaError_deg   — heading error (degrees)
 *   DTP/DistToTarget_m   — straight-line distance to target (metres)
 *   DTP/AtGoal           — all three controllers within tolerance
 *   DTP/HoldTime_s       — consecutive seconds within tolerance
 *   DTP/DriverOverride   — true if driver joystick is cancelling the command
 * </pre>
 */
public class DriveToPose extends Command {

    // ── PID controllers ───────────────────────────────────────────────────────

    private final ProfiledPIDController m_xController;
    private final ProfiledPIDController m_yController;
    /** Continuous-input heading controller — always takes the shortest arc. */
    private final ProfiledPIDController m_thetaController;

    // ── Pose input smoothing ──────────────────────────────────────────────────

    /**
     * 2-tap moving average on the X/Y pose readings fed into the PID.
     * Kills single-loop encoder quantisation spikes before they reach the D-term.
     * Theta is not filtered — the gyro is already noiseless at 250 Hz.
     */
    private final LinearFilter m_xFilter = LinearFilter.movingAverage(2);
    private final LinearFilter m_yFilter = LinearFilter.movingAverage(2);

    // ── Dependencies ──────────────────────────────────────────────────────────

    private final CommandSwerveDrivetrain m_drivetrain;
    /** May be null for autonomous use where vision pausing is not needed. */
    private final VisionSubsystem         m_vision;
    private final Supplier<Optional<Pose2d>> m_targetSupplier;

    // ── Driver override ───────────────────────────────────────────────────────

    private final DoubleSupplier m_driverX;
    private final DoubleSupplier m_driverY;
    private final DoubleSupplier m_driverOmega;

    // ── Per-run state ─────────────────────────────────────────────────────────

    private Pose2d m_targetPose;   // null if tag was not found in layout
    private double m_atGoalStartS; // NaN while outside tolerance
    private double m_startTimeS;

    // ── Swerve output ─────────────────────────────────────────────────────────

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * Full constructor. For most use cases prefer the static factory methods.
     *
     * @param drivetrain     Swerve drivetrain (required by this command).
     * @param vision         VisionSubsystem to pause during the maneuver.
     *                       Pass {@code null} for autonomous-only use.
     * @param targetSupplier Called once in initialize() to resolve the target pose.
     * @param driverX        Forward/back joystick axis for override detection.
     * @param driverY        Left/right joystick axis for override detection.
     * @param driverOmega    Rotation joystick axis for override detection.
     */
    public DriveToPose(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            Supplier<Optional<Pose2d>> targetSupplier,
            DoubleSupplier driverX,
            DoubleSupplier driverY,
            DoubleSupplier driverOmega) {

        m_drivetrain     = drivetrain;
        m_vision         = vision;
        m_targetSupplier = targetSupplier;
        m_driverX        = driverX;
        m_driverY        = driverY;
        m_driverOmega    = driverOmega;

        TrapezoidProfile.Constraints xyConstraints = new TrapezoidProfile.Constraints(
            DriveToPoseConstants.MAX_VEL_MPS,
            DriveToPoseConstants.MAX_ACCEL_MPS2);

        m_xController = new ProfiledPIDController(
            DriveToPoseConstants.kP_XY,
            DriveToPoseConstants.kI_XY,
            DriveToPoseConstants.kD_XY,
            xyConstraints);

        m_yController = new ProfiledPIDController(
            DriveToPoseConstants.kP_XY,
            DriveToPoseConstants.kI_XY,
            DriveToPoseConstants.kD_XY,
            xyConstraints);

        m_thetaController = new ProfiledPIDController(
            DriveToPoseConstants.kP_THETA,
            DriveToPoseConstants.kI_THETA,
            DriveToPoseConstants.kD_THETA,
            new TrapezoidProfile.Constraints(
                DriveToPoseConstants.MAX_OMEGA_RAD_S,
                DriveToPoseConstants.MAX_ALPHA_RAD_S2));

        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        m_xController.setTolerance(DriveToPoseConstants.XY_TOLERANCE_M);
        m_yController.setTolerance(DriveToPoseConstants.XY_TOLERANCE_M);
        m_thetaController.setTolerance(DriveToPoseConstants.THETA_TOLERANCE_RAD);

        addRequirements(drivetrain);
    }

    // ── Static factories ──────────────────────────────────────────────────────

    /**
     * Snap to a pose directly in front of an AprilTag (teleop use, with driver
     * override and vision pausing).
     *
     * @param drivetrain     Swerve drivetrain.
     * @param vision         VisionSubsystem — paused during the maneuver.
     * @param tagId          AprilTag ID (must be in the loaded layout).
     * @param standoffMeters Robot-bumper-to-tag face distance (metres).
     * @param yOffsetMeters  Lateral offset in tag-local frame (0 = centred).
     * @param driverX        Forward/back joystick for override detection.
     * @param driverY        Left/right joystick for override detection.
     * @param driverOmega    Rotation joystick for override detection.
     */
    public static DriveToPose toTag(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            int tagId,
            double standoffMeters,
            double yOffsetMeters,
            DoubleSupplier driverX,
            DoubleSupplier driverY,
            DoubleSupplier driverOmega) {

        return new DriveToPose(
            drivetrain, vision,
            () -> poseInFrontOfTag(tagId, standoffMeters, yOffsetMeters),
            driverX, driverY, driverOmega);
    }

    /**
     * Drive to a static field-space pose (autonomous use, no driver override).
     *
     * @param drivetrain  Swerve drivetrain.
     * @param vision      VisionSubsystem — paused during the maneuver.
     * @param targetPose  Field-space target pose.
     */
    public static DriveToPose toPose(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            Pose2d targetPose) {

        return new DriveToPose(
            drivetrain, vision,
            () -> Optional.of(targetPose),
            () -> 0.0, () -> 0.0, () -> 0.0);
    }

    // ── Tag-space → field-space ───────────────────────────────────────────────

    /**
     * Compute the field-space robot target pose for standing in front of a tag.
     *
     * <p>The tag's {@code +X} axis points outward from the wall into the field.
     * Moving {@code standoffMeters} along that axis places the robot in front of
     * the tag. {@code Rotation2d.k180deg} rotates the robot heading so it faces
     * back toward the tag (toward the wall).
     *
     * @param tagId          AprilTag ID in {@link VisionConstants#kTagLayout}.
     * @param standoffMeters Distance from tag face to robot reference point.
     * @param yOffsetMeters  Lateral offset (positive = left when facing the tag).
     * @return Field-space target pose, or empty if the tag is not in the layout.
     */
    public static Optional<Pose2d> poseInFrontOfTag(
            int tagId, double standoffMeters, double yOffsetMeters) {

        var tagPoseOpt = VisionConstants.kTagLayout.getTagPose(tagId);
        if (tagPoseOpt.isEmpty()) return Optional.empty();

        Pose2d tagPose2d = tagPoseOpt.get().toPose2d();
        Pose2d target    = tagPose2d.transformBy(new Transform2d(
            new Translation2d(standoffMeters, yOffsetMeters),
            Rotation2d.k180deg));

        return Optional.of(target);
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_atGoalStartS = Double.NaN;
        m_startTimeS   = Timer.getFPGATimestamp();

        // ── Resolve target pose (snapshot — never changes again) ─────────────
        // The Kalman-fused pose already incorporates the latest vision frame,
        // so this is effectively "taking a picture" of the field state right now.
        Optional<Pose2d> targetOpt = m_targetSupplier.get();
        if (targetOpt.isEmpty()) {
            m_targetPose = null;
            SmartDashboard.putString("DTP/State", "NO_TAG");
            return;
        }
        m_targetPose = targetOpt.get();

        // ── Pause vision injection ────────────────────────────────────────────
        // From this point the robot navigates on pure wheel odometry + gyro.
        // No Kalman corrections will shift the perceived pose mid-maneuver.
        if (m_vision != null) m_vision.setVisionEnabled(false);

        Pose2d current = m_drivetrain.getState().Pose;
        var    speeds  = m_drivetrain.getState().Speeds;

        // ── Velocity-seeded controller reset ─────────────────────────────────
        // Handing the profiler the robot's current velocity causes it to build
        // a deceleration curve starting from that speed. A robot approaching at
        // 3 m/s will coast in smoothly rather than slamming to a stop.
        m_xController.reset(current.getX(),     speeds.vxMetersPerSecond);
        m_yController.reset(current.getY(),     speeds.vyMetersPerSecond);
        m_thetaController.reset(current.getRotation().getRadians(),
                                speeds.omegaRadiansPerSecond);

        // ── Pre-warm the pose input filters ──────────────────────────────────
        // Fill the moving-average history with the current reading so the first
        // execute() loop outputs the real position, not a partial average.
        m_xFilter.reset();
        m_yFilter.reset();
        for (int i = 0; i < 2; i++) {
            m_xFilter.calculate(current.getX());
            m_yFilter.calculate(current.getY());
        }

        SmartDashboard.putString("DTP/State", "RUNNING");
        Logger.recordOutput("DriveToPose/TargetPose", m_targetPose);
    }

    @Override
    public void execute() {
        if (m_targetPose == null) return;

        Pose2d current = m_drivetrain.getState().Pose;

        // ── Smooth pose readings before feeding into PID ──────────────────────
        // 2-tap moving average removes single-loop encoder quantisation spikes.
        // Without this, the D-term converts millimetre-scale noise into visible
        // motor output chatter even when the robot is stationary at the target.
        double smoothX = m_xFilter.calculate(current.getX());
        double smoothY = m_yFilter.calculate(current.getY());
        // Theta from the gyro is already noiseless — no filter needed.
        double theta   = current.getRotation().getRadians();

        // ── ProfiledPID outputs ───────────────────────────────────────────────
        double vxCmd    = MathUtil.clamp(
            m_xController.calculate(smoothX, m_targetPose.getX()),
            -DriveToPoseConstants.MAX_VEL_MPS, DriveToPoseConstants.MAX_VEL_MPS);

        double vyCmd    = MathUtil.clamp(
            m_yController.calculate(smoothY, m_targetPose.getY()),
            -DriveToPoseConstants.MAX_VEL_MPS, DriveToPoseConstants.MAX_VEL_MPS);

        double omegaCmd = MathUtil.clamp(
            m_thetaController.calculate(theta, m_targetPose.getRotation().getRadians()),
            -DriveToPoseConstants.MAX_OMEGA_RAD_S, DriveToPoseConstants.MAX_OMEGA_RAD_S);

        m_drivetrain.setControl(m_driveRequest
            .withVelocityX(vxCmd)
            .withVelocityY(vyCmd)
            .withRotationalRate(omegaCmd));

        // ── Debounced at-goal ─────────────────────────────────────────────────
        // All three axes must sit inside tolerance for DEBOUNCE_S continuously.
        // Any single axis straying out resets the timer — no false finishes.
        boolean atGoal = m_xController.atGoal()
                      && m_yController.atGoal()
                      && m_thetaController.atGoal();

        if (atGoal) {
            if (Double.isNaN(m_atGoalStartS)) m_atGoalStartS = Timer.getFPGATimestamp();
        } else {
            m_atGoalStartS = Double.NaN;
        }

        publishTelemetry(current, atGoal);
    }

    @Override
    public boolean isFinished() {
        if (m_targetPose == null)                                               return true;
        if (Timer.getFPGATimestamp() - m_startTimeS > DriveToPoseConstants.MAX_RUNTIME_S) return true;
        if (isDriverOverriding())                                               return true;
        return !Double.isNaN(m_atGoalStartS)
            && (Timer.getFPGATimestamp() - m_atGoalStartS) >= DriveToPoseConstants.DEBOUNCE_S;
    }

    @Override
    public void end(boolean interrupted) {
        // Always restore vision — even if the command was interrupted or timed out.
        if (m_vision != null) m_vision.setVisionEnabled(true);

        m_drivetrain.setControl(new SwerveRequest.Idle());
        SmartDashboard.putString("DTP/State", interrupted ? "INTERRUPTED" : "COMPLETE");
        Logger.recordOutput("DriveToPose/TargetPose", new Pose2d(-10, -10, new Rotation2d()));
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private boolean isDriverOverriding() {
        boolean overriding =
               Math.abs(m_driverX.getAsDouble())     > DriveToPoseConstants.OVERRIDE_DEADBAND
            || Math.abs(m_driverY.getAsDouble())     > DriveToPoseConstants.OVERRIDE_DEADBAND
            || Math.abs(m_driverOmega.getAsDouble()) > DriveToPoseConstants.OVERRIDE_DEADBAND;
        SmartDashboard.putBoolean("DTP/DriverOverride", overriding);
        return overriding;
    }

    private void publishTelemetry(Pose2d current, boolean atGoal) {
        double xErr        = m_targetPose.getX() - current.getX();
        double yErr        = m_targetPose.getY() - current.getY();
        double thetaErrRad = MathUtil.angleModulus(
            m_targetPose.getRotation().getRadians() - current.getRotation().getRadians());

        SmartDashboard.putNumber ("DTP/XError_m",       xErr);
        SmartDashboard.putNumber ("DTP/YError_m",       yErr);
        SmartDashboard.putNumber ("DTP/ThetaError_deg", Units.radiansToDegrees(thetaErrRad));
        SmartDashboard.putNumber ("DTP/DistToTarget_m", Math.hypot(xErr, yErr));
        SmartDashboard.putBoolean("DTP/AtGoal",         atGoal);
        SmartDashboard.putNumber ("DTP/HoldTime_s",
            Double.isNaN(m_atGoalStartS) ? 0.0 : Timer.getFPGATimestamp() - m_atGoalStartS);

        Logger.recordOutput("DriveToPose/XError_m",       xErr);
        Logger.recordOutput("DriveToPose/YError_m",       yErr);
        Logger.recordOutput("DriveToPose/ThetaError_deg", Units.radiansToDegrees(thetaErrRad));
        Logger.recordOutput("DriveToPose/AtGoal",         atGoal);
    }
}
