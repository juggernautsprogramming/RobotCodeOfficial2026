package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * TurnToAngle — rotates the robot to an absolute or relative heading.
 *
 * <h3>Fixes applied</h3>
 * <ul>
 *   <li>Removed the erroneous {@code resetPose()} call from {@code initialize()} that was
 *       zeroing the robot's heading and corrupting odometry before computing the target.</li>
 *   <li>Added {@code enableContinuousInput(-Math.PI, Math.PI)} on the theta controller so
 *       the profiled PID always takes the shortest arc through zero (prevents 350° wrong-way
 *       spins).</li>
 *   <li>Tightened X/Y tolerance from 1 m → 0.05 m so {@code atReference()} triggers
 *       reliably on rotation alone without needing a large translational error.</li>
 *   <li>Added a 3-second safety timeout in {@code isFinished()} to prevent infinite spin if
 *       the controller never converges (e.g. blocked wheel, bad gains).</li>
 *   <li>Added SmartDashboard telemetry for heading error and timeout.</li>
 * </ul>
 */
public class TurnToAngle extends Command {

    // ── Safety timeout ─────────────────────────────────────────────────────────
    private static final double TIMEOUT_SECONDS = 3.0;

    // ── Dependencies ───────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain m_swerve;

    // ── Config ─────────────────────────────────────────────────────────────────
    private final double  m_goalDeg;
    private final boolean m_isRelative;

    // ── Controller ─────────────────────────────────────────────────────────────
    private final HolonomicDriveController m_controller;

    // ── State ──────────────────────────────────────────────────────────────────
    private Pose2d m_targetPose  = new Pose2d();
    private double m_startTimeS  = 0.0;

    /**
     * @param swerve     Swerve subsystem.
     * @param angle      Target angle in degrees.
     * @param isRelative {@code true} = relative to current heading;
     *                   {@code false} = absolute field heading.
     */
    public TurnToAngle(CommandSwerveDrivetrain swerve, double angle, boolean isRelative) {
        this.m_swerve     = swerve;
        this.m_goalDeg    = angle;
        this.m_isRelative = isRelative;
        addRequirements(swerve);

        PIDController xController = new PIDController(1.0, 0.0, 0.0);
        PIDController yController = new PIDController(1.0, 0.0, 0.0);

        ProfiledPIDController thetaController = new ProfiledPIDController(
            4.0, 0.0, 0.1,
            new Constraints(
                Math.toRadians(360),  // max angular velocity  (rad/s)
                Math.toRadians(720)   // max angular accel     (rad/s²)
            ));
        // CRITICAL: without this the PID spins the wrong way across the ±180° seam
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        m_controller = new HolonomicDriveController(xController, yController, thetaController);

        // Tight X/Y tolerance so isFinished() is driven purely by rotation error
        m_controller.setTolerance(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(1.0)));
    }

    // ── Lifecycle ──────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        // Do NOT call resetPose() here — that wipes odometry and gives a wrong startPos.
        Pose2d startPose = m_swerve.getState().Pose;
        m_startTimeS     = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        Rotation2d targetRotation = m_isRelative
            ? startPose.getRotation().rotateBy(Rotation2d.fromDegrees(m_goalDeg))
            : Rotation2d.fromDegrees(m_goalDeg);

        m_targetPose = new Pose2d(startPose.getTranslation(), targetRotation);

        SmartDashboard.putNumber("TurnToAngle/TargetDeg", targetRotation.getDegrees());
    }

    @Override
    public void execute() {
        Pose2d        currPose     = m_swerve.getState().Pose;
        ChassisSpeeds speeds       = m_controller.calculate(
            currPose, m_targetPose, 0.0, m_targetPose.getRotation());

        m_swerve.setControl(new SwerveRequest.RobotCentric()
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond));

        // Telemetry
        double errorDeg = m_targetPose.getRotation().getDegrees()
                        - currPose.getRotation().getDegrees();
        // Wrap to [-180, 180]
        errorDeg = ((errorDeg + 180.0) % 360.0) - 180.0;
        SmartDashboard.putNumber ("TurnToAngle/HeadingErrorDeg", errorDeg);
        SmartDashboard.putBoolean("TurnToAngle/AtReference",     m_controller.atReference());
        SmartDashboard.putNumber ("TurnToAngle/ElapsedS",
            edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - m_startTimeS);
    }

    @Override
    public boolean isFinished() {
        boolean timeout = (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - m_startTimeS)
                          >= TIMEOUT_SECONDS;
        return m_controller.atReference() || timeout;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.setControl(new SwerveRequest.RobotCentric()
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0));
        SmartDashboard.putBoolean("TurnToAngle/Interrupted", interrupted);
    }
}