package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * TurnToAngle — rotates the robot to an absolute or relative heading in place.
 *
 * Translation is zero throughout; only the heading is controlled.
 * Uses a ProfiledPIDController + FieldCentric request (same pattern as SnapHeadingToTag).
 */
public class TurnToAngle extends Command {

    private static final double TIMEOUT_SECONDS       = 3.0;
    private static final double TOLERANCE_DEG         = 1.0;
    private static final double MAX_OMEGA_RAD_S       = Math.toRadians(360);
    private static final double MAX_ALPHA_RAD_S2      = Math.toRadians(720);

    private final CommandSwerveDrivetrain     m_swerve;
    private final double                      m_goalDeg;
    private final boolean                     m_isRelative;

    private final ProfiledPIDController       m_thetaController;
    private final SwerveRequest.FieldCentric  m_driveRequest = new SwerveRequest.FieldCentric();

    private Rotation2d m_targetRotation;
    private double     m_startTimeS;

    /**
     * @param swerve     Swerve subsystem.
     * @param angle      Target angle in degrees.
     * @param isRelative {@code true} = relative to current heading;
     *                   {@code false} = absolute field heading.
     */
    public TurnToAngle(CommandSwerveDrivetrain swerve, double angle, boolean isRelative) {
        m_swerve     = swerve;
        m_goalDeg    = angle;
        m_isRelative = isRelative;
        addRequirements(swerve);

        m_thetaController = new ProfiledPIDController(
            4.0, 0.0, 0.0,
            new Constraints(MAX_OMEGA_RAD_S, MAX_ALPHA_RAD_S2));
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_thetaController.setTolerance(Units.degreesToRadians(TOLERANCE_DEG));
    }

    @Override
    public void initialize() {
        Pose2d startPose = m_swerve.getState().Pose;
        m_startTimeS = Timer.getFPGATimestamp();

        m_targetRotation = m_isRelative
            ? startPose.getRotation().rotateBy(Rotation2d.fromDegrees(m_goalDeg))
            : Rotation2d.fromDegrees(m_goalDeg);

        // Seed controller from current state for a smooth, jerk-free start
        m_thetaController.reset(
            startPose.getRotation().getRadians(),
            m_swerve.getState().Speeds.omegaRadiansPerSecond);

        SmartDashboard.putNumber("TurnToAngle/TargetDeg", m_targetRotation.getDegrees());
    }

    @Override
    public void execute() {
        double currentRad = m_swerve.getState().Pose.getRotation().getRadians();
        double targetRad  = m_targetRotation.getRadians();

        double omegaCmd = MathUtil.clamp(
            m_thetaController.calculate(currentRad, targetRad),
            -MAX_OMEGA_RAD_S,
             MAX_OMEGA_RAD_S);

        // Zero translation — rotation only
        m_swerve.setControl(m_driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(omegaCmd));

        double errorDeg = MathUtil.inputModulus(
            Math.toDegrees(targetRad - currentRad), -180.0, 180.0);
        SmartDashboard.putNumber ("TurnToAngle/HeadingErrorDeg", errorDeg);
        SmartDashboard.putBoolean("TurnToAngle/AtGoal",          m_thetaController.atGoal());
        SmartDashboard.putNumber ("TurnToAngle/ElapsedS",        Timer.getFPGATimestamp() - m_startTimeS);
    }

    @Override
    public boolean isFinished() {
        boolean timeout = (Timer.getFPGATimestamp() - m_startTimeS) >= TIMEOUT_SECONDS;
        double errorDeg = Math.abs(MathUtil.inputModulus(
            m_targetRotation.getDegrees() - m_swerve.getState().Pose.getRotation().getDegrees(),
            -180.0, 180.0));
        return errorDeg < TOLERANCE_DEG || timeout;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.setControl(new SwerveRequest.Idle());
        SmartDashboard.putBoolean("TurnToAngle/Interrupted", interrupted);
    }
}