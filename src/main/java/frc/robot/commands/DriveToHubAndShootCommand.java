package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.util.HubAlignController;
import frc.robot.util.HubAlignController.AlignOutput;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotCalculator.ShotResult;

/**
 * Full auto-align + shoot pipeline.
 *
 * Pipeline:
 *  1. Drive toward the hub using the fused odometry pose ONLY.
 *     Raw camera angles never drive motors — they enter only through the
 *     Kalman filter in VisionSubsystem.periodic().
 *  2. Continuously recalculate optimal launch angle + RPM via ShotCalculator
 *     (same physics model as frc_v0_calculator.html).
 *  3. Pre-spin flywheel and set pivot angle while approaching.
 *  4. Fire exactly once when: distance OK AND heading OK AND flywheel at RPM
 *     AND pivot at angle.
 *
 * Bind in RobotContainer:
 *   driverStick.b().whileTrue(
 *       new DriveToHubAndShootCommand(drivetrain, visionSubsystem, shooterSubsystem));
 *
 * NOTE: File must be named exactly DriveToHubAndShootCommand.java (case-sensitive).
 */
public class DriveToHubAndShootCommand extends Command {

    // ── Dependencies ──────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain m_drivetrain;
    @SuppressWarnings("unused")
    private final VisionSubsystem         m_vision;    // future: multi-shot sequencing
    private final IShooterSubsystem       m_shooter;

    // ── Controllers ───────────────────────────────────────────────────────────
    private final HubAlignController m_align = new HubAlignController();

    private final SwerveRequest.FieldCentric m_driveRequest =
        new SwerveRequest.FieldCentric()
            .withDeadband(0.0)
            .withRotationalDeadband(0.0);

    // ── Per-loop state ────────────────────────────────────────────────────────
    private ShotResult m_shotResult   = null;
    private boolean    m_hasFired     = false;
    private double     m_lastLoopTime = 0.0;

    // ── Constructor ───────────────────────────────────────────────────────────

    public DriveToHubAndShootCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            IShooterSubsystem shooter) {
        this.m_drivetrain = drivetrain;
        this.m_vision     = vision;
        this.m_shooter    = shooter;

        // VisionSubsystem runs its own periodic() — no requirement needed here
        addRequirements(drivetrain, shooter.asSubsystem());
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_align.reset();
        m_hasFired     = false;
        m_shotResult   = null;
        m_lastLoopTime = Timer.getFPGATimestamp();

        m_drivetrain.setControl(
            m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        double dt  = now - m_lastLoopTime;
        m_lastLoopTime = now;
        if (dt <= 0.0 || dt > 0.10) dt = 0.020; // guard against timer edge cases

        // 1. Get fused pose — the ONLY pose used for all drive math
        Pose2d pose = m_drivetrain.getState().Pose;

        // 2. Compute drive/strafe/rotation commands from odometry (no camera jitter)
        AlignOutput align = m_align.update(pose, dt);

        // 3. Compute horizontal distance from shooter exit to hub
        double distToHub   = pose.getTranslation().getDistance(ShooterConstants.HUB_CENTER);
        double shooterDist = Math.max(0.1, distToHub - ShooterConstants.SHOOTER_X_OFFSET_METERS);

        // 4. Find optimal launch angle + RPM for current distance
        m_shotResult = ShotCalculator.calculate(shooterDist);

        // 5. Pre-condition shooter while driving in
        if (m_shotResult != null && m_shotResult.isValid()) {
            m_shooter.setLaunchAngleDeg(m_shotResult.launchAngleDeg);
            m_shooter.setFlywheelRPM(m_shotResult.motorRPM);
        } else {
            m_shooter.idleFlywheel();
        }

        // 6. Send drive commands
        m_drivetrain.setControl(
            m_driveRequest
                .withVelocityX(align.xSpeedMps)
                .withVelocityY(align.ySpeedMps)
                .withRotationalRate(align.omegaRadS));

        // 7. Fire when all four gates pass simultaneously
        if (!m_hasFired
                && align.readyToShoot
                && m_shotResult != null
                && m_shotResult.isValid()
                && m_shooter.isAtTargetRPM(m_shotResult.motorRPM)
                && m_shooter.isAtTargetAngle(m_shotResult.launchAngleDeg)) {
            m_shooter.shoot();
            m_hasFired = true;
        }

        publishTelemetry(align, shooterDist);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(
            m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        m_shooter.idleFlywheel();
        SmartDashboard.putBoolean("AutoAlign/Active", false);
    }

    @Override
    public boolean isFinished() {
        return m_hasFired;
    }

    // ── Telemetry ─────────────────────────────────────────────────────────────

    private void publishTelemetry(AlignOutput align, double shooterDist) {
        SmartDashboard.putBoolean("AutoAlign/Active",          true);
        SmartDashboard.putBoolean("AutoAlign/Ready To Shoot",  align.readyToShoot);
        SmartDashboard.putNumber ("AutoAlign/Dist To Hub (m)", shooterDist);
        SmartDashboard.putNumber ("AutoAlign/Drive X (m/s)",   align.xSpeedMps);
        SmartDashboard.putNumber ("AutoAlign/Drive Y (m/s)",   align.ySpeedMps);
        SmartDashboard.putNumber ("AutoAlign/Omega (rad/s)",   align.omegaRadS);

        if (m_shotResult != null) {
            SmartDashboard.putNumber("AutoAlign/Shot Angle (deg)",       m_shotResult.launchAngleDeg);
            SmartDashboard.putNumber("AutoAlign/Shot V0 (m/s)",          m_shotResult.launchSpeedMps);
            SmartDashboard.putNumber("AutoAlign/Shot Motor RPM",         m_shotResult.motorRPM);
            SmartDashboard.putNumber("AutoAlign/Shot Entry Angle (deg)", m_shotResult.entryAngleDeg);
            SmartDashboard.putNumber("AutoAlign/Shot Flight Time (s)",   m_shotResult.flightTimeSec);
            SmartDashboard.putNumber("AutoAlign/Shot Score",             m_shotResult.score);
        }

        Pose2d target = m_align.getTargetPose();
        if (target != null) {
            SmartDashboard.putNumberArray("AutoAlign/Target Pose", new double[]{
                target.getX(),
                target.getY(),
                target.getRotation().getDegrees()
            });
        }
    }

    // ── IShooterSubsystem ─────────────────────────────────────────────────────

    /**
     * Interface implemented by ShooterSubsystem.
     * Decouples this command from the hardware implementation.
     */
    public interface IShooterSubsystem {
        /** Set pivot/hood to this angle (degrees above horizontal). */
        void setLaunchAngleDeg(double degrees);

        /** Spin flywheel to this motor RPM. */
        void setFlywheelRPM(double rpm);

        /** Drop flywheel to idle speed. */
        void idleFlywheel();

        /** Activate feed/indexer to fire the game piece. */
        void shoot();

        /** True when flywheel is within tolerance of targetRPM. */
        boolean isAtTargetRPM(double targetRPM);

        /** True when pivot is within tolerance of targetDeg. */
        boolean isAtTargetAngle(double targetDeg);

        /**
         * Return this as a WPILib Subsystem for addRequirements().
         * Implement as {@code return this;} in your SubsystemBase.
         */
        Subsystem asSubsystem();
    }
}