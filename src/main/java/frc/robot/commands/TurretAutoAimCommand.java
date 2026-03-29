package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.FireControlSolver;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

/**
 * Continuously tracks the velocity-compensated hub aim angle using FireControlSolver,
 * with AprilTag vision correction when a hub tag is visible.
 *
 * <h3>Dual-source aim</h3>
 * <ol>
 *   <li><b>Odometry (primary)</b> — FireControlSolver runs a Newton-method SOTM solve
 *       every 20 ms using the Kalman-fused robot pose. Handles velocity compensation
 *       and angular-velocity feedforward.</li>
 *   <li><b>Vision (correction)</b> — When a hub AprilTag is visible and fresh (&lt;200 ms),
 *       the camera-relative yaw is converted to a robot-frame bearing using the camera's
 *       known mount angle. The final turret target is a weighted blend:<br>
 *       {@code target = odomTarget + VISION_WEIGHT × (visionTarget − odomTarget)}<br>
 *       VISION_WEIGHT = {@value #VISION_WEIGHT} (0 = pure odometry, 1 = pure vision).
 *       The shortest angular path is always used to avoid wrapping artifacts.</li>
 * </ol>
 *
 * <h3>Tuning note — sign convention</h3>
 * If the vision correction pulls the turret the wrong direction, negate the sign of
 * {@code visionRobotYaw} in {@code execute()} (change {@code −} to {@code +} in
 * {@code cameraMountYaw − tagYaw}).  This depends on PhotonVision's yaw sign convention
 * for your specific camera and mount orientation.
 *
 * <h3>Toggle</h3>
 * Bind with {@code toggleOnTrue}. Pressing the button once enables auto-aim; pressing
 * again restores the default (manual right-joystick) command.
 */
public class TurretAutoAimCommand extends Command {

    /**
     * Fraction of the vision-derived correction applied to the odometry target.
     * 0.0 = pure odometry, 1.0 = pure vision. 0.6 gives strong tag correction
     * while retaining SOTM velocity compensation from the solver.
     * Tune on field — reduce if the turret oscillates when the tag is partially
     * occluded; increase if odometry drift is a persistent problem.
     */
    private static final double VISION_WEIGHT = 0.6;

    private final CommandSwerveDrivetrain m_drivetrain;
    private final ShooterSubsystem        m_shooter;
    private final TurretSubsystem         m_turret;
    private final VisionSubsystem         m_vision;

    // Cache of last valid solver result — used as stale fallback when solver is INVALID
    private Rotation2d m_lastAimAngle     = new Rotation2d();
    private double     m_lastAngularVelFF = 0.0;
    private boolean    m_hasValidResult   = false;

    public TurretAutoAimCommand(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            TurretSubsystem turret,
            VisionSubsystem vision) {
        m_drivetrain = drivetrain;
        m_shooter    = shooter;
        m_turret     = turret;
        m_vision     = vision;
        // Only requires the turret — all other subsystems are read-only.
        addRequirements(turret);
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_shooter.getFireControlSolver().resetWarmStart();
        m_hasValidResult   = false;
        m_lastAimAngle     = new Rotation2d();
        m_lastAngularVelFF = 0.0;
        SmartDashboard.putBoolean("TurretAutoAim/Active",      true);
        SmartDashboard.putBoolean("TurretAutoAim/SolverValid", false);
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        ChassisSpeeds rawSpeeds = m_drivetrain.getState().Speeds;

        // Robot chassis speeds in field-relative frame
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(
            rawSpeeds.vxMetersPerSecond,
            rawSpeeds.vyMetersPerSecond,
            rawSpeeds.omegaRadiansPerSecond);

        Translation2d hubCenter = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? ShooterConstants.HUB_CENTER_RED
            : ShooterConstants.HUB_CENTER_BLUE;

        FireControlSolver.ShotInputs inputs = new FireControlSolver.ShotInputs(
            robotPose,
            fieldSpeeds,
            fieldSpeeds,
            hubCenter,
            new Translation2d(0, 0), // disable behind-hub check
            1.0);                    // full vision confidence

        FireControlSolver.LaunchParameters result =
            m_shooter.getFireControlSolver().calculate(inputs);

        // ── Determine odometry aim angle and velocity FF ──────────────────────
        Rotation2d aimAngle;
        double     angularVelFF;

        if (result.isValid()) {
            aimAngle           = result.driveAngle();
            angularVelFF       = result.driveAngularVelocityRadPerSec();
            m_lastAimAngle     = aimAngle;
            m_lastAngularVelFF = angularVelFF;
            m_hasValidResult   = true;
        } else if (m_hasValidResult) {
            aimAngle     = m_lastAimAngle;
            angularVelFF = m_lastAngularVelFF;
        } else {
            // No valid solve yet — geometric aim at hub, no SOTM correction
            double dx = hubCenter.getX() - robotPose.getX();
            double dy = hubCenter.getY() - robotPose.getY();
            aimAngle     = new Rotation2d(dx, dy);
            angularVelFF = 0.0;
        }

        // ── Convert field-frame aim to robot-frame turret angle ──────────────
        double robotHeadingDeg = robotPose.getRotation().getDegrees();
        double fieldAimDeg     = aimAngle.getDegrees();
        double odomTurretDeg   = MathUtil.inputModulus(
            fieldAimDeg - robotHeadingDeg, -180.0, 180.0);

        // ── Vision correction ─────────────────────────────────────────────────
        // When a hub AprilTag is fresh, blend the camera-derived robot-frame bearing
        // into the odometry target along the shortest angular path.
        double turretTargetDeg = odomTurretDeg;
        double visionRobotYaw  = Double.NaN;
        boolean visionActive   = false;

        double hubCenterYaw = m_vision.getHubCenterYawDeg();
        if (!Double.isNaN(hubCenterYaw)) {
            // Use hub CENTER yaw (not raw tag face yaw) so the blend targets the
            // scoring opening rather than the AprilTag surface.
            visionRobotYaw = MathUtil.inputModulus(hubCenterYaw, -180.0, 180.0);

            // Blend along the shortest arc to avoid wrap-around jumps
            double diff = MathUtil.inputModulus(
                visionRobotYaw - odomTurretDeg, -180.0, 180.0);
            turretTargetDeg = MathUtil.inputModulus(
                odomTurretDeg + VISION_WEIGHT * diff, -180.0, 180.0);

            visionActive = true;
        }

        // ── Angular velocity feedforward ─────────────────────────────────────
        double robotOmega    = rawSpeeds.omegaRadiansPerSecond;
        double turretFF_radS = angularVelFF - robotOmega;

        // ── Command turret ────────────────────────────────────────────────────
        m_turret.setAngleDegWithFF(turretTargetDeg, turretFF_radS);

        // ── Telemetry ─────────────────────────────────────────────────────────
        SmartDashboard.putBoolean("TurretAutoAim/SolverValid",     result.isValid());
        SmartDashboard.putBoolean("TurretAutoAim/VisionActive",    visionActive);
        SmartDashboard.putNumber ("TurretAutoAim/FieldAimDeg",     fieldAimDeg);
        SmartDashboard.putNumber ("TurretAutoAim/RobotHeadingDeg", robotHeadingDeg);
        SmartDashboard.putNumber ("TurretAutoAim/OdomTurretDeg",   odomTurretDeg);
        SmartDashboard.putNumber ("TurretAutoAim/VisionRobotYaw",  Double.isNaN(visionRobotYaw) ? -999 : visionRobotYaw);
        SmartDashboard.putNumber ("TurretAutoAim/TurretTargetDeg", turretTargetDeg);
        SmartDashboard.putNumber ("TurretAutoAim/TurretActualDeg", m_turret.getAngleDeg());
        SmartDashboard.putNumber ("TurretAutoAim/TurretErrorDeg",
            turretTargetDeg - m_turret.getAngleDeg());
        SmartDashboard.putNumber ("TurretAutoAim/TurretFF_radS",   turretFF_radS);
        SmartDashboard.putNumber ("TurretAutoAim/RobotOmega_radS", robotOmega);
        SmartDashboard.putNumber ("TurretAutoAim/SolverFF_radS",   angularVelFF);
        SmartDashboard.putBoolean("TurretAutoAim/AtTarget",        m_turret.isAtTarget());
        SmartDashboard.putNumber ("TurretAutoAim/FC/Confidence",
            result.isValid() ? result.confidence() : -1.0);
    }

    @Override
    public boolean isFinished() {
        return false; // only ends when the toggle button is pressed again
    }

    @Override
    public void end(boolean interrupted) {
        // Hold current angle — smooth handoff back to joystick default command
        m_turret.setAngleDeg(m_turret.getAngleDeg());
        SmartDashboard.putBoolean("TurretAutoAim/Active",     false);
        SmartDashboard.putBoolean("TurretAutoAim/VisionActive", false);
    }
}
