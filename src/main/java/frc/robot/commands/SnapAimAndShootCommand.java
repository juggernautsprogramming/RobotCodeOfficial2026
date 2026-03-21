package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Shooter.FireControlSolver;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

/**
 * Vision-triggered snap-aim-and-shoot command.
 *
 * <h3>Sequence</h3>
 * <ol>
 *   <li><b>AIMING</b> — Turret tracks the hub using FireControlSolver (SOTM) blended
 *       with the live AprilTag bearing from the turret camera. Flywheel spins up to
 *       the RPM interpolated from the camera-measured distance (or odometry if no tag
 *       is visible).</li>
 *   <li><b>SETTLING</b> — Once turret is on target, RPM is on target, AND a hub
 *       AprilTag is fresh, the command waits {@value #SETTLE_S} seconds to confirm
 *       the aim is stable before firing.</li>
 *   <li><b>FIRING</b> — Feeder runs at {@value #FEED_VOLTS} V for {@value #FEED_S}
 *       seconds while the flywheel holds target RPM.</li>
 *   <li><b>DONE</b> — Command ends naturally. Feeder stops, flywheel idles, turret
 *       holds its current angle.</li>
 * </ol>
 *
 * <h3>Abort behaviour</h3>
 * If the button is released before firing completes, the command is interrupted:
 * feeder stops, flywheel idles, turret holds last angle.
 *
 * <h3>Button binding</h3>
 * Use {@code whileTrue} so the sequence restarts on each press:
 * <pre>
 *   m_playerStick.leftStick().whileTrue(new SnapAimAndShootCommand(...));
 * </pre>
 *
 * <h3>Vision weight</h3>
 * The vision-blend weight ({@value #VISION_WEIGHT}) matches
 * {@link TurretAutoAimCommand}.  See that class for tuning notes.
 */
public class SnapAimAndShootCommand extends Command {

    // ── Tunable constants ─────────────────────────────────────────────────────

    /** Seconds turret + RPM must both be on target before the feeder fires. */
    private static final double SETTLE_S = 0.10;

    /** Seconds to run the feeder per ball. */
    private static final double FEED_S = 0.40;

    /** Feeder motor voltage while feeding. */
    private static final double FEED_VOLTS = 5.0;

    /** Vision blend weight — 0 = pure odometry aim, 1 = pure vision aim. */
    private static final double VISION_WEIGHT = 0.6;

    // ── State machine ─────────────────────────────────────────────────────────

    private enum Phase { AIMING, SETTLING, FIRING, DONE }

    // ── Dependencies ─────────────────────────────────────────────────────────

    private static final Translation2d HUB = ShooterConstants.HUB_CENTER;

    private final CommandSwerveDrivetrain m_drivetrain;
    private final ShooterSubsystem        m_shooter;
    private final TurretSubsystem         m_turret;
    private final FeederSubsystem         m_feeder;
    private final VisionSubsystem         m_vision;

    // ── Solver state ─────────────────────────────────────────────────────────

    private Rotation2d m_lastAimAngle     = new Rotation2d();
    private double     m_lastAngularVelFF = 0.0;
    private boolean    m_hasValidSolve    = false;

    // ── Runtime state ─────────────────────────────────────────────────────────

    private Phase  m_phase;
    private double m_phaseStartS;

    // ── Constructor ───────────────────────────────────────────────────────────

    public SnapAimAndShootCommand(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            TurretSubsystem turret,
            FeederSubsystem feeder,
            VisionSubsystem vision) {
        m_drivetrain = drivetrain;
        m_shooter    = shooter;
        m_turret     = turret;
        m_feeder     = feeder;
        m_vision     = vision;
        addRequirements(turret, shooter, feeder);
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_shooter.getFireControlSolver().resetWarmStart();
        m_hasValidSolve    = false;
        m_lastAimAngle     = new Rotation2d();
        m_lastAngularVelFF = 0.0;
        m_phase            = Phase.AIMING;
        m_phaseStartS      = Timer.getFPGATimestamp();
        // (no m_currentTargetRPM — uses isReadyToShoot() instead)

        SmartDashboard.putString ("SnapShot/Phase",        "AIMING");
        SmartDashboard.putBoolean("SnapShot/Active",       true);
        SmartDashboard.putBoolean("SnapShot/TagVisible",   false);
        SmartDashboard.putBoolean("SnapShot/TurretReady",  false);
        SmartDashboard.putBoolean("SnapShot/ShooterReady", false);
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        // ── 1. Compute turret aim angle (odometry + vision blend) ─────────────
        Pose2d robotPose = m_drivetrain.getState().Pose;
        ChassisSpeeds rawSpeeds = m_drivetrain.getState().Speeds;
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(
            rawSpeeds.vxMetersPerSecond,
            rawSpeeds.vyMetersPerSecond,
            rawSpeeds.omegaRadiansPerSecond);

        FireControlSolver.ShotInputs inputs = new FireControlSolver.ShotInputs(
            robotPose, fieldSpeeds, fieldSpeeds, HUB,
            new Translation2d(0, 0), 1.0);

        FireControlSolver.LaunchParameters result =
            m_shooter.getFireControlSolver().calculate(inputs);

        Rotation2d aimAngle;
        double angularVelFF;
        if (result.isValid()) {
            aimAngle           = result.driveAngle();
            angularVelFF       = result.driveAngularVelocityRadPerSec();
            m_lastAimAngle     = aimAngle;
            m_lastAngularVelFF = angularVelFF;
            m_hasValidSolve    = true;
        } else if (m_hasValidSolve) {
            aimAngle     = m_lastAimAngle;
            angularVelFF = m_lastAngularVelFF;
        } else {
            double dx = HUB.getX() - robotPose.getX();
            double dy = HUB.getY() - robotPose.getY();
            aimAngle     = new Rotation2d(dx, dy);
            angularVelFF = 0.0;
        }

        double robotHeadingDeg = robotPose.getRotation().getDegrees();
        double odomTurretDeg   = MathUtil.inputModulus(
            aimAngle.getDegrees() - robotHeadingDeg, -180.0, 180.0);

        // Vision correction blend (same as TurretAutoAimCommand)
        double turretTargetDeg = odomTurretDeg;
        if (m_vision.isHubTargetFresh()) {
            double visionRobotYaw = m_vision.getHubTagRobotFrameYawDeg();
            if (!Double.isNaN(visionRobotYaw)) {
                visionRobotYaw = MathUtil.inputModulus(visionRobotYaw, -180.0, 180.0);
                double diff = MathUtil.inputModulus(
                    visionRobotYaw - odomTurretDeg, -180.0, 180.0);
                turretTargetDeg = MathUtil.inputModulus(
                    odomTurretDeg + VISION_WEIGHT * diff, -180.0, 180.0);
            }
        }

        double turretFF = angularVelFF - rawSpeeds.omegaRadiansPerSecond;
        m_turret.setAngleDegWithFF(turretTargetDeg, turretFF);

        // ── 2. Compute and command RPM from distance ──────────────────────────
        double distPhysM;
        if (m_vision.isHubTargetFresh()) {
            // Camera-measured distance to hub (most accurate)
            distPhysM = m_vision.getHubDistanceMeters();
        } else {
            // Odometry fallback: robot-to-hub minus shooter offset
            double dx = HUB.getX() - robotPose.getX();
            double dy = HUB.getY() - robotPose.getY();
            distPhysM = Math.sqrt(dx * dx + dy * dy)
                - ShooterConstants.SHOOTER_X_OFFSET_METERS;
        }
        m_shooter.setFlywheelRPMFromDistance(distPhysM);

        // ── 3. State machine ──────────────────────────────────────────────────
        boolean tagFresh     = m_vision.isHubTargetFresh();
        boolean turretReady  = m_turret.isAtTarget();
        boolean shooterReady = m_shooter.isReadyToShoot();

        switch (m_phase) {

            case AIMING -> {
                if (tagFresh && turretReady && shooterReady) {
                    m_phase       = Phase.SETTLING;
                    m_phaseStartS = now;
                }
            }

            case SETTLING -> {
                // If aim drifts off target, reset back to AIMING
                if (!turretReady || !shooterReady) {
                    m_phase       = Phase.AIMING;
                    m_phaseStartS = now;
                } else if ((now - m_phaseStartS) >= SETTLE_S) {
                    m_phase       = Phase.FIRING;
                    m_phaseStartS = now;
                    m_feeder.setPower(FEED_VOLTS);
                }
            }

            case FIRING -> {
                // Keep flywheel at speed while feeder runs
                if ((now - m_phaseStartS) >= FEED_S) {
                    m_feeder.stop();
                    m_phase = Phase.DONE;
                }
            }

            case DONE -> { /* isFinished() handles exit */ }
        }

        // ── Telemetry ─────────────────────────────────────────────────────────
        SmartDashboard.putString ("SnapShot/Phase",        m_phase.name());
        SmartDashboard.putBoolean("SnapShot/TagVisible",   tagFresh);
        SmartDashboard.putBoolean("SnapShot/TurretReady",  turretReady);
        SmartDashboard.putBoolean("SnapShot/ShooterReady", shooterReady);
        SmartDashboard.putNumber ("SnapShot/CurrentRPM",   m_shooter.getCurrentRPM());
        SmartDashboard.putNumber ("SnapShot/TurretTarget", turretTargetDeg);
        SmartDashboard.putNumber ("SnapShot/TurretActual", m_turret.getAngleDeg());
        SmartDashboard.putNumber ("SnapShot/DistPhys_m",   distPhysM);
    }

    @Override
    public boolean isFinished() {
        return m_phase == Phase.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        m_shooter.idleFlywheel();
        m_turret.setAngleDeg(m_turret.getAngleDeg()); // hold angle on handoff
        SmartDashboard.putString ("SnapShot/Phase",  interrupted ? "INTERRUPTED" : "COMPLETE");
        SmartDashboard.putBoolean("SnapShot/Active", false);
    }
}
