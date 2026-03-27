package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.util.ShiftStateTracker;

/**
 * Rotates the whole robot to face an align tag (same logic as AlignToTag),
 * interpolates flywheel RPM from the camera-measured distance, and fires.
 *
 * <h3>Sequence</h3>
 * <ol>
 *   <li><b>AIMING</b> — Robot rotates via PID on the best visible align-tag yaw.
 *       Flywheel spins up to the RPM interpolated from the tag distance.</li>
 *   <li><b>SETTLING</b> — Waits {@value #SETTLE_S} s once aimed + RPM on target.</li>
 *   <li><b>FIRING</b> — Feeder runs for {@value #FEED_S} s, then resets to AIMING.</li>
 * </ol>
 */
public class SnapAimAndShootCommand extends Command {

    // ── Tunable constants ─────────────────────────────────────────────────────

    /** Robot heading error (degrees) considered "aimed". */
    private static final double AIM_TOLERANCE_DEG = 1.5;

    /** Seconds robot + RPM must both be on target before the feeder fires. */
    private static final double SETTLE_S = 0.20;

    /** Seconds to run the feeder per ball. */
    private static final double FEED_S = 0.28;

    /** Feeder duty cycle while feeding. */
    private static final double FEED_DUTY = 0.65;

    // ── State machine ─────────────────────────────────────────────────────────

    private enum Phase { AIMING, SETTLING, FIRING }

    // ── Dependencies ─────────────────────────────────────────────────────────

    private final CommandSwerveDrivetrain m_drivetrain;
    private final ShooterSubsystem        m_shooter;
    private final FeederSubsystem         m_feeder;
    private final VisionSubsystem         m_vision;

    // ── Rotation PID — identical to AlignToTag ────────────────────────────────

    private final PIDController m_rotPID = new PIDController(
        VisionConstants.kP, VisionConstants.kI, VisionConstants.kD);

    private final SwerveRequest.FieldCentric m_driveRequest =
        new SwerveRequest.FieldCentric();

    // ── Runtime state ─────────────────────────────────────────────────────────

    private Phase  m_phase;
    private double m_phaseStartS;
    private double m_snappedDistM = -1.0; // distance captured once at initialize()

    // ── Constructor ───────────────────────────────────────────────────────────

    public SnapAimAndShootCommand(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            FeederSubsystem feeder,
            VisionSubsystem vision) {
        m_drivetrain = drivetrain;
        m_shooter    = shooter;
        m_feeder     = feeder;
        m_vision     = vision;
        m_rotPID.setTolerance(AIM_TOLERANCE_DEG);
        // feeder intentionally NOT in requirements — other feeder commands (B button,
        // right bumper) must not cancel alignment + RPM spin-up when they run.
        addRequirements(drivetrain, shooter);
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_rotPID.reset();
        m_phase       = Phase.AIMING;
        m_phaseStartS = Timer.getFPGATimestamp();

        // Snapshot distance once — use hub tag pitch if visible, else odometry fallback
        double hubDist = m_vision.getHubDistanceMeters();
        if (hubDist > 0.0) {
            m_snappedDistM = hubDist;
        } else {
            var pose = m_drivetrain.getState().Pose;
            double dx = ShooterConstants.HUB_CENTER.getX() - pose.getX();
            double dy = ShooterConstants.HUB_CENTER.getY() - pose.getY();
            m_snappedDistM = Math.sqrt(dx * dx + dy * dy) - ShooterConstants.SHOOTER_X_OFFSET_METERS;
        }
        m_shooter.setFlywheelRPMFromDistance(m_snappedDistM);

        SmartDashboard.putString ("SnapShot/Phase",        "AIMING");
        SmartDashboard.putBoolean("SnapShot/Active",       true);
        SmartDashboard.putBoolean("SnapShot/TagVisible",   false);
        SmartDashboard.putBoolean("SnapShot/RobotAimed",   false);
        SmartDashboard.putBoolean("SnapShot/ShooterReady", false);
        SmartDashboard.putNumber ("SnapShot/SnappedDist_m", m_snappedDistM);
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        // ── 1. Find best align tag (same filter as AlignToTag) ────────────────
        var target      = m_vision.getBestTarget();
        boolean tagVisible = target != null && AlignToTag.isAlignTag(target.getFiducialId());

        // ── 2. Rotate robot — identical logic to AlignToTag.execute() ─────────
        double rotationSpeed = 0.0;
        double tagYawDeg     = 0.0;

        if (tagVisible) {
            tagYawDeg     = target.getYaw();
            rotationSpeed = Math.max(-3.0, Math.min(3.0,
                m_rotPID.calculate(tagYawDeg, 0.0)));

            double omega = m_drivetrain.getState().Speeds.omegaRadiansPerSecond;
            if (Math.abs(omega) > 2.0) rotationSpeed *= 0.5;
        }

        m_drivetrain.setControl(
            m_driveRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(rotationSpeed)
                .withDeadband(0.02));

        // ── 3. RPM already set from snapped distance in initialize() — no update ──
        double distPhysM = m_snappedDistM;

        // ── 4. State machine ──────────────────────────────────────────────────
        boolean robotAimed   = tagVisible && Math.abs(tagYawDeg) < AIM_TOLERANCE_DEG;
        boolean shooterReady = m_shooter.isReadyToShoot();

        switch (m_phase) {

            case AIMING -> {
                if (robotAimed && shooterReady) {
                    m_phase       = Phase.SETTLING;
                    m_phaseStartS = now;
                }
            }

            case SETTLING -> {
                if (!robotAimed || !shooterReady) {
                    m_phase       = Phase.AIMING;
                    m_phaseStartS = now;
                } else if (ShiftStateTracker.isMyHubActive()
                        && (now - m_phaseStartS) >= SETTLE_S) {
                    m_phase       = Phase.FIRING;
                    m_phaseStartS = now;
                    m_feeder.setPower(FEED_DUTY);
                }
            }

            case FIRING -> {
                if ((now - m_phaseStartS) >= FEED_S) {
                    m_feeder.stop();
                    m_phase       = Phase.AIMING;
                    m_phaseStartS = now;
                }
            }

        }

        // ── Telemetry ─────────────────────────────────────────────────────────
        SmartDashboard.putString ("SnapShot/Phase",        m_phase.name());
        SmartDashboard.putBoolean("SnapShot/TagVisible",   tagVisible);
        SmartDashboard.putBoolean("SnapShot/RobotAimed",   robotAimed);
        SmartDashboard.putBoolean("SnapShot/ShooterReady", shooterReady);
        SmartDashboard.putBoolean("SnapShot/HubActive",    ShiftStateTracker.isMyHubActive());
        SmartDashboard.putNumber ("SnapShot/CurrentRPM",   m_shooter.getCurrentRPM());
        SmartDashboard.putNumber ("SnapShot/TagYaw_deg",   tagYawDeg);
        SmartDashboard.putNumber ("SnapShot/DistPhys_m",   distPhysM);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        m_shooter.idleFlywheel();
        m_drivetrain.setControl(new SwerveRequest.Idle());
        SmartDashboard.putString ("SnapShot/Phase",  interrupted ? "INTERRUPTED" : "COMPLETE");
        SmartDashboard.putBoolean("SnapShot/Active", false);
    }
}
