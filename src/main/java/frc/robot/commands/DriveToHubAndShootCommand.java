package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.VelocityCompensator;
import frc.robot.subsystems.Vision.VisionSubsystem;

/**
 * DriveToHubAndShootCommand — full shoot-on-the-move pipeline.
 *
 * <h3>Per-tick pipeline (every 20 ms)</h3>
 * <ol>
 *   <li>Fetch fused robot pose + chassis speeds from the Kalman pose estimator.</li>
 *   <li>{@link VelocityCompensator} → virtual-hub heading + effective shot distance.</li>
 *   <li>{@link ShotCalculator#calculate} → optimal launch angle + RPM (physics-based,
 *       exact formula from frc_v0_calculator.html).</li>
 *   <li>Command shooter hardware to those setpoints.</li>
 *   <li>{@link HubAlignController} → drive toward standoff + rotate to aim heading.</li>
 *   <li>{@link ReadyToFireCondition} → fire only when all five gates pass.</li>
 *   <li>End after one shot, or when the driver releases the button.</li>
 * </ol>
 */
public class DriveToHubAndShootCommand extends Command {

    // ── Shooter interface ─────────────────────────────────────────────────────

    public interface IShooterSubsystem {
        void      setLaunchAngleDeg(double degrees);
        void      setFlywheelRPM(double rpm);
        void      idleFlywheel();
        void      shoot();
        boolean   isAtTargetRPM(double targetRPM);
        boolean   isAtTargetAngle(double targetDeg);
        SubsystemBase asSubsystem();
    }

    // ── Dependencies ──────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain m_drivetrain;
    private final VisionSubsystem         m_vision;
    private final IShooterSubsystem       m_shooter;

    // ── Controllers ───────────────────────────────────────────────────────────
    private final HubAlignController m_alignCtrl = new HubAlignController();

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean m_shotFired        = false;
    private double  m_currentTargetRPM = ShooterConstants.IDLE_RPM;
    private double  m_currentHoodDeg   = 45.0;

    private ReadyToFireCondition m_fireGate;

    private static final Translation2d HUB = ShooterConstants.HUB_CENTER;

    // ── Constructor ───────────────────────────────────────────────────────────

    public DriveToHubAndShootCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem         vision,
            IShooterSubsystem       shooter) {
        m_drivetrain = drivetrain;
        m_vision     = vision;
        m_shooter    = shooter;
        addRequirements(drivetrain, shooter.asSubsystem());
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_shotFired = false;
        m_alignCtrl.reset();
        m_fireGate = new ReadyToFireCondition(
            (frc.robot.subsystems.Shooter.ShooterSubsystem) m_shooter,
            m_vision, m_drivetrain, m_alignCtrl);
        m_fireGate.reset();
        m_shooter.setFlywheelRPM(ShooterConstants.IDLE_RPM);
    }

    @Override
    public void execute() {
        // 1. Pose + velocity ───────────────────────────────────────────────────
        var robotPose = m_drivetrain.getState().Pose;
        var speeds    = m_drivetrain.getState().Speeds;

        // 2. Velocity compensation (look-ahead for ball ToF) ───────────────────
        var comp = VelocityCompensator.getInstance().compensate(
            robotPose.getTranslation(), HUB,
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond);

        // 3. Physics-based optimal shot parameters ─────────────────────────────
        // ShotCalculator sweeps all angles and picks the highest-scoring one
        // using the exact formula from frc_v0_calculator.html
        ShotCalculator.ShotResult params =
            ShotCalculator.calculate(comp.virtualDistanceMeters());

        m_currentTargetRPM = params.rpm();
        m_currentHoodDeg   = params.hoodDeg();

        // 4. Command shooter ───────────────────────────────────────────────────
        m_shooter.setFlywheelRPM(m_currentTargetRPM);
        m_shooter.setLaunchAngleDeg(m_currentHoodDeg);

        // 5. Drive alignment ───────────────────────────────────────────────────
        HubAlignController.DriveOutput driveOut =
            m_alignCtrl.compute(m_drivetrain, HUB, ShooterConstants.DESIRED_DISTANCE_METERS);

        m_drivetrain.setControl(
            new SwerveRequest.FieldCentric()
                .withVelocityX(driveOut.vxMps())
                .withVelocityY(driveOut.vyMps())
                .withRotationalRate(driveOut.omegaRadS()));

        // 6. Fire gate ─────────────────────────────────────────────────────────
        if (!m_shotFired && m_fireGate.isReady(m_currentTargetRPM, m_currentHoodDeg)) {
            m_shooter.shoot();
            m_shotFired = true;
        }

        // 7. Telemetry ─────────────────────────────────────────────────────────
        SmartDashboard.putNumber ("DTHS/TargetRPM",        m_currentTargetRPM);
        SmartDashboard.putNumber ("DTHS/HoodDeg",          m_currentHoodDeg);
        SmartDashboard.putNumber ("DTHS/V0_mps",           params.v0());
        SmartDashboard.putNumber ("DTHS/EntryAngle_deg",   params.entryAngle());
        SmartDashboard.putNumber ("DTHS/FlightTime_s",     params.flightTime());
        SmartDashboard.putNumber ("DTHS/VirtDist_m",       comp.virtualDistanceMeters());
        SmartDashboard.putNumber ("DTHS/AimHeading_deg",   comp.aimHeadingDeg());
        SmartDashboard.putBoolean("DTHS/ShotFired",        m_shotFired);
    }

    @Override
    public boolean isFinished() { return m_shotFired; }

    @Override
    public void end(boolean interrupted) {
        m_shooter.idleFlywheel();
        m_drivetrain.setControl(new SwerveRequest.Idle());
        SmartDashboard.putBoolean("DTHS/ShotFired", false);
    }
}