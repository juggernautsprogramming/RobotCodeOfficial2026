package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.ShotCalculator;

/**
 * DriveToHubAndShootCommand — <b>ALIGNMENT TEST MODE</b> (shooting disabled).
 *
 * <p>This configuration exercises only the drive-alignment pipeline so the
 * drivetrain's rotational tracking can be validated before the full
 * shoot-on-the-move pipeline is re-enabled.
 *
 * <h3>Design sources</h3>
 * <ul>
 *   <li><b>6328 (Mechanical Advantage) pose fusion</b> — {@code VisionSubsystem}
 *       feeds PhotonPoseEstimator data into {@code SwerveDrivePoseEstimator} every
 *       loop, giving a Kalman-fused pose used by {@link HubAlignController}.</li>
 *   <li><b>2910 (Jack in the Bot) rotation control</b> — {@link HubAlignController}
 *       uses a {@code ProfiledPIDController} that generates a smooth trapezoidal
 *       rotation profile toward the hub heading. The driver retains full control of
 *       translation (strafe / forward) via the left joystick.</li>
 *   <li><b>1678 (Citrus) alignment gate</b> — {@link #isHubConfirmedActive()} checks
 *       {@code DriverStation.getGameSpecificMessage()} to confirm the hub is active
 *       before declaring alignment.  The FMS check is skipped during practice.</li>
 *   <li><b>frc_v0_calculator physics</b> — {@link ShotCalculator#OPTIMAL_STANDOFF_M}
 *       is precomputed at startup by sweeping distances and picking the one whose
 *       best angle yields the highest shot-quality score. The command drives toward
 *       this distance and displays exactly how close the driver is to the ideal spot.</li>
 * </ul>
 *
 * <h3>Dashboard keys (Shuffleboard / SmartDashboard)</h3>
 * <pre>
 *   DTHS/OptimalDist_m    — physics-computed ideal standoff (meters from hub)
 *   DTHS/OptimalAngle_deg — hood angle at that distance
 *   DTHS/OptimalRPM       — required flywheel RPM at that distance
 *   DTHS/OptimalEntry_deg — hub entry angle at that distance (&gt;35° = good)
 *   DTHS/CurrentDist_m    — robot's actual distance to hub (odometry)
 *   DTHS/DistError_m      — (currentDist − optimalDist), + means too far
 *   DTHS/RotAligned       — rotation error &lt; 1.5°
 *   DTHS/HubActive        — hub confirmed via game message (or no FMS)
 *   DTHS/Aligned          — rotAligned AND hubActive simultaneously
 *   DTHS/ElapsedS         — seconds since command started
 *   DTHS/TimedOut         — true when MAX_RUNTIME_S is exceeded
 * </pre>
 *
 * <h3>To restore shooting</h3>
 * Re-add {@code IShooterSubsystem}, {@code VelocityCompensator},
 * and {@link ReadyToFireCondition} from the pre-test version, and change
 * {@code isFinished()} back to {@code m_shotFired || timedOut}.
 */
public class DriveToHubAndShootCommand extends Command {

    // ── Safety timeout ─────────────────────────────────────────────────────────
    private static final double MAX_RUNTIME_S = 10.0;

    // ── Hub centre (field-relative, metres) ────────────────────────────────────
    // Loaded from the same constant used by HubAlignController so there is one
    // source of truth for the hub location.
    private static final Translation2d HUB =
        frc.robot.Constants.ShooterConstants.HUB_CENTER;

    // ── Dependencies ──────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain m_drivetrain;
    /** Driver forward/back speed (m/s, field-relative). Usually left-stick Y. */
    private final DoubleSupplier          m_translationX;
    /** Driver strafe speed (m/s, field-relative). Usually left-stick X. */
    private final DoubleSupplier          m_translationY;

    // ── Controllers ───────────────────────────────────────────────────────────
    private final HubAlignController m_alignCtrl = new HubAlignController();

    // ── State ─────────────────────────────────────────────────────────────────
    private double m_startTimeS = 0.0;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * @param drivetrain   Swerve drivetrain (required — command owns rotation).
     * @param translationX Driver forward/back speed supplier (m/s, field-relative).
     * @param translationY Driver strafe speed supplier (m/s, field-relative).
     */
    public DriveToHubAndShootCommand(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier          translationX,
            DoubleSupplier          translationY) {
        m_drivetrain   = drivetrain;
        m_translationX = translationX;
        m_translationY = translationY;
        addRequirements(drivetrain);
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_startTimeS = Timer.getFPGATimestamp();
        m_alignCtrl.reset();

        // Publish precomputed optimal-shot info immediately so the driver can see
        // where to position the robot before the command even starts aligning.
        ShotCalculator.ShotResult opt = ShotCalculator.OPTIMAL_SHOT;
        SmartDashboard.putNumber ("DTHS/OptimalDist_m",    ShotCalculator.OPTIMAL_STANDOFF_M);
        SmartDashboard.putNumber ("DTHS/OptimalAngle_deg", opt.hoodDeg());
        SmartDashboard.putNumber ("DTHS/OptimalRPM",       opt.rpm());
        SmartDashboard.putNumber ("DTHS/OptimalEntry_deg", opt.entryAngle());

        SmartDashboard.putBoolean("DTHS/Aligned",    false);
        SmartDashboard.putBoolean("DTHS/RotAligned", false);
        SmartDashboard.putBoolean("DTHS/HubActive",  false);
        SmartDashboard.putBoolean("DTHS/TimedOut",   false);
    }

    @Override
    public void execute() {
        // ── 2910 approach: profiled rotation, driver translation ───────────────
        // compute() runs the ProfiledPIDController toward the hub heading and the
        // range PID relative to the physics-optimal standoff distance.
        // In test mode we take only omegaRadS — the driver steers with the joystick.
        HubAlignController.DriveOutput driveOut =
            m_alignCtrl.compute(m_drivetrain, HUB, ShotCalculator.OPTIMAL_STANDOFF_M);

        m_drivetrain.setControl(
            new SwerveRequest.FieldCentric()
                .withVelocityX(m_translationX.getAsDouble())
                .withVelocityY(m_translationY.getAsDouble())
                .withRotationalRate(driveOut.omegaRadS()));

        // ── 1678 (Citrus) alignment gate ───────────────────────────────────────
        boolean rotAligned = m_alignCtrl.isAligned();
        boolean hubActive  = isHubConfirmedActive();
        boolean aligned    = rotAligned && hubActive;

        // ── Distance-to-hub telemetry ──────────────────────────────────────────
        double currentDist = m_drivetrain.getState().Pose.getTranslation().getDistance(HUB);
        double distError   = currentDist - ShotCalculator.OPTIMAL_STANDOFF_M;

        // ── Telemetry ─────────────────────────────────────────────────────────
        double elapsed = Timer.getFPGATimestamp() - m_startTimeS;
        SmartDashboard.putNumber ("DTHS/CurrentDist_m",  currentDist);
        SmartDashboard.putNumber ("DTHS/DistError_m",    distError);
        SmartDashboard.putBoolean("DTHS/Aligned",        aligned);
        SmartDashboard.putBoolean("DTHS/RotAligned",     rotAligned);
        SmartDashboard.putBoolean("DTHS/HubActive",      hubActive);
        SmartDashboard.putBoolean("DTHS/TimedOut",       elapsed >= MAX_RUNTIME_S);
        SmartDashboard.putNumber ("DTHS/ElapsedS",       elapsed);
    }

    @Override
    public boolean isFinished() {
        // Test mode: hold button to maintain rotation alignment; release to stop.
        // Watch "DTHS/DistError_m" to drive to the optimal position,
        // then "DTHS/Aligned" lights up when the robot is ready to shoot.
        return (Timer.getFPGATimestamp() - m_startTimeS) >= MAX_RUNTIME_S;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(new SwerveRequest.Idle());
        SmartDashboard.putBoolean("DTHS/Aligned",    false);
        SmartDashboard.putBoolean("DTHS/RotAligned", false);
        SmartDashboard.putBoolean("DTHS/HubActive",  false);
        SmartDashboard.putBoolean("DTHS/TimedOut",   false);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    /**
     * Citrus (1678) approach: confirm the hub is active before declaring aligned.
     * FMS check is skipped during practice/pit so the team can tune without FMS.
     */
    private boolean isHubConfirmedActive() {
        if (!DriverStation.isFMSAttached()) return true;
        String msg = DriverStation.getGameSpecificMessage();
        return msg != null && !msg.isEmpty();
    }
}
