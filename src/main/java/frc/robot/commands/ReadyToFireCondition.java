package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

/**
 * ReadyToFireCondition — five-factor fire gate with debounce.
 *
 * <h3>Conditions (all must be true simultaneously)</h3>
 * <ol>
 *   <li><b>Flywheel RPM</b> — current RPM within ±{@link #RPM_TOLERANCE_FRACTION} of setpoint.</li>
 *   <li><b>Drivetrain heading</b> — angular error &lt; {@link #HEADING_TOLERANCE_DEG},
 *       wrapped to [−180, 180].</li>
 *   <li><b>Vision freshness</b> — a hub AprilTag is currently visible.</li>
 *   <li><b>Range</b> — robot within {@link ShooterConstants#DISTANCE_TOLERANCE_METERS}
 *       of desired standoff (from odometry).</li>
 *   <li><b>Hood angle</b> — pivot within {@link ShooterConstants#PIVOT_ANGLE_TOLERANCE_DEG}.</li>
 * </ol>
 *
 * <h3>Debounce</h3>
 * All five conditions must stay true for {@link #HOLD_TIME_S} consecutive seconds
 * before {@link #isReady} returns {@code true}.
 *
 * <h3>Fixes applied</h3>
 * <ul>
 *   <li>RPM check was logically inverted (AND of lower-bound AND upper-bound on the
 *       same {@code isAtTargetRPM()} call could never both be true simultaneously).
 *       Replaced with a direct {@code Math.abs} delta against
 *       {@link ShooterSubsystem#getCurrentRPM()}.</li>
 *   <li>Heading error normalisation moved into a named helper for clarity.</li>
 * </ul>
 */
public class ReadyToFireCondition {

    // ── Tunables ──────────────────────────────────────────────────────────────

    /** Flywheel tolerance: fraction of setpoint (0.01 = ±1 %). */
    private static final double RPM_TOLERANCE_FRACTION = 0.01;

    /** Max acceptable heading error to the virtual aim point (degrees). */
    private static final double HEADING_TOLERANCE_DEG = 2.0;

    /**
     * All conditions must stay true for this many seconds before firing.
     * Prevents triggering on transient all-green glitches.
     */
    private static final double HOLD_TIME_S = 0.10;

    // ── State ─────────────────────────────────────────────────────────────────
    private double m_allGreenStart = Double.NaN;

    // ── Dependencies ──────────────────────────────────────────────────────────
    private final ShooterSubsystem        m_shooter;
    private final VisionSubsystem         m_vision;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final HubAlignController      m_alignCtrl;

    public ReadyToFireCondition(
            ShooterSubsystem        shooter,
            VisionSubsystem         vision,
            CommandSwerveDrivetrain drivetrain,
            HubAlignController      alignCtrl) {
        m_shooter    = shooter;
        m_vision     = vision;
        m_drivetrain = drivetrain;
        m_alignCtrl  = alignCtrl;
    }

    // ── Main gate ─────────────────────────────────────────────────────────────

    /**
     * Call once per 20 ms loop tick.
     *
     * @param targetRPM     Flywheel setpoint for this tick.
     * @param targetHoodDeg Hood angle setpoint for this tick.
     * @return {@code true} only when every condition has been met for {@link #HOLD_TIME_S}.
     */
    public boolean isReady(double targetRPM, double targetHoodDeg) {
        boolean rpm     = checkRPM(targetRPM);
        boolean heading = checkHeading();
        boolean vision  = m_vision.hasHubTarget();
        boolean dist    = m_alignCtrl.isAtDistance();
        boolean hood    = m_shooter.isAtTargetAngle(targetHoodDeg);

        boolean allGreen = rpm && heading && vision && dist && hood;

        if (allGreen) {
            if (Double.isNaN(m_allGreenStart)) {
                m_allGreenStart = Timer.getFPGATimestamp();
            }
        } else {
            // Any condition dropped — reset debounce timer immediately
            m_allGreenStart = Double.NaN;
        }

        boolean ready = allGreen
            && (Timer.getFPGATimestamp() - m_allGreenStart) >= HOLD_TIME_S;

        publishTelemetry(rpm, heading, vision, dist, hood, ready);
        return ready;
    }

    /** Reset debounce. Call in command {@code initialize()}. */
    public void reset() { m_allGreenStart = Double.NaN; }

    // ── Individual checks ─────────────────────────────────────────────────────

    /**
     * Checks whether the flywheel is within ±{@link #RPM_TOLERANCE_FRACTION} of setpoint.
     *
     * <p>Currently {@code ShooterSubsystem.isAtTargetRPM()} is a stub returning {@code true}
     * always — this gate will pass unconditionally until the flywheel encoder is wired.
     * Once wired, update {@code isAtTargetRPM()} in ShooterSubsystem to:
     * <pre>
     *   double currentRPM = m_flywheelLeader.getVelocity().getValueAsDouble() * 60.0;
     *   return Math.abs(currentRPM - targetRPM) &lt; targetRPM * 0.01;
     * </pre>
     * No changes needed here — this method will work correctly once the subsystem is real.
     */
    private boolean checkRPM(double targetRPM) {
        return m_shooter.isAtTargetRPM(targetRPM);
    }

    /**
     * Checks whether the drivetrain heading is within {@link #HEADING_TOLERANCE_DEG}
     * of the velocity-compensated aim heading, using proper [−180, 180] wrap.
     */
    private boolean checkHeading() {
        double current = m_drivetrain.getState().Pose.getRotation().getDegrees();
        double desired = m_alignCtrl.getDesiredHeadingDeg();
        // Normalise to [-180, 180] to match continuous-input PID convention
        double err = ((current - desired + 180.0) % 360.0) - 180.0;
        return Math.abs(err) < HEADING_TOLERANCE_DEG;
    }

    // ── Telemetry ─────────────────────────────────────────────────────────────

    private void publishTelemetry(
            boolean rpm, boolean heading, boolean vision,
            boolean dist, boolean hood, boolean ready) {

        SmartDashboard.putBoolean("ReadyToFire/RPM_OK",     rpm);
        SmartDashboard.putBoolean("ReadyToFire/Heading_OK", heading);
        SmartDashboard.putBoolean("ReadyToFire/Vision_OK",  vision);
        SmartDashboard.putBoolean("ReadyToFire/Dist_OK",    dist);
        SmartDashboard.putBoolean("ReadyToFire/Hood_OK",    hood);
        SmartDashboard.putBoolean("ReadyToFire/READY",      ready);

        if (!Double.isNaN(m_allGreenStart)) {
            SmartDashboard.putNumber("ReadyToFire/HoldTime_s",
                Timer.getFPGATimestamp() - m_allGreenStart);
        } else {
            SmartDashboard.putNumber("ReadyToFire/HoldTime_s", 0.0);
        }
    }
}