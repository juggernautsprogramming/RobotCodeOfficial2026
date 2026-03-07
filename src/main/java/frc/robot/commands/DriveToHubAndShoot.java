package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.VelocityCompensator;

/**
 * DriveToHubAndShoot — high-level, state-driven shoot-on-the-move command.
 *
 * <h2>State machine</h2>
 * <pre>
 *   DRIVING ──(in zone)──► ALIGNING ──(all 3 gates, 100 ms)──► FIRING ──► DONE
 *               ▲                 │
 *               └──(drifted out)──┘
 * </pre>
 *
 * <h2>Design sources</h2>
 * <ul>
 *   <li><b>1323 (Speedy Gonzales) seamlessness</b> — flywheel spins up the instant
 *       the command starts; drive, rotate, and spin-up happen in parallel so the
 *       robot arrives at the zone already at speed with minimal dead time.</li>
 *   <li><b>6328 (Mechanical Advantage) pose fusion</b> — {@code VisionSubsystem}
 *       continuously feeds Kalman-fused estimates into {@code SwerveDrivePoseEstimator}
 *       with speed-adaptive and ambiguity-adaptive std-dev trust scaling.</li>
 *   <li><b>2910 (Jack in the Bot) rotation</b> — {@link HubAlignController} uses a
 *       {@code ProfiledPIDController} for smooth trapezoidal heading control.</li>
 *   <li><b>1678 (Citrus) handshake</b> — three independent conditions must all be
 *       true for {@link #HOLD_TIME_S} before the command fires:
 *       <ol>
 *         <li>Angular error &lt; 1.5° ({@code isAligned})</li>
 *         <li>Robot within ±0.25 m of optimal standoff ({@code isAtDistance})</li>
 *         <li>Flywheel within ±50 RPM of setpoint ({@code isAtVelocity})</li>
 *       </ol>
 *   </li>
 * </ul>
 *
 * <h2>Dashboard keys ({@code DTHS2/*})</h2>
 * <pre>
 *   DTHS2/State           — current state name (DRIVING / ALIGNING / FIRING / DONE)
 *   DTHS2/DistToHub_m     — live odometry distance to hub
 *   DTHS2/DistError_m     — distance minus optimal standoff (+ = too far)
 *   DTHS2/ZoneMin_m       — lower bound of scoring zone
 *   DTHS2/ZoneMax_m       — upper bound of scoring zone
 *   DTHS2/TargetRPM       — setpoint sent to flywheel this tick
 *   DTHS2/HoodDeg         — setpoint sent to pivot this tick
 *   DTHS2/VirtDist_m      — velocity-compensated distance used for shot calc
 *   DTHS2/Gate/Aligned    — angular error &lt; 1.5°
 *   DTHS2/Gate/AtDist     — within scoring zone
 *   DTHS2/Gate/AtVelocity — flywheel within ±50 RPM of setpoint
 *   DTHS2/Gate/HubActive  — game message confirms hub is live
 *   DTHS2/Gate/HoldTime_s — consecutive seconds all gates have been green
 *   DTHS2/Gate/FIRE       — all four gates simultaneously true
 * </pre>
 */
public class DriveToHubAndShoot extends Command {

    // ── State machine ─────────────────────────────────────────────────────────

    private enum State {
        /** Outside optimal zone — drive autonomously while rotating and spinning up. */
        DRIVING,
        /** Inside zone — hold position, fine-tune heading, wait for fire gate. */
        ALIGNING,
        /** All gates cleared for {@link #HOLD_TIME_S} — execute shot. */
        FIRING,
        /** Shot fired — {@code isFinished()} returns true. */
        DONE
    }

    // ── Safety ────────────────────────────────────────────────────────────────
    private static final double MAX_RUNTIME_S = 8.0;

    // ── Optimal scoring zone (±0.25 m around the physics-derived standoff) ───
    private static final double ZONE_MIN_M = ShotCalculator.OPTIMAL_STANDOFF_M - 0.25;
    private static final double ZONE_MAX_M = ShotCalculator.OPTIMAL_STANDOFF_M + 0.25;

    // ── Re-approach trigger: if we drift this far past the zone boundary ──────
    private static final double DRIFT_THRESHOLD_M = 0.5;

    // ── 1678 fire-gate thresholds ─────────────────────────────────────────────
    // Angular tolerance (1.5°) is enforced inside HubAlignController.isAligned().
    // RPM tolerance (±50 RPM) is enforced inside ShooterSubsystem.isAtTargetRPM().
    /** All three gates must stay true this long before the shot is triggered. */
    private static final double HOLD_TIME_S = 0.10;

    // ── Hub centre (field-relative) ───────────────────────────────────────────
    private static final Translation2d HUB = ShooterConstants.HUB_CENTER;

    // ── Dependencies ──────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain m_drivetrain;
    private final ShooterSubsystem        m_shooter;

    // ── Controllers ───────────────────────────────────────────────────────────
    private final HubAlignController m_alignCtrl = new HubAlignController();

    // ── Per-run state ─────────────────────────────────────────────────────────
    private State  m_state;
    private double m_startTimeS;
    private double m_allGreenStartS;
    private double m_targetRPM;
    private double m_targetHoodDeg;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * @param drivetrain Swerve drivetrain — required (command owns rotation + translation).
     * @param shooter    Shooter subsystem — required (command owns flywheel + pivot).
     */
    public DriveToHubAndShoot(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem        shooter) {
        m_drivetrain = drivetrain;
        m_shooter    = shooter;
        addRequirements(drivetrain, shooter);
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_state          = State.DRIVING;
        m_startTimeS     = Timer.getFPGATimestamp();
        m_allGreenStartS = Double.NaN;
        m_alignCtrl.reset();

        // 1323 approach: pre-spin flywheel immediately — robot arrives already at speed.
        m_shooter.setFlywheelRPM(ShooterConstants.IDLE_RPM);

        publishInit();
    }

    @Override
    public void execute() {

        // ── 1. Live pose + velocity-compensated shot params ───────────────────
        var robotPose = m_drivetrain.getState().Pose;
        var speeds    = m_drivetrain.getState().Speeds;
        double distToHub = robotPose.getTranslation().getDistance(HUB);

        // Velocity-compensated virtual distance accounts for robot motion during ToF
        var comp = VelocityCompensator.getInstance().compensate(
            robotPose.getTranslation(), HUB,
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        ShotCalculator.ShotResult shot = ShotCalculator.calculate(comp.virtualDistanceMeters());
        m_targetRPM     = shot.rpm();
        m_targetHoodDeg = shot.hoodDeg();

        // ── 2. Shooter runs every tick regardless of state ────────────────────
        m_shooter.setFlywheelRPM(m_targetRPM);
        m_shooter.setLaunchAngleDeg(m_targetHoodDeg);

        // ── 3. Rotation + range alignment ────────────────────────────────────
        HubAlignController.DriveOutput driveOut =
            m_alignCtrl.compute(m_drivetrain, HUB, ShotCalculator.OPTIMAL_STANDOFF_M);

        boolean inZone = distToHub >= ZONE_MIN_M && distToHub <= ZONE_MAX_M;

        // ── 4. State machine ──────────────────────────────────────────────────
        switch (m_state) {

            case DRIVING -> {
                // Full autonomous translation + rotation from HubAlignController.
                // The range PID drives toward OPTIMAL_STANDOFF_M and the
                // ProfiledPIDController rotates toward the hub simultaneously.
                m_drivetrain.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(driveOut.vxMps())
                    .withVelocityY(driveOut.vyMps())
                    .withRotationalRate(driveOut.omegaRadS()));

                if (inZone) m_state = State.ALIGNING;
            }

            case ALIGNING -> {
                // Inside zone: hold position (zero translation), ProfiledPID owns rotation.
                // If we drift slightly outside the zone the range PID gently corrects,
                // but if we drift far we restart the approach phase.
                m_drivetrain.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(inZone ? 0.0 : driveOut.vxMps())
                    .withVelocityY(inZone ? 0.0 : driveOut.vyMps())
                    .withRotationalRate(driveOut.omegaRadS()));

                // ── 1678 three-condition handshake ────────────────────────────
                boolean aligned    = m_alignCtrl.isAligned();         // < 1.5°
                boolean atDist     = inZone;                           // ±0.25 m
                boolean atVelocity = isAtVelocity();                   // ±50 RPM
                boolean hubActive  = isHubConfirmedActive();           // game msg / no FMS

                boolean allGreen = aligned && atDist && atVelocity && hubActive;

                if (allGreen) {
                    if (Double.isNaN(m_allGreenStartS)) {
                        m_allGreenStartS = Timer.getFPGATimestamp();
                    }
                    if ((Timer.getFPGATimestamp() - m_allGreenStartS) >= HOLD_TIME_S) {
                        m_state = State.FIRING;
                    }
                } else {
                    m_allGreenStartS = Double.NaN; // any gate drops — reset debounce
                }

                // Significantly outside zone → back to full approach
                if (distToHub < ZONE_MIN_M - DRIFT_THRESHOLD_M
                        || distToHub > ZONE_MAX_M + DRIFT_THRESHOLD_M) {
                    m_state = State.DRIVING;
                    m_allGreenStartS = Double.NaN;
                }

                publishFireGate(aligned, atDist, atVelocity, hubActive);
            }

            case FIRING -> {
                // Keep heading locked for one tick while the shot command issues.
                m_drivetrain.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(driveOut.omegaRadS()));
                m_shooter.shoot();
                m_state = State.DONE;
            }

            case DONE -> { /* isFinished() returns true next scheduler cycle */ }
        }

        // ── 5. Telemetry ──────────────────────────────────────────────────────
        double elapsed = Timer.getFPGATimestamp() - m_startTimeS;
        SmartDashboard.putString ("DTHS2/State",        m_state.name());
        SmartDashboard.putNumber ("DTHS2/DistToHub_m",  distToHub);
        SmartDashboard.putNumber ("DTHS2/DistError_m",  distToHub - ShotCalculator.OPTIMAL_STANDOFF_M);
        SmartDashboard.putNumber ("DTHS2/ZoneMin_m",    ZONE_MIN_M);
        SmartDashboard.putNumber ("DTHS2/ZoneMax_m",    ZONE_MAX_M);
        SmartDashboard.putNumber ("DTHS2/TargetRPM",    m_targetRPM);
        SmartDashboard.putNumber ("DTHS2/HoodDeg",      m_targetHoodDeg);
        SmartDashboard.putNumber ("DTHS2/VirtDist_m",   comp.virtualDistanceMeters());
        SmartDashboard.putNumber ("DTHS2/ElapsedS",     elapsed);
        SmartDashboard.putBoolean("DTHS2/TimedOut",     elapsed >= MAX_RUNTIME_S);
    }

    @Override
    public boolean isFinished() {
        boolean timedOut = (Timer.getFPGATimestamp() - m_startTimeS) >= MAX_RUNTIME_S;
        return m_state == State.DONE || timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(new SwerveRequest.Idle());
        m_shooter.idleFlywheel();
        SmartDashboard.putString("DTHS2/State", interrupted ? "INTERRUPTED" : "COMPLETE");
    }

    // ── Fire-gate helpers ─────────────────────────────────────────────────────

    /**
     * Flywheel velocity gate — ±{@link #RPM_TOLERANCE} RPM.
     * Returns {@code true} unconditionally while the flywheel encoder is a stub.
     */
    private boolean isAtVelocity() {
        return m_shooter.isAtTargetRPM(m_targetRPM);
    }

    /**
     * 1678 hub-active check via FMS game message.
     * Bypassed in practice/pit (no FMS) so the team can test without a field.
     */
    private boolean isHubConfirmedActive() {
        if (!DriverStation.isFMSAttached()) return true;
        String msg = DriverStation.getGameSpecificMessage();
        return msg != null && !msg.isEmpty();
    }

    // ── Telemetry helpers ─────────────────────────────────────────────────────

    private void publishInit() {
        SmartDashboard.putString ("DTHS2/State",             State.DRIVING.name());
        SmartDashboard.putBoolean("DTHS2/Gate/Aligned",      false);
        SmartDashboard.putBoolean("DTHS2/Gate/AtDist",       false);
        SmartDashboard.putBoolean("DTHS2/Gate/AtVelocity",   false);
        SmartDashboard.putBoolean("DTHS2/Gate/HubActive",    false);
        SmartDashboard.putBoolean("DTHS2/Gate/FIRE",         false);
        SmartDashboard.putNumber ("DTHS2/Gate/HoldTime_s",   0.0);
        SmartDashboard.putBoolean("DTHS2/TimedOut",          false);
        SmartDashboard.putNumber ("DTHS2/ZoneMin_m",         ZONE_MIN_M);
        SmartDashboard.putNumber ("DTHS2/ZoneMax_m",         ZONE_MAX_M);
    }

    private void publishFireGate(
            boolean aligned, boolean atDist, boolean atVelocity, boolean hubActive) {
        SmartDashboard.putBoolean("DTHS2/Gate/Aligned",    aligned);
        SmartDashboard.putBoolean("DTHS2/Gate/AtDist",     atDist);
        SmartDashboard.putBoolean("DTHS2/Gate/AtVelocity", atVelocity);
        SmartDashboard.putBoolean("DTHS2/Gate/HubActive",  hubActive);
        SmartDashboard.putBoolean("DTHS2/Gate/FIRE",       aligned && atDist && atVelocity && hubActive);
        SmartDashboard.putNumber ("DTHS2/Gate/HoldTime_s",
            Double.isNaN(m_allGreenStartS) ? 0.0
                : Timer.getFPGATimestamp() - m_allGreenStartS);
    }
}
