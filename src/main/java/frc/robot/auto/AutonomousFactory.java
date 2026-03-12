package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.Constants.AutoStartConstants;
import frc.robot.commands.ShootNBallsCommand;
import frc.robot.commands.SnapHeadingToTag;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake.ActuationSubsystem;
import frc.robot.subsystems.Intake.IntakeAdapter;
import frc.robot.subsystems.Intake.UptakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Vision.VisionSubsystem;

/**
 * AutonomousFactory — builds Left, Center, and Right 8-ball autonomous routines
 * and registers them on a SmartDashboard {@link SendableChooser}.
 *
 * <h2>Sequence (all three positions)</h2>
 * <pre>
 *  Reset pose
 *  → Drive to Shoot Zone 1  (flywheel pre-spins in parallel)
 *  → Shoot × 3 preloaded balls
 *  → Drive to Pickup Zone   (intake running during drive)
 *  → Intake safety catch    (max 3 s)
 *  → Drive to Shoot Zone 2  (flywheel pre-spins in parallel)
 *  → SnapHeadingToTag align (max 2.5 s) then shoot × 5
 *  → Idle cleanup
 * </pre>
 *
 * <h2>PathPlanner paths required</h2>
 * Place in {@code src/main/deploy/pathplanner/paths/}:
 * Center_StartToShoot.path, Center_ShootToPickup.path, Center_PickupToShoot2.path,
 * Left_StartToShoot.path, Left_ShootToPickup.path, Left_PickupToShoot2.path,
 * Right_StartToShoot.path, Right_ShootToPickup.path, Right_PickupToShoot2.path
 */
public class AutonomousFactory {

    // ── PathPlanner motion constraints ────────────────────────────────────────
    // Defined inline here — no dependency on AutoConstants fields.
    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        2.0,                            // max velocity m/s
        2.5,                            // max acceleration m/s²
        Units.degreesToRadians(360.0),  // max angular velocity rad/s
        Units.degreesToRadians(720.0)   // max angular acceleration rad/s²
    );

    // ── Dependencies ──────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain m_drivetrain;
    private final ShooterSubsystem        m_shooter;
    private final ActuationSubsystem      m_actuation;
    private final UptakeSubsystem         m_uptake;
    private final VisionSubsystem         m_vision;
    private final IntakeAdapter           m_intake;

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * @param drivetrain AutoBuilder must be configured before calling this.
     * @param shooter    Shooter subsystem.
     * @param actuation  Arm deployment motor (CAN ID 43).
     * @param uptake     Intake roller motor (CAN ID 42).
     * @param vision     VisionSubsystem passed to SnapHeadingToTag.
     */
    public AutonomousFactory(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem        shooter,
            ActuationSubsystem      actuation,
            UptakeSubsystem         uptake,
            VisionSubsystem         vision) {
        m_drivetrain = drivetrain;
        m_shooter    = shooter;
        m_actuation  = actuation;
        m_uptake     = uptake;
        m_vision     = vision;
        m_intake     = new IntakeAdapter(actuation, uptake);

        registerNamedCommands();
        buildAndRegister();
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /** Returns the SmartDashboard chooser. Pass to Shuffleboard/Elastic. */
    public SendableChooser<Command> getChooser() { return m_chooser; }

    /** Returns the currently selected autonomous command. */
    public Command getSelected() { return m_chooser.getSelected(); }

    // ── Registration ──────────────────────────────────────────────────────────

    // ── Named commands for PathPlanner GUI ───────────────────────────────────

    /**
     * Registers all robot actions as named commands so they can be placed as
     * event markers inside PathPlanner's GUI auto editor.
     *
     * Available names (use these exact strings in the PathPlanner app):
     *   "SpinUpFlywheel"  — set flywheel to optimal RPM and hood angle
     *   "IdleFlywheel"    — return flywheel to idle speed
     *   "ShootPreloads"   — fire the 3 preloaded balls
     *   "ShootPickups"    — fire the 5 picked-up balls
     *   "StartIntake"     — deploy arm and run intake roller
     *   "StopIntake"      — retract arm and stop intake roller
     *   "AlignToTag"      — rotate to nearest AprilTag (max 2.5 s)
     */
    private void registerNamedCommands() {
        double optRPM     = ShotCalculator.OPTIMAL_SHOT.rpm();
        double optHoodDeg = ShotCalculator.OPTIMAL_SHOT.hoodDeg();

        NamedCommands.registerCommand("SpinUpFlywheel", new InstantCommand(() -> {
            m_shooter.setFlywheelRPM(optRPM);
            m_shooter.setLaunchAngleDeg(optHoodDeg);
        }));

        NamedCommands.registerCommand("IdleFlywheel",
            new InstantCommand(m_shooter::idleFlywheel));

        NamedCommands.registerCommand("ShootPreloads",
            new ShootNBallsCommand(m_shooter, AutoStartConstants.PRELOAD_BALL_COUNT, optRPM));

        NamedCommands.registerCommand("ShootPickups",
            new ShootNBallsCommand(m_shooter, AutoStartConstants.PICKUP_BALL_COUNT, optRPM));

        NamedCommands.registerCommand("StartIntake",
            new InstantCommand(m_intake::run, m_actuation, m_uptake));

        NamedCommands.registerCommand("StopIntake",
            new InstantCommand(m_intake::stop, m_actuation, m_uptake));

        NamedCommands.registerCommand("AlignToTag",
            new SnapHeadingToTag(m_drivetrain, m_vision, () -> 0.0, () -> 0.0)
                .withTimeout(2.5));
    }

    private void buildAndRegister() {
        // Safe default — does nothing if no auto is selected
        m_chooser.setDefaultOption("Do Nothing", Commands.none());

        // Load every .auto file from deploy/pathplanner/autos/ automatically
        for (String name : AutoBuilder.getAllAutoNames()) {
            m_chooser.addOption(name, AutoBuilder.buildAuto(name));
        }

        SmartDashboard.putData("Auto Chooser", m_chooser);
    }
}