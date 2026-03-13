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
import frc.robot.subsystems.Intake.IntakeRollerSubsystem;
import frc.robot.subsystems.Intake.UptakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Vision.VisionSubsystem;

/**
 * AutonomousFactory — builds Left, Center, and Right autonomous routines
 * and registers them on a SmartDashboard {@link SendableChooser}.
 */
public class AutonomousFactory {

    // ── PathPlanner motion constraints ────────────────────────────────────────
    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        2.0,
        2.5,
        Units.degreesToRadians(360.0),
        Units.degreesToRadians(720.0)
    );

    // ── Dependencies ──────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain m_drivetrain;
    private final ShooterSubsystem        m_shooter;
    private final ActuationSubsystem      m_actuation;
    private final UptakeSubsystem         m_uptake;
    private final IntakeRollerSubsystem   m_rollers;
    private final VisionSubsystem         m_vision;
    private final IntakeAdapter           m_intake;

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * @param drivetrain AutoBuilder must be configured before calling this.
     * @param shooter    Shooter subsystem.
     * @param actuation  Arm deployment motor (CAN ID 43).
     * @param uptake     Intake roller motor (CAN ID 42).
     * @param rollers    Intake bar motors (CAN IDs 27 & 28).
     * @param vision     VisionSubsystem passed to SnapHeadingToTag.
     */
    public AutonomousFactory(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem        shooter,
            ActuationSubsystem      actuation,
            UptakeSubsystem         uptake,
            IntakeRollerSubsystem   rollers,
            VisionSubsystem         vision) {
        m_drivetrain = drivetrain;
        m_shooter    = shooter;
        m_actuation  = actuation;
        m_uptake     = uptake;
        m_rollers    = rollers;
        m_vision     = vision;
        m_intake     = new IntakeAdapter(m_actuation, m_uptake, m_rollers);

        registerNamedCommands();
        buildAndRegister();
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /** Returns the SmartDashboard chooser. */
    public SendableChooser<Command> getChooser() { return m_chooser; }

    /** Returns the currently selected autonomous command. */
    public Command getSelected() { return m_chooser.getSelected(); }

    // ── Named commands ────────────────────────────────────────────────────────

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
            new InstantCommand(m_intake::run, m_actuation, m_uptake, m_rollers));

        NamedCommands.registerCommand("StopIntake",
            new InstantCommand(m_intake::stop, m_actuation, m_uptake, m_rollers));

        NamedCommands.registerCommand("AlignToTag",
            new SnapHeadingToTag(m_drivetrain, m_vision, () -> 0.0, () -> 0.0)
                .withTimeout(2.5));
    }

    private void buildAndRegister() {
        m_chooser.setDefaultOption("Do Nothing", Commands.none());

        for (String name : AutoBuilder.getAllAutoNames()) {
            m_chooser.addOption(name, AutoBuilder.buildAuto(name));
        }

        SmartDashboard.putData("Auto Chooser", m_chooser);
    }
}