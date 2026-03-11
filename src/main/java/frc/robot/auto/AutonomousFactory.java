package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.Constants.AutoStartConstants;
import frc.robot.commands.RunIntakeCommand;
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

        buildAndRegister();
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /** Returns the SmartDashboard chooser. Pass to Shuffleboard/Elastic. */
    public SendableChooser<Command> getChooser() { return m_chooser; }

    /** Returns the currently selected autonomous command. */
    public Command getSelected() { return m_chooser.getSelected(); }

    // ── Registration ──────────────────────────────────────────────────────────

    private void buildAndRegister() {
        m_chooser.setDefaultOption("Center Auto (8 balls)",
            buildSequence("Center",
                new Pose2d(AutoStartConstants.CENTER_START_X,
                           AutoStartConstants.CENTER_START_Y,
                           Rotation2d.fromDegrees(AutoStartConstants.CENTER_START_HDG))));

        m_chooser.addOption("Left Auto (8 balls)",
            buildSequence("Left",
                new Pose2d(AutoStartConstants.LEFT_START_X,
                           AutoStartConstants.LEFT_START_Y,
                           Rotation2d.fromDegrees(AutoStartConstants.LEFT_START_HDG))));

        m_chooser.addOption("Right Auto (8 balls)",
            buildSequence("Right",
                new Pose2d(AutoStartConstants.RIGHT_START_X,
                           AutoStartConstants.RIGHT_START_Y,
                           Rotation2d.fromDegrees(AutoStartConstants.RIGHT_START_HDG))));

        SmartDashboard.putData("Auto Chooser", m_chooser);
    }

    // ── Sequence builder ──────────────────────────────────────────────────────

    private Command buildSequence(String label, Pose2d startPose) {

        double optRPM     = ShotCalculator.OPTIMAL_SHOT.rpm();
        double optHoodDeg = ShotCalculator.OPTIMAL_SHOT.hoodDeg();

        // Phase 0: Seed odometry
        Command resetPose = new InstantCommand(
            () -> m_drivetrain.resetPose(startPose));

        // Phase 1: Drive to first shoot zone (~2 m forward) + pre-spin flywheel
        Command driveToShot1 = new ParallelCommandGroup(
            loadPath(label + "_StartToShoot"),
            new InstantCommand(() -> {
                m_shooter.setFlywheelRPM(optRPM);
                m_shooter.setLaunchAngleDeg(optHoodDeg);
            })
        );

        // Phase 2: Fire 3 preloaded balls
        Command shootPreloads = new ShootNBallsCommand(
            m_shooter, AutoStartConstants.PRELOAD_BALL_COUNT, optRPM);

        // Phase 3: Drive to pickup zone with intake running the whole way
        Command driveToPickup = new SequentialCommandGroup(
            new InstantCommand(m_intake::run, m_actuation, m_uptake),
            loadPath(label + "_ShootToPickup"),
            new InstantCommand(m_intake::stop, m_actuation, m_uptake)
        );

        // Phase 4: Safety intake catch — max 3 s
        Command intakeSafety = new RunIntakeCommand(m_intake, m_actuation, m_uptake)
            .withTimeout(3.0);

        // Phase 5: Drive to second shoot zone + pre-spin flywheel
        Command driveToShot2 = new ParallelCommandGroup(
            loadPath(label + "_PickupToShoot2"),
            new InstantCommand(() -> {
                m_shooter.setFlywheelRPM(optRPM);
                m_shooter.setLaunchAngleDeg(optHoodDeg);
            })
        );

        // Phase 6: Align to AprilTag (hold position, rotate only) then shoot 5
        Command alignAndShoot = new SequentialCommandGroup(
            new SnapHeadingToTag(
                m_drivetrain, m_vision,
                () -> 0.0,  // no X translation while aligning
                () -> 0.0   // no Y translation while aligning
            ).withTimeout(2.5),
            new ShootNBallsCommand(
                m_shooter, AutoStartConstants.PICKUP_BALL_COUNT, optRPM)
        );

        // Phase 7: Cleanup — idle flywheel and stow intake
        Command cleanup = new InstantCommand(() -> {
            m_shooter.idleFlywheel();
            m_intake.stop();
        });

        return new SequentialCommandGroup(
            resetPose,
            driveToShot1,
            shootPreloads,
            driveToPickup,
            intakeSafety,
            driveToShot2,
            alignAndShoot,
            cleanup
        )
        .withTimeout(AutoStartConstants.AUTO_TIMEOUT_S)
        .withName(label + "Auto8Ball");
    }

    // ── PathPlanner path loader ───────────────────────────────────────────────

    private Command loadPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            System.err.println("[AutoFactory] Missing path: " + pathName + ".path");
            System.err.println("  → Place in: deploy/pathplanner/paths/");
            return new WaitCommand(0.5);
        }
    }
}