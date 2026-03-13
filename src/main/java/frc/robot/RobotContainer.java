package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;

// Constants
import frc.robot.Constants.DriveToPoseConstants;
import frc.robot.Constants.VisionHardware;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Shooter.ShotCalculator;

// Subsystems
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Intake.ActuationSubsystem;
import frc.robot.subsystems.Intake.UptakeSubsystem;
import frc.robot.subsystems.Intake.IntakeAdapter;

// Commands
import frc.robot.commands.DriveToHubAndShoot;
import frc.robot.commands.SnapHeadingToTag;
import frc.robot.commands.TurnToAngle;

/**
 * RobotContainer — wires together all subsystems and button bindings.
 *
 * <h3>Button map (Driver — port 0)</h3>
 * <pre>
 *   Left  Joystick      — Translation (field-centric)
 *   Right Joystick X    — Rotation
 *   Right Bumper        — Auto-align to AprilTag (hold)
 *   B Button            — Drive to Hub + Shoot (hold, auto-releases after shot)
 *   Left  Bumper        — Gyro reset (field-centric re-seed)
 *   A Button            — Emergency brake
 *   Y Button            — Quick turn to 45°
 *   D-Pad               — Precision nudge
 * </pre>
 *
 * <h3>Button map (Operator — port 1)</h3>
 * <pre>
 *   A Button             — Toggle intake  (deploy + spin  ↔ stow + stop)
 *   B Button             — Toggle feeder  (run forward     ↔ stop)
 *   X Button             — Toggle shooter flywheel (spin up to optimal RPM ↔ idle)
 *   Left Trigger  (hold) — Climber up,   hold position on release
 *   Right Trigger (hold) — Climber down, hold position on release
 *   Left Bumper          — Eject (intake reverse, hold)
 *   Right Bumper         — Reverse feeder (hold)
 * </pre>
 */
public class RobotContainer {

    // ── Speed constants ───────────────────────────────────────────────────────
    private final double kMaxSpeed       = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final double kNudgeSpeed     = 0.2 * kMaxSpeed;

    // ── Swerve requests ───────────────────────────────────────────────────────
    private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
        .withDeadband(kMaxSpeed * 0.1)
        .withRotationalDeadband(kMaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry m_logger = new Telemetry(kMaxSpeed);

    // ── Controllers ───────────────────────────────────────────────────────────
    private final CommandXboxController m_driverStick = new CommandXboxController(0);

    private final CommandXboxController m_playerStick = new CommandXboxController(1);

    // ── Autonomous chooser ────────────────────────────────────────────────────
    private final SendableChooser<Command> m_autoChooser;

    /** Field2d used to display PathPlanner path + target pose in AdvantageScope 2D view. */
    private final Field2d m_pathPlannerField = new Field2d();

    // ── Subsystems ────────────────────────────────────────────────────────────
    public final CommandSwerveDrivetrain drivetrain;
    public final VisionSubsystem         visionSubsystem;
    public final ShooterSubsystem        shooterSubsystem;
    public final ClimberSubsystem        climberSubsystem;
    public final FeederSubsystem         feederSubsystem;
    public final ActuationSubsystem      actuationSubsystem;
    public final UptakeSubsystem         uptakeSubsystem;
    public final IntakeAdapter           intakeAdapter;

    // ── Constructor ───────────────────────────────────────────────────────────

    public RobotContainer() {
        // 1. Drivetrain MUST be first — everything else depends on it
        drivetrain = TunerConstants.createDrivetrain();

        // 2. Shooter created early — needed by Named Commands below
        shooterSubsystem = new ShooterSubsystem();

        // 3. Configure PathPlanner AutoBuilder
        //    Pose supplier uses SwerveDrivePoseEstimator (vision-fused) — NOT raw encoders.
        //    Dynamic replanning is built into FollowPathCommand in PathPlanner 2026 and runs
        //    automatically — no ReplanningConfig needed (class removed in 2026 API).
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                () -> drivetrain.getState().Pose,                          // vision-fused pose
                drivetrain::resetPose,
                () -> drivetrain.getState().Speeds,
                (speeds, feedforwards) -> drivetrain.driveRobotRelative(speeds),
                new PPHolonomicDriveController(
                    new PIDConstants(DriveToPoseConstants.kP_XY,    0, DriveToPoseConstants.kD_XY),
                    new PIDConstants(DriveToPoseConstants.kP_THETA, 0, DriveToPoseConstants.kD_THETA)
                ),
                config,
                // ⚠ Custom hub field: hub is at x=4.597, NOT the field centre (x=4.0).
                // PathPlanner's mirror-around-centre would put path endpoints ~0.2 m from
                // the hub instead of the 1.5 m standoff we need, so we NEVER flip paths.
                // Design separate blue/red path files instead (e.g. Center_Blue_StartToShoot.path).
                () -> false,
                drivetrain
            );
        } catch (Exception e) {
            DriverStation.reportError("AutoBuilder config failed: " + e.getMessage(), e.getStackTrace());
        }

        // 4. PathPlanner → AdvantageScope logging
        //    Field2d approach: AdvantageScope 2D field natively understands Field2d objects,
        //    so "active-path" shows up as a selectable Trajectory and "target" as a robot pose.
        SmartDashboard.putData("PathPlanner/Field", m_pathPlannerField);

        PathPlannerLogging.setLogActivePathCallback(path -> {
            Logger.recordOutput("PathPlanner/ActivePath", path.toArray(new Pose2d[0]));
            m_pathPlannerField.getObject("active-path").setPoses(path);
        });
        PathPlannerLogging.setLogTargetPoseCallback(pose -> {
            Logger.recordOutput("PathPlanner/TargetPose", pose);
            m_pathPlannerField.getObject("target-pose").setPose(pose);
        });
        PathPlannerLogging.setLogCurrentPoseCallback(
            pose -> Logger.recordOutput("PathPlanner/CurrentPose", pose));

        // 5. Register Named Commands — must be done BEFORE buildAuto() is called.
        //    These names must match exactly what is typed in the PathPlanner GUI.
        NamedCommands.registerCommand("SpinUpFlywheel", Commands.runOnce(() -> {
            shooterSubsystem.setFlywheelRPM(ShotCalculator.OPTIMAL_SHOT_FIXED.rpm());
            shooterSubsystem.setLaunchAngleDeg(ShotCalculator.OPTIMAL_SHOT_FIXED.hoodDeg());
        }, shooterSubsystem));

        NamedCommands.registerCommand("IdleFlywheel",
            Commands.runOnce(shooterSubsystem::idleFlywheel, shooterSubsystem));

        // 6. Build auto chooser from all .auto files in deploy/pathplanner/autos/
        m_autoChooser = new SendableChooser<>();
        m_autoChooser.setDefaultOption("Do Nothing", Commands.none());
        for (String name : AutoBuilder.getAllAutoNames()) {
            m_autoChooser.addOption(name, AutoBuilder.buildAuto(name));
        }
        SmartDashboard.putData("Auto Chooser", m_autoChooser);

        // 7. Remaining subsystems
        visionSubsystem = new VisionSubsystem(
            new String[]{
                VisionHardware.CAMERA_BACK_LEFT,
                VisionHardware.CAMERA_BACK_RIGHT,
                VisionHardware.CAMERA_FRONT
            },
            drivetrain
        );
        climberSubsystem    = new ClimberSubsystem();
        feederSubsystem     = new FeederSubsystem();
        actuationSubsystem  = new ActuationSubsystem();
        uptakeSubsystem     = new UptakeSubsystem();
        intakeAdapter       = new IntakeAdapter(actuationSubsystem, uptakeSubsystem);

        // Publish static shot-calculator results once so Elastic can see the
        // DTHS/* keys immediately on connect, before the command ever runs.
        SmartDashboard.putNumber("DTHS/OptimalDist_m",    ShotCalculator.OPTIMAL_STANDOFF_FIXED_M);
        SmartDashboard.putNumber("DTHS/OptimalAngle_deg", ShotCalculator.OPTIMAL_SHOT_FIXED.hoodDeg());
        SmartDashboard.putNumber("DTHS/OptimalRPM",       ShotCalculator.OPTIMAL_SHOT_FIXED.rpm());
        SmartDashboard.putNumber("DTHS/OptimalEntry_deg", ShotCalculator.OPTIMAL_SHOT_FIXED.entryAngle());

        configureBindings();
    }

    // ── Button bindings ───────────────────────────────────────────────────────

    private void configureBindings() {

        // ── Default: field-centric teleop ─────────────────────────────────────
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                m_drive
                    .withVelocityX(-m_driverStick.getLeftY()  * kMaxSpeed)
                    .withVelocityY(-m_driverStick.getLeftX()  * kMaxSpeed)
                    .withRotationalRate(-MathUtil.applyDeadband(m_driverStick.getRightX(), 0.15) * kMaxAngularRate)
            )
        );

        // ── Right Bumper: Continuous heading alignment to best visible AprilTag ─
        // Hold → robot continuously faces whatever tag is visible.
        // Driver keeps full X/Y translation control.
        // Releases when button is released; holds last known heading if tag lost.
        m_driverStick.rightBumper().whileTrue(
            new SnapHeadingToTag(
                drivetrain,
                visionSubsystem,
                () -> -m_driverStick.getLeftY() * kMaxSpeed,
                () -> -m_driverStick.getLeftX() * kMaxSpeed
            )
        );

        // ── B Button: Drive to hub + shoot ───────────────────────────────────
        // Measures current distance to hub, drives to optimal standoff,
        // aligns to hub center, and fires when all gates pass.
        m_driverStick.b().whileTrue(
            new DriveToHubAndShoot(drivetrain, shooterSubsystem)
        );

        // ── Left Bumper: Gyro reset ───────────────────────────────────────────
        // Re-seeds field-centric heading so the current robot direction = "forward".
        m_driverStick.leftBumper().onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric)
        );

        // ── A Button: Emergency brake ─────────────────────────────────────────
        m_driverStick.a().whileTrue(
            drivetrain.applyRequest(() -> m_brake)
        );

        // ── X Button: Drive to Hub + Shoot (full autonomous sequence) ────────
        // State machine: DRIVING → ALIGNING → FIRING → DONE.
        // Flywheel pre-spins on press. Fires automatically when all three gates
        // pass (aligned < 1.5°, in zone ±0.25 m, RPM ±50). Watch DTHS2/* keys.
        m_driverStick.x().whileTrue(
            new DriveToHubAndShoot(drivetrain, shooterSubsystem)
        );

        // ── Y Button: Quick-turn 45° ──────────────────────────────────────────
        m_driverStick.y().onTrue(new TurnToAngle(drivetrain, 45, false));

        // ── D-Pad: Precision nudge ────────────────────────────────────────────
        m_driverStick.povUp()   .whileTrue(drivetrain.applyRequest(() -> m_drive.withVelocityX( kNudgeSpeed)));
        m_driverStick.povDown() .whileTrue(drivetrain.applyRequest(() -> m_drive.withVelocityX(-kNudgeSpeed)));
        m_driverStick.povLeft() .whileTrue(drivetrain.applyRequest(() -> m_drive.withVelocityY( kNudgeSpeed)));
        m_driverStick.povRight().whileTrue(drivetrain.applyRequest(() -> m_drive.withVelocityY(-kNudgeSpeed)));

        // ── Operator (port 1) ─────────────────────────────────────────────────

        // Default: always retract intake when not actively toggled
        actuationSubsystem.setDefaultCommand(
            Commands.run(() -> actuationSubsystem.setPosition(0.0), actuationSubsystem)
        );

        // A: Toggle intake — deploy + spin roller  ↔  stow + stop
        m_playerStick.a().toggleOnTrue(
            Commands.startEnd(
                intakeAdapter::run,
                intakeAdapter::stop,
                actuationSubsystem, uptakeSubsystem
            )
        );

        // B: Toggle feeder — run forward  ↔  stop
        m_playerStick.b().toggleOnTrue(
            Commands.startEnd(
                () -> feederSubsystem.setPower(5.0),
                feederSubsystem::stop,
                feederSubsystem
            )
        );

        // X: Toggle shooter flywheel — spin up to optimal RPM  ↔  idle
        m_playerStick.x().toggleOnTrue(
            Commands.startEnd(
                () -> {
                    shooterSubsystem.setFlywheelRPM(ShotCalculator.OPTIMAL_SHOT_FIXED.rpm());
                    shooterSubsystem.setLaunchAngleDeg(ShotCalculator.OPTIMAL_SHOT_FIXED.hoodDeg());
                },
                shooterSubsystem::idleFlywheel,
                shooterSubsystem
            )
        );

        // Left Trigger: Climber up (hold), hold position on release
        m_playerStick.leftTrigger().whileTrue(
            Commands.run(() -> climberSubsystem.setPowerLevel(0.5), climberSubsystem)
        ).onFalse(
            Commands.runOnce(() -> {
                double pos = climberSubsystem.getCurrentPosition();
                climberSubsystem.setPositionDegrees(pos * 360.0);
            }, climberSubsystem)
        );

        // Right Trigger: Climber down (hold), hold position on release
        m_playerStick.rightTrigger().whileTrue(
            Commands.run(() -> climberSubsystem.setPowerLevel(-0.5), climberSubsystem)
        ).onFalse(
            Commands.runOnce(() -> {
                double pos = climberSubsystem.getCurrentPosition();
                climberSubsystem.setPositionDegrees(pos * 360.0);
            }, climberSubsystem)
        );

        // Left Bumper: Eject — intake roller reverse (hold)
        m_playerStick.leftBumper().whileTrue(
            Commands.run(intakeAdapter::eject, actuationSubsystem, uptakeSubsystem)
        );

        // Right Bumper: Reverse feeder (toggle)
        m_playerStick.rightBumper().toggleOnTrue(
            Commands.startEnd(
                () -> feederSubsystem.setPower(-3.0),
                feederSubsystem::stop,
                feederSubsystem
            )
        );

        // ── Telemetry hook ────────────────────────────────────────────────────
        drivetrain.registerTelemetry(m_logger::telemeterize);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    /** Returns the robot's current heading in degrees (field-relative). */
    public double getCurrentRotation() {
        return drivetrain.getState().Pose.getRotation().getDegrees();
    }

    // ── Autonomous ────────────────────────────────────────────────────────────

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}