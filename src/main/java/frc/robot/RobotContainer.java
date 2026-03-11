package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.VisionHardware;
import frc.robot.auto.AutonomousFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake.ActuationSubsystem;
import frc.robot.subsystems.Intake.UptakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.commands.DriveToHubAndShoot;
import frc.robot.commands.SnapHeadingToTag;
import frc.robot.commands.TurnToAngle;

/**
 * RobotContainer — wires together all subsystems and button bindings.
 *
 * <h3>Button map (Driver — port 0)</h3>
 * <pre>
 *   Left  Joystick        — Translation (field-centric)
 *   Right Joystick X      — Rotation
 *   Right Bumper  (hold)  — Auto-align heading to best visible AprilTag
 *   B Button      (toggle)— DriveToHubAndShootCommand (vision-based hub align)
 *   X Button      (hold)  — DriveToHubAndShoot (full state-machine: drive+align+fire)
 *   Left  Bumper          — Gyro reset (re-seed field-centric)
 *   A Button      (hold)  — Emergency brake (X-lock)
 *   Y Button              — Quick-turn to 180°
 *   Left  Trigger (hold)  — Climber up   → hold position on release
 *   Right Trigger (hold)  — Climber down → hold position on release
 *   D-Pad                 — Precision nudge
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
    // Operator controller — reserved for future use (intake/shooter manual overrides)
    @SuppressWarnings("unused")
    private final CommandXboxController m_playerStick = new CommandXboxController(1);

    // ── Subsystems ────────────────────────────────────────────────────────────
    public final CommandSwerveDrivetrain drivetrain;
    public final VisionSubsystem         visionSubsystem;
    public final ShooterSubsystem        shooterSubsystem;
    public final ClimberSubsystem        climberSubsystem;
    public final ActuationSubsystem      actuationSubsystem;
    public final UptakeSubsystem         uptakeSubsystem;

    // ── Autonomous ────────────────────────────────────────────────────────────
    private final AutonomousFactory m_autoFactory;

    // ── Constructor ───────────────────────────────────────────────────────────

    public RobotContainer() {

        // 1. Drivetrain MUST be first — everything else depends on it
        drivetrain = TunerConstants.createDrivetrain();

        // 2. Vision — names must match exactly what is set in PhotonVision UI
        visionSubsystem = new VisionSubsystem(
            new String[]{
                VisionHardware.CAMERA_BACK_LEFT,
                VisionHardware.CAMERA_BACK_RIGHT
            },
            drivetrain
        );

        // 3. Shooter
        shooterSubsystem = new ShooterSubsystem();

        // 4. Climber
        climberSubsystem = new ClimberSubsystem();

        // 5. Intake — arm (actuation) + roller (uptake)
        actuationSubsystem = new ActuationSubsystem();
        uptakeSubsystem    = new UptakeSubsystem();

        // 6. PathPlanner AutoBuilder — MUST be configured before AutonomousFactory
        configurePathPlanner();

        // 7. Autonomous chooser
        m_autoFactory = new AutonomousFactory(
            drivetrain,
            shooterSubsystem,
            actuationSubsystem,
            uptakeSubsystem,
            visionSubsystem
        );

        // 8. Publish static shot-calculator results to dashboard
        SmartDashboard.putNumber("DTHS/OptimalDist_m",    ShotCalculator.OPTIMAL_STANDOFF_M);
        SmartDashboard.putNumber("DTHS/OptimalAngle_deg", ShotCalculator.OPTIMAL_SHOT.hoodDeg());
        SmartDashboard.putNumber("DTHS/OptimalRPM",       ShotCalculator.OPTIMAL_SHOT.rpm());
        SmartDashboard.putNumber("DTHS/OptimalEntry_deg", ShotCalculator.OPTIMAL_SHOT.entryAngle());

        // 9. Button bindings — always last
        configureBindings();
    }

    // ── PathPlanner configuration ─────────────────────────────────────────────

    private void configurePathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                () -> drivetrain.getState().Pose,
                drivetrain::resetPose,
                () -> drivetrain.getState().Speeds,
                (speeds, feedforwards) -> drivetrain.driveRobotRelative(speeds),
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),  // translation kP
                    new PIDConstants(5.0, 0.0, 0.0)   // rotation kP
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent()
                        && alliance.get() == DriverStation.Alliance.Red;
                },
                drivetrain
            );

        } catch (Exception e) {
            System.err.println("[RobotContainer] PathPlanner AutoBuilder config failed!");
            System.err.println("  → Open project in PathPlanner app, configure settings, re-deploy.");
            System.err.println("  → Exception: " + e);
        }
    }

    // ── Button bindings ───────────────────────────────────────────────────────

    private void configureBindings() {

        // Default: field-centric teleop drive
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                m_drive
                    .withVelocityX(-m_driverStick.getLeftY()  * kMaxSpeed)
                    .withVelocityY(-m_driverStick.getLeftX()  * kMaxSpeed)
                    .withRotationalRate(-MathUtil.applyDeadband(m_driverStick.getRightX(), 0.15) * kMaxAngularRate)
            )
        );

        // Right Bumper: continuous AprilTag heading lock (hold)
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

        // X Button: full auto drive-to-hub + shoot state machine (hold)
        m_driverStick.x().whileTrue(
            new DriveToHubAndShoot(drivetrain, shooterSubsystem)
        );

        // Left Bumper: gyro reset
        m_driverStick.leftBumper().onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric)
        );

        // A Button: emergency brake (X-lock)
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

        // D-Pad: precision nudge
        m_driverStick.povUp()   .whileTrue(drivetrain.applyRequest(() -> m_drive.withVelocityX( kNudgeSpeed)));
        m_driverStick.povDown() .whileTrue(drivetrain.applyRequest(() -> m_drive.withVelocityX(-kNudgeSpeed)));
        m_driverStick.povLeft() .whileTrue(drivetrain.applyRequest(() -> m_drive.withVelocityY( kNudgeSpeed)));
        m_driverStick.povRight().whileTrue(drivetrain.applyRequest(() -> m_drive.withVelocityY(-kNudgeSpeed)));

        // Left Trigger: climber up → hold position on release
        m_driverStick.leftTrigger().whileTrue(
            Commands.run(() -> climberSubsystem.setPowerLevel(0.5), climberSubsystem)
        ).onFalse(
            Commands.runOnce(() -> {
                double pos = climberSubsystem.getCurrentPosition();
                climberSubsystem.setPositionDegrees(pos * 360.0);
            }, climberSubsystem)
        );

        // Right Trigger: climber down → hold position on release
        m_driverStick.rightTrigger().whileTrue(
            Commands.run(() -> climberSubsystem.setPowerLevel(-0.5), climberSubsystem)
        ).onFalse(
            Commands.runOnce(() -> {
                double pos = climberSubsystem.getCurrentPosition();
                climberSubsystem.setPositionDegrees(pos * 360.0);
            }, climberSubsystem)
        );

        // Telemetry
        drivetrain.registerTelemetry(m_logger::telemeterize);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    public double getCurrentRotation() {
        return drivetrain.getState().Pose.getRotation().getDegrees();
    }

    // ── Autonomous ────────────────────────────────────────────────────────────

    public Command getAutonomousCommand() {
        return m_autoFactory.getSelected();
    }
}