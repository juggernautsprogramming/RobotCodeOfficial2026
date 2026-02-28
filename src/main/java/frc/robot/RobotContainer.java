package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Constants
import frc.robot.Constants.VisionHardware;
import frc.robot.generated.TunerConstants;

// Subsystems
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;

// Commands
import frc.robot.commands.AlignToTag;
import frc.robot.commands.DriveToHubAndShootCommand;
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
 *   Y Button            — Quick turn to 180°
 *   Left Trigger (hold) — Climber up (hold), hold position on release
 *   Right Trigger (hold)— Climber down (hold), hold position on release
 *   D-Pad               — Precision nudge
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

    // ── Subsystems ────────────────────────────────────────────────────────────
    public final CommandSwerveDrivetrain drivetrain;
    public final VisionSubsystem         visionSubsystem;
    public final ShooterSubsystem        shooterSubsystem;
    public final ClimberSubsystem        climberSubsystem;

    // ── Constructor ───────────────────────────────────────────────────────────

    public RobotContainer() {
        // 1. Drivetrain MUST be first — everything else depends on it
        drivetrain = TunerConstants.createDrivetrain();

        // 2. Vision — camera names come from VisionHardware constants
        //    (these must match exactly what is set in the PhotonVision web UI)
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
                    .withRotationalRate(-m_driverStick.getRightX() * kMaxAngularRate)
            )
        );

        // ── Right Bumper: AlignToTag (hold) ───────────────────────────────────
        // Snaps heading to the nearest tag while the driver controls translation.
        m_driverStick.rightBumper().whileTrue(
            new AlignToTag(
                drivetrain,
                visionSubsystem,
                () -> -m_driverStick.getLeftY() * kMaxSpeed,
                () -> -m_driverStick.getLeftX() * kMaxSpeed
            )
        );

        // ── B Button: Drive to Hub + Shoot (hold) ─────────────────────────────
        // Automatically drives to the hub, calculates the optimal shot angle,
        // pre-spins the flywheel, and fires when all conditions are met.
        // The command ends automatically after one shot; release early to abort.
        m_driverStick.b().whileTrue(
            new DriveToHubAndShootCommand(drivetrain, visionSubsystem, shooterSubsystem)
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

        // ── Y Button: Quick-turn 180° ─────────────────────────────────────────
        m_driverStick.y().onTrue(new TurnToAngle(drivetrain, 180, false));

        // ── D-Pad: Precision nudge ────────────────────────────────────────────
        m_driverStick.povUp()   .whileTrue(drivetrain.applyRequest(() -> m_drive.withVelocityX( kNudgeSpeed)));
        m_driverStick.povDown() .whileTrue(drivetrain.applyRequest(() -> m_drive.withVelocityX(-kNudgeSpeed)));
        m_driverStick.povLeft() .whileTrue(drivetrain.applyRequest(() -> m_drive.withVelocityY( kNudgeSpeed)));
        m_driverStick.povRight().whileTrue(drivetrain.applyRequest(() -> m_drive.withVelocityY(-kNudgeSpeed)));

        // ── Left Trigger: Climber up (manual, hold then hold-position) ────────
        // NOTE: was previously on leftBumper — moved to leftTrigger to free
        //       leftBumper for gyro reset (the original code had a conflict).
        m_driverStick.leftTrigger().whileTrue(
            Commands.run(() -> climberSubsystem.setPowerLevel(0.5), climberSubsystem)
        ).onFalse(
            Commands.runOnce(() -> {
                double pos = climberSubsystem.getCurrentPosition();
                climberSubsystem.setPositionDegrees(pos * 360.0);
            }, climberSubsystem)
        );

        // ── Right Trigger: Climber down (manual, hold then hold-position) ─────
        m_driverStick.rightTrigger().whileTrue(
            Commands.run(() -> climberSubsystem.setPowerLevel(-0.5), climberSubsystem)
        ).onFalse(
            Commands.runOnce(() -> {
                double pos = climberSubsystem.getCurrentPosition();
                climberSubsystem.setPositionDegrees(pos * 360.0);
            }, climberSubsystem)
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
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Re-seed field-centric so autonomous starts with the correct heading
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Drive forward for 5 seconds
            drivetrain.applyRequest(() ->
                m_drive
                    .withVelocityX(0.5)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0)
            ).withTimeout(5.0),
            drivetrain.applyRequest(() -> idle)
        );
    }
}