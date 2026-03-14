package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;

// Constants
import frc.robot.Constants.DriveToPoseConstants;
import frc.robot.Constants.ShooterConstants;
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
import frc.robot.subsystems.Intake.IntakeRollerSubsystem;
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
 *   Left  Trigger       — Turn sensitivity reduction (0% = full, 100% = 20%)
 *   Right Trigger       — Turbo mode (boosts translation speed for bumps)
 *   Right Bumper        — Auto-align to AprilTag (hold)
 *   B Button            — Drive to Hub + Shoot
 *   Left  Bumper        — Gyro reset
 *   A Button            — Emergency brake
 *   Y Button            — Quick turn to 45°
 *   D-Pad               — Precision nudge
 * </pre>
 *
 * <h3>Button map (Operator — port 1)</h3>
 * <pre>
 *   A Button             — Toggle intake (deploy + all rollers ↔ retract + 3s roller push)
 *   B Button             — Toggle feeder (run forward ↔ stop)
 *   X Button             — Toggle shooter flywheel
 *   Left Trigger  (hold) — Climber up,   hold position on release
 *   Right Trigger (hold) — Climber down, hold position on release
 *   Left Bumper          — Eject (all intake motors reverse, hold)
 *   Right Bumper         — Reverse feeder (hold)
 * </pre>
 */
public class RobotContainer {

    // ── Speed constants ───────────────────────────────────────────────────────
    private final double kMaxSpeed       = 5.0;  // m/s — normal max translation speed
    private final double kTurboSpeed     = 6.5;  // m/s — turbo max speed (right trigger)
    private final double kMaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    // Left trigger turn sensitivity:
    //   Released (0.0) → scale = 1.0 (full turn rate)
    //   Fully pressed (1.0) → scale = kMinTurnScale (20%)
    private final double kMinTurnScale = 0.2;
    private final double kNudgeSpeed   = 0.2 * kMaxSpeed;

    // Current limits applied to drive and steer motors at runtime
    private static final double kDriveStatorLimit = 60.0;
    private static final double kDriveSupplyLimit = 40.0;
    private static final double kSteerStatorLimit = 30.0;
    private static final double kSteerSupplyLimit = 20.0;

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

    /** Field2d for PathPlanner path display in AdvantageScope. */
    private final Field2d m_pathPlannerField = new Field2d();

    // ── Subsystems ────────────────────────────────────────────────────────────
    public final CommandSwerveDrivetrain drivetrain;
    public final VisionSubsystem         visionSubsystem;
    public final ShooterSubsystem        shooterSubsystem;
    
    public final ClimberSubsystem        climberSubsystem;
    public final FeederSubsystem         feederSubsystem;
    public final ActuationSubsystem      actuationSubsystem;
    public final UptakeSubsystem         uptakeSubsystem;
    public final IntakeRollerSubsystem   intakeRollerSubsystem;
    public final IntakeAdapter           intakeAdapter;

    // ── Constructor ───────────────────────────────────────────────────────────
    public RobotContainer() {
        // 1. Drivetrain MUST be first
        drivetrain = TunerConstants.createDrivetrain();
        applyDrivetrainLimits();

        // 2. Shooter early — needed by Named Commands
        shooterSubsystem = new ShooterSubsystem();
        feederSubsystem  = new FeederSubsystem();
        // 3. Configure PathPlanner AutoBuilder
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                () -> drivetrain.getState().Pose,
                drivetrain::resetPose,
                () -> drivetrain.getState().Speeds,
                (speeds, feedforwards) -> drivetrain.driveRobotRelative(speeds),
                new PPHolonomicDriveController(
                    new PIDConstants(DriveToPoseConstants.kP_XY,    0, DriveToPoseConstants.kD_XY),
                    new PIDConstants(DriveToPoseConstants.kP_THETA, 0, DriveToPoseConstants.kD_THETA)
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                drivetrain
            );
        } catch (Exception e) {
            DriverStation.reportError("AutoBuilder config failed: " + e.getMessage(), e.getStackTrace());
        }

        // 4. PathPlanner → AdvantageScope logging
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

        // 5. Register Named Commands — before buildAuto()
        NamedCommands.registerCommand("SpinUpFlywheel", Commands.runOnce(() -> {
            shooterSubsystem.setFlywheelRPM(ShotCalculator.OPTIMAL_SHOT.rpm());
            shooterSubsystem.setLaunchAngleDeg(ShotCalculator.OPTIMAL_SHOT.hoodDeg());
        }, shooterSubsystem));

        NamedCommands.registerCommand("IdleFlywheel",
            Commands.runOnce(shooterSubsystem::idleFlywheel, shooterSubsystem));

        NamedCommands.registerCommand("Shoot",
            Commands.sequence(
            Commands.runOnce(() ->
                shooterSubsystem.setFlywheelRPM(ShooterConstants.FIXED_SHOT_RPM_M),
                shooterSubsystem),

            Commands.waitUntil(() -> shooterSubsystem.isAtTargetRPM(ShooterConstants.FIXED_SHOT_RPM_M)),

            Commands.runOnce(() -> feederSubsystem.setPower(5.0), feederSubsystem),

            Commands.waitSeconds(0.5),

            Commands.runOnce(() -> {
                feederSubsystem.stop();
                shooterSubsystem.idleFlywheel();
            }, shooterSubsystem, feederSubsystem)
        )
    );
        // 6. Build auto chooser
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
                VisionHardware.CAMERA_BACK_RIGHT
            },
            drivetrain
        );
        climberSubsystem      = new ClimberSubsystem();
        actuationSubsystem    = new ActuationSubsystem();
        uptakeSubsystem       = new UptakeSubsystem();
        intakeRollerSubsystem = new IntakeRollerSubsystem();  // IDs 27 & 28
        intakeAdapter         = new IntakeAdapter(actuationSubsystem, uptakeSubsystem, intakeRollerSubsystem);

        // Publish shot-calculator results once
        SmartDashboard.putNumber("DTHS/OptimalDist_m",    ShotCalculator.OPTIMAL_STANDOFF_M);
        SmartDashboard.putNumber("DTHS/OptimalAngle_deg", ShotCalculator.OPTIMAL_SHOT.hoodDeg());
        SmartDashboard.putNumber("DTHS/OptimalRPM",       ShotCalculator.OPTIMAL_SHOT.rpm());
        SmartDashboard.putNumber("DTHS/OptimalEntry_deg", ShotCalculator.OPTIMAL_SHOT.entryAngle());

        configureBindings();
    }

    // ── Current limit application ─────────────────────────────────────────────

    private void applyDrivetrainLimits() {
        CurrentLimitsConfigs driveLimits = new CurrentLimitsConfigs();
        driveLimits.StatorCurrentLimit       = kDriveStatorLimit;
        driveLimits.StatorCurrentLimitEnable = true;
        driveLimits.SupplyCurrentLimit       = kDriveSupplyLimit;
        driveLimits.SupplyCurrentLimitEnable = true;

        CurrentLimitsConfigs steerLimits = new CurrentLimitsConfigs();
        steerLimits.StatorCurrentLimit       = kSteerStatorLimit;
        steerLimits.StatorCurrentLimitEnable = true;
        steerLimits.SupplyCurrentLimit       = kSteerSupplyLimit;
        steerLimits.SupplyCurrentLimitEnable = true;

        for (int i = 0; i < 4; i++) {
            drivetrain.getModule(i).getDriveMotor().getConfigurator().apply(driveLimits);
            drivetrain.getModule(i).getSteerMotor().getConfigurator().apply(steerLimits);
        }
    }

    // ── Trigger helpers ───────────────────────────────────────────────────────

    /** Left trigger → turn rate scale. Released = 1.0, fully pressed = kMinTurnScale. */
    private double getTurnScale() {
        double trigger = m_driverStick.getLeftTriggerAxis();
        return 1.0 - trigger * (1.0 - kMinTurnScale);
    }

    /** Right trigger → translation speed. Released = kMaxSpeed, fully pressed = kTurboSpeed. */
    private double getTranslationSpeed() {
        double trigger = m_driverStick.getRightTriggerAxis();
        return kMaxSpeed + trigger * (kTurboSpeed - kMaxSpeed);
    }

    // ── Button bindings ───────────────────────────────────────────────────────

    private void configureBindings() {

        // ── Default: field-centric teleop ─────────────────────────────────────
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double translationSpeed = getTranslationSpeed();
                double turnScale        = getTurnScale();

                Logger.recordOutput("Driver/TurnScale",        turnScale);
                Logger.recordOutput("Driver/TranslationSpeed", translationSpeed);

                return m_drive
                    .withVelocityX(-m_driverStick.getLeftY() * translationSpeed)
                    .withVelocityY(-m_driverStick.getLeftX() * translationSpeed)
                    .withRotationalRate(
                        -MathUtil.applyDeadband(m_driverStick.getRightX(), 0.15)
                        * kMaxAngularRate * turnScale
                    );
            })
        );

        // ── Right Bumper: Continuous heading alignment to AprilTag ────────────
        m_driverStick.rightBumper().whileTrue(
            new SnapHeadingToTag(
                drivetrain,
                visionSubsystem,
                () -> -m_driverStick.getLeftY() * getTranslationSpeed(),
                () -> -m_driverStick.getLeftX() * getTranslationSpeed()
            )
        );

        // ── B Button: Drive to hub + shoot ───────────────────────────────────
        m_driverStick.b().whileTrue(
            new DriveToHubAndShoot(drivetrain, shooterSubsystem)
        );

        // ── Left Bumper: Gyro reset ───────────────────────────────────────────
        m_driverStick.leftBumper().onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric)
        );

        // ── A Button: Emergency brake ─────────────────────────────────────────
        m_driverStick.a().whileTrue(
            drivetrain.applyRequest(() -> m_brake)
        );

        // ── X Button: Drive to Hub + Shoot ───────────────────────────────────
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

        // Default: arm holds position passively via brake mode — no whirring
        // RobotContainer — replace the default command
    actuationSubsystem.setDefaultCommand(
        Commands.run(() -> {}, actuationSubsystem) // idle, no-op — just holds requirement
    );

        // A: Toggle intake — deploy arm + spin all rollers  ↔  retract + 3s roller push then stop
        m_playerStick.a().toggleOnTrue(
    Commands.sequence(
        // Deploy arm + start rollers
        Commands.runOnce(() -> {
            actuationSubsystem.setPosition(11.33);
            uptakeSubsystem.setPower(0.8);
            intakeRollerSubsystem.setPower(0.8);
        }, actuationSubsystem, uptakeSubsystem, intakeRollerSubsystem),

        // Wait until arm is fully deployed
        Commands.waitUntil(() ->
            Math.abs(actuationSubsystem.getCurrentPosition() - 11.33) < 0.2),

        // Keep rollers spinning until toggled off
        Commands.run(() -> {
            uptakeSubsystem.setPower(0.8);
            intakeRollerSubsystem.setPower(0.8);
        }, uptakeSubsystem, intakeRollerSubsystem)
    ).finallyDo(() -> {
        // On toggle off — stop rollers and retract
        uptakeSubsystem.stopMotors();
        intakeRollerSubsystem.stop();
        actuationSubsystem.setPosition(0.0);
    })
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
        () -> shooterSubsystem.setFlywheelRPM(ShooterConstants.FIXED_SHOT_RPM_M),
        shooterSubsystem::idleFlywheel,
        shooterSubsystem
    )
);

        // Left Trigger (operator): Climber up (hold), hold position on release
        m_playerStick.leftTrigger().whileTrue(
            Commands.run(() -> climberSubsystem.setPowerLevel(0.5), climberSubsystem)
        ).onFalse(
            Commands.runOnce(() -> {
                double pos = climberSubsystem.getCurrentPosition();
                climberSubsystem.setPositionDegrees(pos * 360.0);
            }, climberSubsystem)
        );

        // Right Trigger (operator): Climber down (hold), hold position on release
        m_playerStick.rightTrigger().whileTrue(
            Commands.run(() -> climberSubsystem.setPowerLevel(-0.5), climberSubsystem)
        ).onFalse(
            Commands.runOnce(() -> {
                double pos = climberSubsystem.getCurrentPosition();
                climberSubsystem.setPositionDegrees(pos * 360.0);
            }, climberSubsystem)
        );

        // Left Bumper: Eject — all intake motors reverse (hold)
        m_playerStick.leftBumper().whileTrue(
            Commands.run(intakeAdapter::eject, actuationSubsystem, uptakeSubsystem, intakeRollerSubsystem)
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