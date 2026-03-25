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
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import frc.robot.subsystems.Turret.TurretSubsystem;

// Commands
import frc.robot.commands.AutoClimbCommand;
import frc.robot.commands.DriveToHubAndShoot;
import frc.robot.commands.SnapAimAndShootCommand;
import frc.robot.commands.SnapHeadingToTag;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurretAutoAimCommand;

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
 *   B / X Button        — Drive to Hub + Shoot
 *   Left  Bumper        — Gyro reset
 *   A Button            — Emergency brake
 *   Y Button            — Quick turn to 45°
 *   D-Pad               — Precision nudge
 * </pre>
 *
 * <h3>Button map (Operator — port 1)</h3>
 * <pre>
 *   A Button             — Toggle intake (deploy + all rollers ↔ retract + 3s roller push)
 *   B Button             — Toggle feeder (waits for RPM, then runs ↔ stop)
 *   X Button             — D-pad modifier (hold while pressing D-pad for bank 2)
 *   Left  Trigger (hold) — Climber up,   hold position on release
 *   Right Trigger (hold) — Climber down, hold position on release
 *   Left  Bumper  (hold) — Eject (all intake motors reverse)
 *   Right Bumper         — Reverse feeder (toggle)
 *   Left  Stick  (press) — Snap-aim-and-shoot (turret locks to hub tag → spins up → fires)
 *   Right Stick  (press) — Toggle turret auto-aim (hub track w/ SOTM + vision blend)
 *   Y Button             — Zero turret (point straight ahead first)
 *
 *   D-Pad (no X held):
 *     Up    — Flywheel @ 1.5 m tape  (1729 RPM ★ confirmed)
 *     Right — Flywheel @ 2.0 m tape  (~1995 RPM)
 *     Down  — Flywheel @ 2.5 m tape  (2081 RPM ★ confirmed)
 *     Left  — Flywheel @ 3.0 m tape  (~2430 RPM)
 *   D-Pad (hold X):
 *     Up    — Flywheel @ 3.5 m tape  (~2620 RPM)
 *     Right — Flywheel @ 4.0 m tape  (2570 RPM ★ confirmed)
 *     Down  — Flywheel @ 4.5 m tape  (~2815 RPM)
 *     Left  — Flywheel @ 5.0 m tape  (interpolated)
 *   Hold D-pad to spin; release to idle flywheel.
 * </pre>
 */
public class RobotContainer {

    // ── Speed constants ───────────────────────────────────────────────────────
    private final double kMaxSpeed       = 4.5;  // m/s — normal max translation speed
    private final double kTurboSpeed     = 6.0;  // m/s — turbo max speed (right trigger)
    private final double kMaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // reduced for control

    // Left trigger turn sensitivity:
    //   Released (0.0) → scale = 1.0 (full turn rate)
    //   Fully pressed (1.0) → scale = kMinTurnScale (20%)
    private final double kMinTurnScale = 0.2;
    private final double kNudgeSpeed   = 0.15 * kMaxSpeed;

    // ── Joystick deadbands ────────────────────────────────────────────────────
    // (These were replaced with inline deaband values in drive requests)

    // ── Slew rate limiters (acceleration limiting) ────────────────────────────
    // Units: m/s² for translation, rad/s² for rotation.
    // Lower value = smoother but less responsive. Tune on carpet.
    private final SlewRateLimiter m_xLimiter   = new SlewRateLimiter(3.5);
    private final SlewRateLimiter m_yLimiter   = new SlewRateLimiter(3.5);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(6.0);

    // ── Current limits applied to drive and steer motors at runtime ───────────
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
    public final TurretSubsystem         turretSubsystem;

    // ── Constructor ───────────────────────────────────────────────────────────
    public RobotContainer() {
        // 1. Drivetrain MUST be first
        drivetrain = TunerConstants.createDrivetrain();
        applyDrivetrainLimits();

        // 2. Mechanism subsystems — all instantiated before Named Commands
        shooterSubsystem      = new ShooterSubsystem();
        feederSubsystem       = new FeederSubsystem();
        actuationSubsystem    = new ActuationSubsystem();
        uptakeSubsystem       = new UptakeSubsystem();
        intakeRollerSubsystem = new IntakeRollerSubsystem();
        intakeAdapter         = new IntakeAdapter(actuationSubsystem, uptakeSubsystem, intakeRollerSubsystem);
        climberSubsystem      = new ClimberSubsystem();

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

        // ── Flywheel ─────────────────────────────────────────────────────────
        NamedCommands.registerCommand("SpinUpFlywheel", Commands.runOnce(() -> {
            shooterSubsystem.setFlywheelRPM(ShotCalculator.OPTIMAL_SHOT.rpm());
            shooterSubsystem.setLaunchAngleDeg(ShotCalculator.OPTIMAL_SHOT.hoodDeg());
        }, shooterSubsystem));
        NamedCommands.registerCommand("IdleFlywheel",
            Commands.runOnce(shooterSubsystem::idleFlywheel, shooterSubsystem));

        // ── Intake ────────────────────────────────────────────────────────────
        // "StartIntake" — deploy arm + run all rollers (use until ball collected)
        NamedCommands.registerCommand("StartIntake",
            Commands.runOnce(intakeAdapter::run,
                actuationSubsystem, uptakeSubsystem, intakeRollerSubsystem));
        // "StopIntake" — retract arm + stop all rollers
        NamedCommands.registerCommand("StopIntake",
            Commands.runOnce(intakeAdapter::stop,
                actuationSubsystem, uptakeSubsystem, intakeRollerSubsystem));

        // ── Feeder ────────────────────────────────────────────────────────────
        // "StartFeeder" — run feeder (gate check done by operator; in auto pair with SpinUp_*)
        NamedCommands.registerCommand("StartFeeder",
            Commands.runOnce(() -> feederSubsystem.setPower(5.0), feederSubsystem));
        // "StopFeeder" — stop feeder roller
        NamedCommands.registerCommand("StopFeeder",
            Commands.runOnce(feederSubsystem::stop, feederSubsystem));

        // ── Flywheel spin-up presets (instant — flywheel keeps running until IdleFlywheel) ──
        // Pair with StartFeeder / StopFeeder for manual control in path events.
        NamedCommands.registerCommand("SpinUp_Close",   // 1.5 m
            Commands.runOnce(() -> shooterSubsystem.setFlywheelRPM(ShooterConstants.PRESET_CLOSE_RPM),
                shooterSubsystem));
        NamedCommands.registerCommand("SpinUp_Mid",     // 2.5 m
            Commands.runOnce(() -> shooterSubsystem.setFlywheelRPM(ShooterConstants.PRESET_MID_RPM),
                shooterSubsystem));
        NamedCommands.registerCommand("SpinUp_Far",     // 4.0 m
            Commands.runOnce(() -> shooterSubsystem.setFlywheelRPM(ShooterConstants.PRESET_FAR_RPM),
                shooterSubsystem));
        NamedCommands.registerCommand("SpinUp_VFar",    // 5.5 m
            Commands.runOnce(() -> shooterSubsystem.setFlywheelRPM(ShooterConstants.PRESET_VFAR_RPM),
                shooterSubsystem));

        // ── Full shoot sequences (spin up → wait for RPM → feed → stop) ──────
        // Each command is self-contained: set RPM, wait (≤2 s), feed all 8 balls, stop feeder.
        NamedCommands.registerCommand("Shoot_Close", makeShootCommand(ShooterConstants.PRESET_CLOSE_RPM, 8));
        NamedCommands.registerCommand("Shoot_Mid",   makeShootCommand(ShooterConstants.PRESET_MID_RPM,   8));
        NamedCommands.registerCommand("Shoot_Far",   makeShootCommand(ShooterConstants.PRESET_FAR_RPM,   8));
        NamedCommands.registerCommand("Shoot_VFar",  makeShootCommand(ShooterConstants.PRESET_VFAR_RPM,  8));

        // ── Auto Level 1 climb — add to end of any auto path ─────────────────
        NamedCommands.registerCommand("AutoClimbLevel1",
            new AutoClimbCommand(climberSubsystem).withTimeout(3.0));

        // 6. Build auto chooser
        m_autoChooser = new SendableChooser<>();
        m_autoChooser.setDefaultOption("Do Nothing", Commands.none());
        for (String name : AutoBuilder.getAllAutoNames()) {
            m_autoChooser.addOption(name, AutoBuilder.buildAuto(name));
        }
        SmartDashboard.putData("Auto Chooser", m_autoChooser);

        // 7. Remaining subsystems (vision + turret — depend on drivetrain being configured)
        visionSubsystem = new VisionSubsystem(
            new String[]{
                VisionHardware.CAMERA_BACK_LEFT,
                VisionHardware.CAMERA_BACK_RIGHT
            },
            drivetrain
        );
        turretSubsystem = new TurretSubsystem();

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

                // Get raw stick inputs
                double rawX = -m_driverStick.getLeftY() * translationSpeed;
                double rawY = -m_driverStick.getLeftX() * translationSpeed;
                double rawRot = -MathUtil.applyDeadband(m_driverStick.getRightX(), 0.15)
                    * kMaxAngularRate * turnScale;

                // Apply slew rate limiting for smoother acceleration
                double limitedX = m_xLimiter.calculate(rawX);
                double limitedY = m_yLimiter.calculate(rawY);
                double limitedRot = m_rotLimiter.calculate(rawRot);

                Logger.recordOutput("Driver/TurnScale",        turnScale);
                Logger.recordOutput("Driver/TranslationSpeed", translationSpeed);

                return m_drive
                    .withVelocityX(limitedX)
                    .withVelocityY(limitedY)
                    .withRotationalRate(limitedRot);
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
        actuationSubsystem.setDefaultCommand(
            Commands.run(() -> {}, actuationSubsystem) // no-op: lets MotionMagic finish after commands end
        );

        // A: Toggle intake — deploy arm + spin all rollers  ↔  retract + 3s roller push then stop
        m_playerStick.a().toggleOnTrue(
    Commands.startEnd(
        intakeAdapter::run,
        intakeAdapter::stop,
        actuationSubsystem, uptakeSubsystem, intakeRollerSubsystem
    )
);

        // B: Toggle feeder — waits for flywheel to reach speed, then latches on until toggled off
        m_playerStick.b().toggleOnTrue(
            Commands.sequence(
                Commands.waitUntil(shooterSubsystem::isReadyToShoot),
                Commands.startEnd(
                    () -> feederSubsystem.setPower(0.55),
                    feederSubsystem::stop,
                    feederSubsystem
                )
            )
        );

        // X: hold as D-pad modifier (see D-Pad section below — no standalone action)

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
                () -> feederSubsystem.setPower(-0.40),
                feederSubsystem::stop,
                feederSubsystem
            )
        );

        // ── Operator D-Pad: distance-based RPM presets (hold to spin, release to idle) ─
        // Without X:  Up=1.5m  Right=2.0m  Down=2.5m  Left=3.0m
        // Hold X:     Up=3.5m  Right=4.0m  Down=4.5m  Left=5.0m
        // RPM interpolated live from RPM_DISTANCE_TABLE.
        m_playerStick.povUp().and(m_playerStick.x().negate()).whileTrue(
            Commands.startEnd(
                () -> shooterSubsystem.setFlywheelRPMFromDistance(ShooterConstants.PRESET_CLOSE_DIST_M),
                shooterSubsystem::idleFlywheel, shooterSubsystem));
        m_playerStick.povRight().and(m_playerStick.x().negate()).whileTrue(
            Commands.startEnd(
                () -> shooterSubsystem.setFlywheelRPMFromDistance(ShooterConstants.PRESET_2M_DIST_M),
                shooterSubsystem::idleFlywheel, shooterSubsystem));
        m_playerStick.povDown().and(m_playerStick.x().negate()).whileTrue(
            Commands.startEnd(
                () -> shooterSubsystem.setFlywheelRPMFromDistance(ShooterConstants.PRESET_MID_DIST_M),
                shooterSubsystem::idleFlywheel, shooterSubsystem));
        m_playerStick.povLeft().and(m_playerStick.x().negate()).whileTrue(
            Commands.startEnd(
                () -> shooterSubsystem.setFlywheelRPMFromDistance(ShooterConstants.PRESET_3M_DIST_M),
                shooterSubsystem::idleFlywheel, shooterSubsystem));

        m_playerStick.povUp().and(m_playerStick.x()).whileTrue(
            Commands.startEnd(
                () -> shooterSubsystem.setFlywheelRPMFromDistance(ShooterConstants.PRESET_3_5M_DIST_M),
                shooterSubsystem::idleFlywheel, shooterSubsystem));
        m_playerStick.povRight().and(m_playerStick.x()).whileTrue(
            Commands.startEnd(
                () -> shooterSubsystem.setFlywheelRPMFromDistance(ShooterConstants.PRESET_FAR_DIST_M),
                shooterSubsystem::idleFlywheel, shooterSubsystem));
        m_playerStick.povDown().and(m_playerStick.x()).whileTrue(
            Commands.startEnd(
                () -> shooterSubsystem.setFlywheelRPMFromDistance(ShooterConstants.PRESET_4_5M_DIST_M),
                shooterSubsystem::idleFlywheel, shooterSubsystem));
        m_playerStick.povLeft().and(m_playerStick.x()).whileTrue(
            Commands.startEnd(
                () -> shooterSubsystem.setFlywheelRPMFromDistance(ShooterConstants.PRESET_5M_DIST_M),
                shooterSubsystem::idleFlywheel, shooterSubsystem));

        // ── Turret: right joystick X = manual drive, Y button = zero ─────────
        // Right joystick X on operator controller drives the turret open-loop.
        // Use this to find your hard-stop angles — watch "Angle (deg)" in Elastic.
        turretSubsystem.setDefaultCommand(
            Commands.run(
                () -> turretSubsystem.setOpenLoop(
                    MathUtil.applyDeadband(-m_playerStick.getRightX(), 0.1) * 0.3
                ),
                turretSubsystem
            )
        );
        // Y: zero turret — point straight ahead then press this
        m_playerStick.y().onTrue(
            Commands.runOnce(turretSubsystem::zeroPosition, turretSubsystem)
        );

        // Left Stick (LS/L3): Snap-aim-and-shoot — turret locks to hub tag, spins up, fires one ball
        m_playerStick.leftStick().whileTrue(
            new SnapAimAndShootCommand(drivetrain, shooterSubsystem, turretSubsystem, feederSubsystem, visionSubsystem)
        );

        // Right Stick (RS/R3): Toggle turret auto-aim
        // Active: turret tracks velocity-compensated hub via FireControlSolver (SOTM + rotation cancel).
        // Inactive: right joystick X manual control resumes automatically.
        m_playerStick.rightStick().toggleOnTrue(
            new TurretAutoAimCommand(drivetrain, shooterSubsystem, turretSubsystem, visionSubsystem)
        );

        // ── Telemetry hook ────────────────────────────────────────────────────
        drivetrain.registerTelemetry(m_logger::telemeterize);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    /**
     * Builds a self-contained shoot command for PathPlanner named commands.
     * Sequence: set RPM → wait until at speed (≤2 s timeout) → run feeder for all balls → stop feeder.
     * Feed time = balls × 0.25 s per ball + 0.5 s buffer.
     * Calibrated from observed cycle rate: ~2 balls per 0.5 s on the robot.
     * The flywheel keeps spinning after this command; follow with "IdleFlywheel" when done.
     */
    private Command makeShootCommand(double targetRpm, int balls) {
        double feedSeconds = balls * 0.25 + 0.5;
        return Commands.sequence(
            Commands.runOnce(() -> shooterSubsystem.setFlywheelRPM(targetRpm), shooterSubsystem),
            Commands.waitUntil(() -> shooterSubsystem.isAtTargetRPM(targetRpm)).withTimeout(2.0),
            Commands.runOnce(() -> feederSubsystem.setPower(0.55), feederSubsystem),
            Commands.waitSeconds(feedSeconds),
            Commands.runOnce(feederSubsystem::stop, feederSubsystem)
        );
    }

    /** Returns the robot's current heading in degrees (field-relative). */
    public double getCurrentRotation() {
        return drivetrain.getState().Pose.getRotation().getDegrees();
    }

    // ── Autonomous ────────────────────────────────────────────────────────────

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}