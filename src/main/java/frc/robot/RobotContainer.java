package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//Constants
import frc.robot.generated.TunerConstants;


//Subsystems
//Drive Train
import frc.robot.subsystems.CommandSwerveDrivetrain;
//Vision
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.commands.AlignToTag;
import frc.robot.commands.TurnToAngle;
//Climber
import frc.robot.subsystems.Climber.ClimberSubsystem;
public class RobotContainer {
    // Speed Constants
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final double NudgeSpeed = 0.2 * MaxSpeed; 
    
    // Swerve Requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1) // Lowered deadband slightly for more precision
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers
    private final CommandXboxController DriverStick = new CommandXboxController(0);
    @SuppressWarnings("unused")
    private final CommandXboxController PlayerStick = new CommandXboxController(1);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain;
    public final VisionSubsystem visionSubsystem;
    public final ClimberSubsystem climberSubsystem;

    public RobotContainer() {
        // 1. Drivetrain MUST be initialized first
        drivetrain = TunerConstants.createDrivetrain();
        
        // 2. Initialize Vision and pass the drivetrain into it for Pose Estimation
        visionSubsystem = new VisionSubsystem(
            new String[] {"Camera-BackLeft", "Camera-BackRight"},
            drivetrain 
        );
        // 3. Initialize Climber
        climberSubsystem = new ClimberSubsystem();

        configureBindings();
    }
    
    private void configureBindings() {
        // Default Command: Standard Field-Centric Teleop
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-DriverStick.getLeftY() * MaxSpeed)
                     .withVelocityY(-DriverStick.getLeftX() * MaxSpeed)
                     .withRotationalRate(-DriverStick.getRightX() * MaxAngularRate)
            )
        );

        // --- Driver Actions ---

        // Align to Tag: Uses Pose Estimation to "Snap" to target while maintaining driver translation control
        DriverStick.rightBumper().whileTrue(
            new AlignToTag(
                drivetrain,
                visionSubsystem,
                () -> -DriverStick.getLeftY() * MaxSpeed,
                () -> -DriverStick.getLeftX() * MaxSpeed
            )
        );

        // Reset Gyro / Field Centricity
        DriverStick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Emergency Brake
        DriverStick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Quick Turn: Face 180 degrees
        DriverStick.y().onTrue(new TurnToAngle(drivetrain, 180, false));

        // Precision Nudging (D-Pad)
        DriverStick.povUp().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(NudgeSpeed)));
        DriverStick.povDown().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-NudgeSpeed)));
        DriverStick.povLeft().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityY(NudgeSpeed)));
        DriverStick.povRight().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityY(-NudgeSpeed)));

        // Telemetry
        drivetrain.registerTelemetry(logger::telemeterize);

        //Elevator bindings
        // 1. Left Trigger (Operator): Manual Up (using power)
        DriverStick.leftBumper().whileTrue(
            Commands.run(() -> climberSubsystem.setPowerLevel(0.5), climberSubsystem)
        ).onFalse(
        Commands.runOnce(() -> {
            double currentRotations = climberSubsystem.getCurrentPosition();
            // setPositionDegrees expects degrees, so we convert rotations back to degrees
            climberSubsystem.setPositionDegrees(currentRotations * 360.0);
        }, climberSubsystem)
        );

        // 2. Right Trigger (or X button): Manual Down. On release, hold position.
// Note: Changed from Left Bumper to avoid conflict with Gyro Reset
        DriverStick.leftTrigger().whileTrue(
             Commands.run(() -> climberSubsystem.setPowerLevel(-0.5), climberSubsystem)
        ).onFalse(
            Commands.runOnce(() -> {
            double currentRotations = climberSubsystem.getCurrentPosition();
            climberSubsystem.setPositionDegrees(currentRotations * 360.0);
        }, climberSubsystem)
        );
    }

    public double getCurrentRotation() {
        return drivetrain.getState().Pose.getRotation().getDegrees();
    }

    public Command getAutonomousCommand() {
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                     .withVelocityY(0)
                     .withRotationalRate(0)
            ).withTimeout(5.0),
            drivetrain.applyRequest(() -> idle)
        );
    }
}