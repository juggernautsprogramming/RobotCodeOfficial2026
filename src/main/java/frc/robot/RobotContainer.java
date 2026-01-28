// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double NudgeSpeed = 0.2 * MaxSpeed; 

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController DriverStick = new CommandXboxController(0);
    private final CommandXboxController PlayerStick = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(DriverStick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(DriverStick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-DriverStick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        DriverStick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        DriverStick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-DriverStick.getLeftY(), -DriverStick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        DriverStick.back().and(DriverStick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        DriverStick.back().and(DriverStick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        DriverStick.start().and(DriverStick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        DriverStick.start().and(DriverStick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        DriverStick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        //Nudge Control
        DriverStick.povUp().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityX(-NudgeSpeed).withVelocityY(0).withRotationalRate(0))
        );
        DriverStick.povDown().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityX(NudgeSpeed).withVelocityY(0).withRotationalRate(0))
        );
        DriverStick.povLeft().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(-NudgeSpeed).withRotationalRate(0))
        );
        DriverStick.povRight().whileTrue(
            drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(NudgeSpeed).withRotationalRate(0))
        );
        
    }
    public double getCurrentRotation() {
        Rotation2d currentRotation = drivetrain.getState().Pose.getRotation();
        double robotDegrees = currentRotation.getDegrees();
        return robotDegrees;
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
    public Command do180turn() {
        return drivetrain.applyRequest(() -> 
            facingAngleRequest
            .withVelocityX(0)
            .withVelocityY(0)
            // We take the current rotation and add 180 degrees
            .withTargetDirection(drivetrain.getState().Pose.getRotation().plus(Rotation2d.k180deg))
    )
    // This tells the command to stop once the robot is within 2 degrees of the target
        .until(() -> Math.abs(drivetrain.getState().Pose.getRotation().getDegrees() % 360) > 178); 
    }
}
