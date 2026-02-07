package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * This command will turn the robot to a specified angle.
 */
public class TurnToAngle extends Command{

    private CommandSwerveDrivetrain swerve;
    private boolean isRelative;
    private double goal;
    private HolonomicDriveController holonomicDriveController;
    private Pose2d startPos = new Pose2d();
    private Pose2d targetPose2d = new Pose2d();

    /**
     * Turns robot to specified angle. Uses absolute rotation on field.
     *
     * @param swerve Swerve subsystem
     * @param angle Requested angle to turn to
     * @param isRelative Whether the angle is relative to the current angle: true = relative, false
     *        = absolute
     */

    public TurnToAngle(CommandSwerveDrivetrain swerve, double angle, boolean isRelative) {
        addRequirements(swerve);
        this.swerve = swerve;
        this.goal = angle;
        this.isRelative = isRelative;

        PIDController xcontroller = new PIDController(1, 0, 0);
        PIDController ycontroller = new PIDController(1, 0, 0);
        ProfiledPIDController thetacontroller =
            new ProfiledPIDController(4, 0, 0, new Constraints(2, 2));
        holonomicDriveController =
            new HolonomicDriveController(xcontroller, ycontroller, thetacontroller);
        holonomicDriveController.setTolerance(new Pose2d(1, 1, Rotation2d.fromDegrees(1)));

    }

    @Override
    public void initialize() {
        swerve.resetPose(new Pose2d(swerve.getState().Pose.getTranslation(), Rotation2d.kZero));
        startPos = swerve.getState().Pose;
        
        if (isRelative) {
            targetPose2d = new Pose2d(startPos.getTranslation(),
                startPos.getRotation().rotateBy(Rotation2d.fromDegrees(goal)));
        } else {
            targetPose2d = new Pose2d(startPos.getTranslation(), Rotation2d.fromDegrees(goal));
        }
    }

    @Override
    public void execute() {
        Pose2d currPose2d = swerve.getState().Pose;
        ChassisSpeeds chassisSpeeds = this.holonomicDriveController.calculate(currPose2d,
            targetPose2d, 0, targetPose2d.getRotation());
        swerve.setControl(new SwerveRequest.RobotCentric()
            .withVelocityX(chassisSpeeds.vxMetersPerSecond)
            .withVelocityY(chassisSpeeds.vyMetersPerSecond)
            .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond));
        // System.out.println(targetPose2d.relativeTo(currPose2d));
    }

    @Override
    public void end(boolean interrupt) {
        System.out.println("end" + interrupt);
        swerve.setControl(new SwerveRequest.RobotCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
        // Reset the rotation to 0 after turning
        swerve.resetPose(new Pose2d(swerve.getState().Pose.getTranslation(), Rotation2d.kZero));
    }

    @Override
    public boolean isFinished() {
        System.out.println("is finsihed" + holonomicDriveController.atReference());
        return holonomicDriveController.atReference();

}
}
