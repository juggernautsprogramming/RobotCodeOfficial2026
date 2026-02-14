package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain; // Adjust based on your drivetrain class name
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignToTag extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final VisionSubsystem m_vision;
    private final DoubleSupplier m_translationX;
    private final DoubleSupplier m_translationY;

    // Local PID controller for rotation
    private final PIDController m_turnPID = new PIDController(
        VisionConstants.kP, 
        VisionConstants.kI, 
        VisionConstants.kD
    );

    // Swerve request to modify
    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();

    /**
     * @param drivetrain The swerve subsystem
     * @param vision The vision subsystem
     * @param translationX Supplier for forward/backward speed (usually joystick Y)
     * @param translationY Supplier for strafe speed (usually joystick X)
     */
    public AlignToTag(
        CommandSwerveDrivetrain drivetrain, 
        VisionSubsystem vision,
        DoubleSupplier translationX,
        DoubleSupplier translationY) {
        
        this.m_drivetrain = drivetrain;
        this.m_vision = vision;
        this.m_translationX = translationX;
        this.m_translationY = translationY;

        m_turnPID.setTolerance(VisionConstants.angleTolerance);
        
        // We require the drivetrain because we are moving the robot
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_turnPID.reset();
    }

    @Override
    public void execute() {
        // Inside AlignToTag.java execute()
        double rotationSpeed = 0;
        if (m_vision.hasValidTarget()) {
            double currentYaw = m_vision.getBestTarget().getYaw();
            // Only calculate if the error is greater than our tolerance
            if (Math.abs(currentYaw) > 0.5) {
                rotationSpeed = m_turnPID.calculate(currentYaw, 0);

                 // Add a "Feedforward" or Minimum Power
                double kS = 0.05; // Minimum volts/percent to break friction
                rotationSpeed += Math.copySign(kS, rotationSpeed);
            }
        }

        // Apply inputs to the swerve drivetrain
        m_drivetrain.setControl(
            m_driveRequest
                .withVelocityX(m_translationX.getAsDouble())
                .withVelocityY(m_translationY.getAsDouble())
                .withRotationalRate(rotationSpeed)
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Optional: stop the robot when the command ends
        m_drivetrain.setControl(new SwerveRequest.Idle());
    }
}