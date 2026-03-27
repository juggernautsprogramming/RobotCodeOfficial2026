package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain; // Adjust based on your drivetrain class name
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignToTag extends Command {

    // Tag IDs shared with SnapAimAndShootCommand — defined in VisionConstants.ALIGN_TAG_IDS

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
        double rotationSpeed = 0;

        var target = m_vision.getBestTarget();
        if (target != null && isAlignTag(target.getFiducialId())) {
            rotationSpeed = Math.max(-3.0, Math.min(3.0,
                m_turnPID.calculate(target.getYaw(), 0.0)));

            // Dampen if already rotating fast to prevent overshoot
            double currentRotVelocity = m_drivetrain.getState().Speeds.omegaRadiansPerSecond;
            if (Math.abs(currentRotVelocity) > 2.0) {
                rotationSpeed *= 0.5;
            }
        }

        m_drivetrain.setControl(
            m_driveRequest
                .withVelocityX(m_translationX.getAsDouble())
                .withVelocityY(m_translationY.getAsDouble())
                .withRotationalRate(rotationSpeed)
                .withDeadband(0.02) // Prevents joystick drift from fighting the PID
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Optional: stop the robot when the command ends
        m_drivetrain.setControl(new SwerveRequest.Idle());
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Returns true if the given tag ID is in VisionConstants.ALIGN_TAG_IDS. */
    public static boolean isAlignTag(int id) {
        for (int alignId : frc.robot.Constants.VisionConstants.ALIGN_TAG_IDS) {
            if (id == alignId) return true;
        }
        return false;
    }
}