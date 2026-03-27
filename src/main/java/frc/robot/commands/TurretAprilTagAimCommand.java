package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

/**
 * TurretAprilTagAimCommand — aims the turret purely from the hub AprilTag's
 * camera-relative yaw. No odometry or FireControlSolver involved.
 *
 * <p>While a hub tag is visible, the turret is commanded to the camera yaw
 * (adjusted to robot frame). When the tag disappears the turret holds the
 * last known angle until a tag reappears or the command ends.
 *
 * <p>Uses robot angular velocity feedforward to maintain lock while the robot rotates.
 *
 * <p>Bind with {@code toggleOnTrue}.
 */
public class TurretAprilTagAimCommand extends Command {

    private final TurretSubsystem       m_turret;
    private final VisionSubsystem       m_vision;
    private final CommandSwerveDrivetrain m_drivetrain;

    private double  m_lastValidDeg = 0.0;
    private boolean m_hasTarget    = false;

    public TurretAprilTagAimCommand(TurretSubsystem turret, VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        m_turret     = turret;
        m_vision     = vision;
        m_drivetrain = drivetrain;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        m_hasTarget    = false;
        m_lastValidDeg = m_turret.getAngleDeg(); // start from wherever turret currently is
        SmartDashboard.putBoolean("TurretTagAim/Active",    true);
        SmartDashboard.putBoolean("TurretTagAim/HasTarget", false);
    }

    @Override
    public void execute() {
        // Get robot's angular velocity in rad/s (positive = CCW)
        double robotAngularVelRadPerSec = m_drivetrain.getState().Speeds.omegaRadiansPerSecond;

        if (m_vision.getBestHubTarget() != null) {
            double rawYaw = m_vision.getHubTagYawDeg();
            if (!Double.isNaN(rawYaw)) {
                // Camera yaw is the error from camera center to the tag.
                // Add to current turret angle to get the new absolute robot-frame target.
                double turretDeg = MathUtil.inputModulus(
                    m_turret.getAngleDeg() + rawYaw, -180.0, 180.0);
                
                // Use velocity feedforward to counter-rotate with the robot
                m_turret.setAngleDegWithFF(turretDeg, robotAngularVelRadPerSec);
                m_lastValidDeg = turretDeg;
                m_hasTarget    = true;

                SmartDashboard.putNumber ("TurretTagAim/RawYawDeg",     rawYaw);
                SmartDashboard.putNumber ("TurretTagAim/TurretTargetDeg", turretDeg);
                SmartDashboard.putBoolean("TurretTagAim/HasTarget",     true);
                SmartDashboard.putNumber ("TurretTagAim/RobotOmegaRadPerSec", robotAngularVelRadPerSec);
            }
        } else {
            // Tag lost — hold last known angle with feedforward
            if (m_hasTarget) {
                m_turret.setAngleDegWithFF(m_lastValidDeg, robotAngularVelRadPerSec);
            }
            SmartDashboard.putBoolean("TurretTagAim/HasTarget", false);
            SmartDashboard.putNumber ("TurretTagAim/RobotOmegaRadPerSec", robotAngularVelRadPerSec);
        }

        SmartDashboard.putNumber ("TurretTagAim/TurretActualDeg", m_turret.getAngleDeg());
        SmartDashboard.putNumber ("TurretTagAim/ErrorDeg",
            m_lastValidDeg - m_turret.getAngleDeg());
        SmartDashboard.putBoolean("TurretTagAim/AtTarget", m_turret.isAtTarget());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.setAngleDeg(m_turret.getAngleDeg()); // hold on handoff
        SmartDashboard.putBoolean("TurretTagAim/Active",    false);
        SmartDashboard.putBoolean("TurretTagAim/HasTarget", false);
    }
}
