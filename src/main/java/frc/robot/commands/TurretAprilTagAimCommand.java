package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

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
 * <p>Bind with {@code toggleOnTrue}.
 */
public class TurretAprilTagAimCommand extends Command {

    private final TurretSubsystem m_turret;
    private final VisionSubsystem m_vision;

    private double  m_lastValidDeg = 0.0;
    private boolean m_hasTarget    = false;

    public TurretAprilTagAimCommand(TurretSubsystem turret, VisionSubsystem vision) {
        m_turret = turret;
        m_vision = vision;
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
        if (m_vision.getBestHubTarget() != null) {
            double rawYaw = m_vision.getHubTagYawDeg();
            if (!Double.isNaN(rawYaw)) {
                // Camera yaw is already in robot-relative frame for a forward-facing camera.
                // Clamp to ±180° for shortest-arc tracking.
                double turretDeg = MathUtil.inputModulus(rawYaw, -180.0, 180.0);
                m_turret.setAngleDeg(turretDeg);
                m_lastValidDeg = turretDeg;
                m_hasTarget    = true;

                SmartDashboard.putNumber ("TurretTagAim/RawYawDeg",     rawYaw);
                SmartDashboard.putNumber ("TurretTagAim/TurretTargetDeg", turretDeg);
                SmartDashboard.putBoolean("TurretTagAim/HasTarget",     true);
            }
        } else {
            // Tag lost — hold last known angle
            if (m_hasTarget) {
                m_turret.setAngleDeg(m_lastValidDeg);
            }
            SmartDashboard.putBoolean("TurretTagAim/HasTarget", false);
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
