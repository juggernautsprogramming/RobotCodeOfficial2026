package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.VelocityCompensator;
import frc.robot.subsystems.Vision.VisionSubsystem;

public class DriveToHubAndShootCommand extends Command {

    // ── State machine ─────────────────────────────────────────────────────────
    private enum State {
        DRIVING,    // Navigate to optimal standoff pose
        ALIGNING,   // Fine-tune heading, wait for all fire gates
        FIRING,     // Execute shot
        DONE
    }

    // ── Constants ─────────────────────────────────────────────────────────────
    private static final double MAX_RUNTIME_S    = 8.0;
    private static final double HOLD_TIME_S      = 0.10;
    private static final double DRIFT_THRESHOLD_M = 0.5;

    private static final double ZONE_MIN_M = ShotCalculator.OPTIMAL_STANDOFF_FIXED_M - 0.25;
    private static final double ZONE_MAX_M = ShotCalculator.OPTIMAL_STANDOFF_FIXED_M + 0.25;

    private static final Translation2d HUB = ShooterConstants.HUB_CENTER;

    // ── Translation PID controllers ───────────────────────────────────────────
    // These drive the robot to the optimal field position
    private final ProfiledPIDController m_xController = new ProfiledPIDController(
        ShooterConstants.DRIVE_kP, 0, ShooterConstants.DRIVE_kD,
        new TrapezoidProfile.Constraints(
            ShooterConstants.DRIVE_MAX_SPEED_MPS,
            ShooterConstants.DRIVE_MAX_ACCEL));

    private final ProfiledPIDController m_yController = new ProfiledPIDController(
        ShooterConstants.STRAFE_kP, 0, ShooterConstants.STRAFE_kD,
        new TrapezoidProfile.Constraints(
            ShooterConstants.STRAFE_MAX_SPEED_MPS,
            ShooterConstants.DRIVE_MAX_ACCEL));

    // ── Dependencies ──────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain m_drivetrain;
    private final ShooterSubsystem        m_shooter;
    private final VisionSubsystem         m_vision;
    private final HubAlignController      m_alignCtrl = new HubAlignController();

    // ── Per-run state ─────────────────────────────────────────────────────────
    private State    m_state;
    private double   m_startTimeS;
    private double   m_allGreenStartS;
    private double   m_targetRPM;
    private double   m_targetHoodDeg;
    /** The field-relative pose the robot should drive to before shooting. */
    private Pose2d   m_targetPose;

    // ── Constructor ───────────────────────────────────────────────────────────
    public DriveToHubAndShootCommand(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            VisionSubsystem vision) {
        m_drivetrain = drivetrain;
        m_shooter    = shooter;
        m_vision     = vision;
        addRequirements(drivetrain, shooter);
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_state          = State.DRIVING;
        m_startTimeS     = Timer.getFPGATimestamp();
        m_allGreenStartS = Double.NaN;

        // ── Compute target pose ───────────────────────────────────────────────
        // Stand at OPTIMAL_STANDOFF_M from hub center, on the same bearing the
        // robot is currently on so we don't waste time going around the hub.
        Pose2d currentPose = m_drivetrain.getState().Pose;
        double bearingRad  = Math.atan2(
            currentPose.getY() - HUB.getY(),
            currentPose.getX() - HUB.getX());

        double targetX = HUB.getX() + ShotCalculator.OPTIMAL_STANDOFF_FIXED_M * Math.cos(bearingRad);
        double targetY = HUB.getY() + ShotCalculator.OPTIMAL_STANDOFF_FIXED_M * Math.sin(bearingRad);

        // Robot must face the hub (opposite of bearing)
        Rotation2d targetHeading = new Rotation2d(bearingRad + Math.PI);
        m_targetPose = new Pose2d(targetX, targetY, targetHeading);

        // Reset translation PIDs to current position
        m_xController.reset(currentPose.getX());
        m_yController.reset(currentPose.getY());
        m_xController.setGoal(targetX);
        m_yController.setGoal(targetY);

        m_alignCtrl.reset();

        // Pause vision Kalman injection so pose resets don't jerk the PID mid-drive
        m_vision.setVisionEnabled(false);

        // 1323: pre-spin flywheel immediately so robot arrives already at speed
        m_shooter.setFlywheelRPM(ShooterConstants.IDLE_RPM);

        publishInit();
        SmartDashboard.putNumber("DTHS2/TargetPoseX", targetX);
        SmartDashboard.putNumber("DTHS2/TargetPoseY", targetY);
        SmartDashboard.putNumber("DTHS2/TargetHeading_deg", targetHeading.getDegrees());
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        var    speeds    = m_drivetrain.getState().Speeds;

        // ── FIX 3: Correct distance uses shooter exit, not robot center ───────
        // Shift the effective robot position forward by SHOOTER_X_OFFSET_METERS
        // along the robot's current heading before measuring distance to hub.
        double headingRad    = robotPose.getRotation().getRadians();
        double shooterX      = robotPose.getX() + ShooterConstants.SHOOTER_X_OFFSET_METERS * Math.cos(headingRad);
        double shooterY      = robotPose.getY() + ShooterConstants.SHOOTER_X_OFFSET_METERS * Math.sin(headingRad);
        Translation2d alignTarget = getAlignTarget();
        double distToHub     = new Translation2d(shooterX, shooterY).getDistance(alignTarget);

        // Velocity-compensated virtual distance
        var comp = VelocityCompensator.getInstance().compensate(
            new Translation2d(shooterX, shooterY), alignTarget,
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        ShotCalculator.ShotResult shot = ShotCalculator.calculateFixed(comp.virtualDistanceMeters());
        m_targetRPM     = shot.rpm();
        m_targetHoodDeg = shot.hoodDeg();

        // Shooter runs every tick regardless of state (1323 pre-spin)
        m_shooter.setFlywheelRPM(m_targetRPM);
        m_shooter.setLaunchAngleDeg(m_targetHoodDeg);

        // Rotation output from HubAlignController (ProfiledPID toward tag/hub)
        HubAlignController.DriveOutput alignOut =
            m_alignCtrl.compute(m_drivetrain, alignTarget, ShotCalculator.OPTIMAL_STANDOFF_FIXED_M);

        boolean inZone = distToHub >= ZONE_MIN_M && distToHub <= ZONE_MAX_M;

        switch (m_state) {

            case DRIVING -> {
                // ── FIX 1: Full field-relative navigation to target pose ───────
                // X/Y ProfiledPID drives to the computed standoff position while
                // HubAlignController simultaneously locks rotation onto the hub.
                double vx = m_xController.calculate(robotPose.getX());
                double vy = m_yController.calculate(robotPose.getY());

                // Clamp outputs to max speeds
                vx = Math.max(-ShooterConstants.DRIVE_MAX_SPEED_MPS,
                     Math.min( ShooterConstants.DRIVE_MAX_SPEED_MPS, vx));
                vy = Math.max(-ShooterConstants.STRAFE_MAX_SPEED_MPS,
                     Math.min( ShooterConstants.STRAFE_MAX_SPEED_MPS, vy));

                m_drivetrain.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(alignOut.omegaRadS()));

                // Log how close we are to the target pose
                double poseError = robotPose.getTranslation()
                    .getDistance(m_targetPose.getTranslation());
                SmartDashboard.putNumber("DTHS2/PoseError_m", poseError);

                if (inZone && poseError < 0.15) m_state = State.ALIGNING;
            }

            case ALIGNING -> {
                // ── FIX 2: Only zero translation when truly in zone ───────────
                // If we've drifted, gently correct with X/Y PIDs rather than
                // cutting translation entirely.
                double vx = inZone ? 0.0 : m_xController.calculate(robotPose.getX());
                double vy = inZone ? 0.0 : m_yController.calculate(robotPose.getY());

                m_drivetrain.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(alignOut.omegaRadS()));

                boolean aligned    = m_alignCtrl.isAligned();
                boolean atDist     = inZone;
                boolean atVelocity = m_shooter.isAtTargetRPM(m_targetRPM);
                boolean hubActive  = isHubConfirmedActive();
                boolean allGreen   = aligned && atDist && atVelocity && hubActive;

                if (allGreen) {
                    if (Double.isNaN(m_allGreenStartS))
                        m_allGreenStartS = Timer.getFPGATimestamp();
                    if ((Timer.getFPGATimestamp() - m_allGreenStartS) >= HOLD_TIME_S)
                        m_state = State.FIRING;
                } else {
                    m_allGreenStartS = Double.NaN;
                }

                // Far outside zone → restart full approach
                if (distToHub < ZONE_MIN_M - DRIFT_THRESHOLD_M
                        || distToHub > ZONE_MAX_M + DRIFT_THRESHOLD_M) {
                    // Recompute target pose from current position
                    double bearingRad = Math.atan2(
                        robotPose.getY() - alignTarget.getY(),
                        robotPose.getX() - alignTarget.getX());
                    m_xController.setGoal(alignTarget.getX() + ShotCalculator.OPTIMAL_STANDOFF_FIXED_M * Math.cos(bearingRad));
                    m_yController.setGoal(alignTarget.getY() + ShotCalculator.OPTIMAL_STANDOFF_FIXED_M * Math.sin(bearingRad));
                    m_state = State.DRIVING;
                    m_allGreenStartS = Double.NaN;
                }

                publishFireGate(aligned, atDist, atVelocity, hubActive);
            }

            case FIRING -> {
                // Hold heading for one tick while shot executes
                m_drivetrain.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(alignOut.omegaRadS()));
                m_shooter.shoot();
                m_state = State.DONE;
            }

            case DONE -> {}
        }

        // ── Telemetry ──────────────────────────────────────────────────────────
        double elapsed = Timer.getFPGATimestamp() - m_startTimeS;
        SmartDashboard.putString ("DTHS2/State",       m_state.name());
        SmartDashboard.putNumber ("DTHS2/DistToHub_m", distToHub);
        SmartDashboard.putNumber ("DTHS2/DistError_m", distToHub - ShotCalculator.OPTIMAL_STANDOFF_FIXED_M);
        SmartDashboard.putNumber ("DTHS2/ZoneMin_m",   ZONE_MIN_M);
        SmartDashboard.putNumber ("DTHS2/ZoneMax_m",   ZONE_MAX_M);
        SmartDashboard.putNumber ("DTHS2/TargetRPM",   m_targetRPM);
        SmartDashboard.putNumber ("DTHS2/HoodDeg",     m_targetHoodDeg);
        SmartDashboard.putNumber ("DTHS2/VirtDist_m",  comp.virtualDistanceMeters());
        SmartDashboard.putNumber ("DTHS2/ElapsedS",    elapsed);
        SmartDashboard.putBoolean("DTHS2/TimedOut",    elapsed >= MAX_RUNTIME_S);
    }

    @Override
    public boolean isFinished() {
        return m_state == State.DONE
            || (Timer.getFPGATimestamp() - m_startTimeS) >= MAX_RUNTIME_S;
    }

    @Override
    public void end(boolean interrupted) {
        m_vision.setVisionEnabled(true);
        m_drivetrain.setControl(new SwerveRequest.Idle());
        m_shooter.idleFlywheel();
        SmartDashboard.putString("DTHS2/State", interrupted ? "INTERRUPTED" : "COMPLETE");
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private Translation2d getAlignTarget() {
        return HUB;
    }

    private boolean isHubConfirmedActive() {
        if (!DriverStation.isFMSAttached()) return true;
        String msg = DriverStation.getGameSpecificMessage();
        return msg != null && !msg.isEmpty();
    }

    private void publishInit() {
        SmartDashboard.putString ("DTHS2/State",           State.DRIVING.name());
        SmartDashboard.putBoolean("DTHS2/Gate/Aligned",    false);
        SmartDashboard.putBoolean("DTHS2/Gate/AtDist",     false);
        SmartDashboard.putBoolean("DTHS2/Gate/AtVelocity", false);
        SmartDashboard.putBoolean("DTHS2/Gate/HubActive",  false);
        SmartDashboard.putBoolean("DTHS2/Gate/FIRE",       false);
        SmartDashboard.putNumber ("DTHS2/Gate/HoldTime_s", 0.0);
        SmartDashboard.putBoolean("DTHS2/TimedOut",        false);
        SmartDashboard.putNumber ("DTHS2/ZoneMin_m",       ZONE_MIN_M);
        SmartDashboard.putNumber ("DTHS2/ZoneMax_m",       ZONE_MAX_M);
        SmartDashboard.putNumber ("DTHS2/PoseError_m",     0.0);
    }

    private void publishFireGate(
            boolean aligned, boolean atDist, boolean atVelocity, boolean hubActive) {
        SmartDashboard.putBoolean("DTHS2/Gate/Aligned",    aligned);
        SmartDashboard.putBoolean("DTHS2/Gate/AtDist",     atDist);
        SmartDashboard.putBoolean("DTHS2/Gate/AtVelocity", atVelocity);
        SmartDashboard.putBoolean("DTHS2/Gate/HubActive",  hubActive);
        SmartDashboard.putBoolean("DTHS2/Gate/FIRE",
            aligned && atDist && atVelocity && hubActive);
        SmartDashboard.putNumber ("DTHS2/Gate/HoldTime_s",
            Double.isNaN(m_allGreenStartS) ? 0.0
                : Timer.getFPGATimestamp() - m_allGreenStartS);
    }
}