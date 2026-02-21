package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {
    // Hardware & Subsystems
    private final CommandSwerveDrivetrain m_drivetrain;
    private final List<ReadAprilTag> m_sensors = new ArrayList<>();
    private final Field2d m_field = new Field2d();
    
    // Logic State for UI and Commands
    private PhotonTrackedTarget m_bestTarget;
    private ReadAprilTag m_activeSensor;
    private boolean m_allianceSet = false;
    
    // Controllers for simple alignment
    private final PIDController strafePID;

    // Shuffleboard Entries
    private final GenericEntry m_tagIdWidget;
    private final GenericEntry m_yawWidget;
    private final GenericEntry m_robotXWidget;
    private final GenericEntry m_robotYWidget;
    private final GenericEntry m_distanceWidget;
    private final GenericEntry m_hasTargetWidget;

    public VisionSubsystem(String[] cameraNames, CommandSwerveDrivetrain drivetrain) {
        this.m_drivetrain = drivetrain;

        // 1. Initialize PID for alignment
        strafePID = new PIDController(VisionConstants.kP, VisionConstants.kI, VisionConstants.kD);
        strafePID.setTolerance(VisionConstants.angleTolerance);

        // 2. Setup Shuffleboard Layout
        var cameraGroup = Constants.ExampleTab
            .getLayout("Robot Cameras", BuiltInLayouts.kGrid)
            .withPosition(2, 0)
            .withSize(6, 3)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 1));

        // 3. Initialize Sensors (ReadAprilTag objects)
        for (String name : cameraNames) {
            Transform3d robotToCam = Constants.VisionHardware.kCameraOffsets.getOrDefault(
                name, Constants.VisionHardware.kDefaultRobotToCam
            );

            ReadAprilTag sensor = new ReadAprilTag(name, robotToCam);
            m_sensors.add(sensor);

            // Add camera stream to Shuffleboard
            String hostname = name.toLowerCase(); 
            String streamUrl = "mjpg:http://" + hostname + ".local:1181/?action=stream";
            cameraGroup.addCamera("Stream-" + name, name, streamUrl)
                       .withProperties(Map.of("showControls", false));
        }

        // 4. Initialize UI Widgets
        m_tagIdWidget = Constants.ExampleTab.add("Target ID", -1).withPosition(0, 0).getEntry();
        m_hasTargetWidget = Constants.ExampleTab.add("Visible", false)
            .withWidget(BuiltInWidgets.kBooleanBox).withPosition(1, 0).getEntry();
        
        m_yawWidget = Constants.ExampleTab.add("Target Yaw", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar).withPosition(0, 1).withSize(2, 1).getEntry();
        
        m_robotXWidget = Constants.ExampleTab.add("Robot X", 0.0).withPosition(0, 3).getEntry();
        m_robotYWidget = Constants.ExampleTab.add("Robot Y", 0.0).withPosition(1, 3).getEntry();
        m_distanceWidget = Constants.ExampleTab.add("Distance (M)", 0.0).withPosition(0, 4).getEntry();

        Constants.ExampleTab.add("Vision Field Map", m_field).withPosition(5, 0).withSize(4, 3);
    }

    @Override
    public void periodic() {
        m_bestTarget = null;
        m_activeSensor = null;

        // 1. Handle Alliance Origin (Run once when alliance is available)
        if (!m_allianceSet && DriverStation.getAlliance().isPresent()) {
            updateAllianceOrigin();
            m_allianceSet = true;
        }

        // 2. Update Field2d UI with Robot and Camera Poses
        Pose2d robotPose = m_drivetrain.getState().Pose;
        m_field.setRobotPose(robotPose);

        // Visualize specific camera layouts (front-left outward, back-left inward)
        for (ReadAprilTag sensor : m_sensors) {
            String camName = sensor.getCamera().getName();
            Transform3d camOffset = Constants.VisionHardware.kCameraOffsets.getOrDefault(
                camName, Constants.VisionHardware.kDefaultRobotToCam);
            
            // Project the camera's position/rotation onto the 2D field map
            Pose2d camPoseOnField = robotPose.transformBy(
                new Transform2d(camOffset.getTranslation().toTranslation2d(), camOffset.getRotation().toRotation2d())
            );
            m_field.getObject(camName + " Layout").setPose(camPoseOnField);
        }

        // 3. Detect Tilt (Bump/Obstacles)
        double pitch = m_drivetrain.getPigeon2().getPitch().getValueAsDouble();
        double roll = m_drivetrain.getPigeon2().getRoll().getValueAsDouble();
        boolean isOnBump = Math.abs(pitch) > 5.0 || Math.abs(roll) > 5.0;

        // 4. Process Camera Data
        for (ReadAprilTag sensor : m_sensors) {
            var visionUpdate = sensor.getEstimatedGlobalPose(robotPose);

            if (visionUpdate.isPresent()) {
                EstimatedRobotPose estimate = visionUpdate.get();
                Pose2d estPose2d = estimate.estimatedPose.toPose2d();

                // Draw "Ghost Robot" for raw vision data
                m_field.getObject("VisionTarget-" + sensor.getCamera().getName()).setPose(estPose2d);

                // Trust Logic: High trust in X/Y (0.1), ignore Vision Rotation (99999)
                // If on bump, we trust vision even more (0.05) to override wheel slip
                double xyTrust = isOnBump ? 0.05 : 0.5;
                var stdDevs = VecBuilder.fill(xyTrust, xyTrust, Units.degreesToRadians(99999));

                m_drivetrain.addVisionMeasurement(
                    estPose2d,
                    estimate.timestampSeconds,
                    stdDevs
                );

                // Update best target for UI logic
                var result = sensor.getCamera().getLatestResult();
                if (result.hasTargets()) {
                    if (m_bestTarget == null || result.getBestTarget().getArea() > m_bestTarget.getArea()) {
                        m_bestTarget = result.getBestTarget();
                        m_activeSensor = sensor;
                    }
                }
            } else {
                // Clear ghost if camera sees nothing
                m_field.getObject("VisionTarget-" + sensor.getCamera().getName()).setPose(new Pose2d(-10, -10, new Rotation2d()));
            }
        }   
        updateUI();
    }

    private void updateUI() {
        Pose2d currentPose = m_drivetrain.getState().Pose;
        m_robotXWidget.setDouble(currentPose.getX());
        m_robotYWidget.setDouble(currentPose.getY());

        if (m_bestTarget != null) {
            m_hasTargetWidget.setBoolean(true);
            m_tagIdWidget.setInteger(m_bestTarget.getFiducialId());
            m_yawWidget.setDouble(m_bestTarget.getYaw());
            m_distanceWidget.setDouble(getDistanceToTarget());
        } else {
            m_hasTargetWidget.setBoolean(false);
            m_tagIdWidget.setInteger(-1);
        }
    }

    public double getDistanceToTarget() {
        if (m_bestTarget == null) return 0.0;
        return Units.metersToInches(PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.CAMERA_HEIGHT_METERS,
            VisionConstants.TARGET_HEIGHT_METERS,
            VisionConstants.CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(m_bestTarget.getPitch())
        ));
    }

    public double getAlignmentRotationSpeed() {
        if (m_bestTarget == null) return 0.0;
        double rotationOutput = strafePID.calculate(m_bestTarget.getYaw(), 0);
        return Math.max(-3.0, Math.min(3.0, rotationOutput));
    }

    private void updateAllianceOrigin() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            VisionConstants.kTagLayout.setOrigin(
                alliance.get() == DriverStation.Alliance.Red 
                    ? edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide 
                    : edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide
            );
        }
    }

    public boolean hasValidTarget() { return m_bestTarget != null; }
    public PhotonTrackedTarget getBestTarget() { return m_bestTarget; }
}