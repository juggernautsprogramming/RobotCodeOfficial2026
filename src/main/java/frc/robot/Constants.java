package frc.robot;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class Constants {
    /*  */
    public static final ShuffleboardTab ExampleTab = Shuffleboard.getTab("Main Tab");
    public static final ShuffleboardTab VisionTab = Shuffleboard.getTab("Vision Tab");
    public static final class AutoConstants {
        public static final double kPXController = 1.0;
        public static final double kPYController = 1.0;
        public static final Constraints kThetaControllerConstraints = new Constraints(3.0, 2.0);
        
        /**
         * Swerve Kinematics (Meters)
         * Distance from center to wheels. 10 inches = 0.254 meters.
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(10)),
            new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(-10)),
            new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(10)),
            new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(-10))
        );
    }

    public static class VisionHardware {
        // Define the unique offsets for each camera
        public static final Map<String, Transform3d> kCameraOffsets = Map.of(
            "Camera-BackRight", new Transform3d(new Translation3d(0.3, 0.0, 0.5), new Rotation3d(0, 0, 0)),
           "Camera-BackLeft", new Transform3d(new Translation3d(-0.3, 0.2, 0.5), new Rotation3d(0, 0, Math.toRadians(180)))
        );

        // Keep this as a fallback just in case
        public static final Transform3d kDefaultRobotToCam = new Transform3d(); 
    }

    public static final class VisionConstants {
        /** * The 2026 Field Layout. 
         * Note: If using the 2025 library still, use k2025Reefscape. 
         */
        public static final AprilTagFieldLayout kTagLayout = 
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        public static final String kFrontLeftName = "FrontLeft";
        public static final String kBackLeftName = "BackLeft";

        public static final Map<String, Transform3d> kCameraOffsets = Map.of(
        // Front Left: 10" forward, 10" left. Facing "Out" (45 deg CCW)
        kFrontLeftName, new Transform3d(
            new Translation3d(Units.inchesToMeters(10), Units.inchesToMeters(10), Units.inchesToMeters(20)), 
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(45))
        ),
        // Back Left: 10" back (-10), 10" left. Facing "In" (135 deg CCW)
        kBackLeftName, new Transform3d(
            new Translation3d(Units.inchesToMeters(-10), Units.inchesToMeters(10), Units.inchesToMeters(20)), 
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(135))
        )
        );
        // Visual feedback/Tolerances
       // Logic Thresholds
        public static final double kAmbiguityThreshold = 0.2; 
        public static final double angleTolerance = 0.5;

        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(20);
        public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(12.9); // Adjust based on 2026 tag height
        public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-20);

        // PID for "Drive to Tag" or "Snap to Heading"
        public static final double kP = 0.06; 
        public static final double kI = 0.002;
        public static final double kD = 0.005;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }
}