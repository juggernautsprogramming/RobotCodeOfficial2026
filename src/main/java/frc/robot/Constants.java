package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class Constants {

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

    public static final class VisionHardware {
        // Must match the name in the PhotonVision web dashboard exactly
        public static final String CAMERA_OBJECT_NAME = "Camera_BackLeft";

        /**
         * Physical location of the camera relative to the robot center (Meters).
         * X: Forward/Backward (Positive is forward)
         * Y: Left/Right (Positive is left)
         * Z: Up/Down (Positive is up)
         */
        public static final Transform3d kRobotToCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(-10), 0.0, Units.inchesToMeters(15)), 
            new Rotation3d(0, Units.degreesToRadians(-15), Math.PI) // Facing backward (PI) and tilted 15 deg down
        );

    }

    public static final class VisionConstants {
        /** * The 2026 Field Layout. 
         * Note: If using the 2025 library still, use k2025Reefscape. 
         */
        public static final AprilTagFieldLayout kTagLayout = 
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        // Visual feedback/Tolerances
        public static final double angleTolerance = 1.0; // Degrees
        public static final double kAmbiguityThreshold = 0.2; // Ignore targets that are "blurry" or confusing

        public static final double kP = 0.1; // Adjust for better tuning
        public static final double kI = 0.00;
        public static final double kD = 0.005;
        public static final double kS = 0.1;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }
}