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

/**
 * Central constants file for the FRC 2026 robot.
 *
 * Layout:
 *   AutoConstants    — autonomous path-following gains
 *   VisionHardware   — camera names, physical Transform3d offsets
 *   VisionConstants  — tag layout, Kalman/PID tuning, distance thresholds
 *   ShooterConstants — hub geometry, ballistics parameters, alignment PID
 */
public final class Constants {

    private Constants() {}

    // =========================================================================
    // AUTO
    // =========================================================================

    public static final class AutoConstants {
        public static final double kPXController = 1.0;
        public static final double kPYController = 1.0;
        public static final Constraints kThetaControllerConstraints = new Constraints(3.0, 2.0);

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d( Units.inchesToMeters(10),  Units.inchesToMeters(10)),
            new Translation2d( Units.inchesToMeters(10),  Units.inchesToMeters(-10)),
            new Translation2d( Units.inchesToMeters(-10),  Units.inchesToMeters(10)),
            new Translation2d( Units.inchesToMeters(-10), Units.inchesToMeters(-10))
        );

        private AutoConstants() {}
    }

    // =========================================================================
    // VISION — hardware (camera physical positions)
    // =========================================================================

    /**
     * Camera names must match exactly what is configured in PhotonVision.
     * These are the two physical cameras on the robot.
     */
    public static final class VisionHardware {

        /** Name used in PhotonVision UI for the rear-right camera. */
        public static final String CAMERA_BACK_RIGHT = "Camera-BackRight";
        /** Name used in PhotonVision UI for the rear-left camera. */
        public static final String CAMERA_BACK_LEFT  = "Camera-BackLeft";

        /**
         * All camera offsets keyed by camera name.
         * Transform3d is robot-center → camera (forward=+X, left=+Y, up=+Z).
         * Adjust translations/rotations to match your actual mounting.
         */
        public static final Map<String, Transform3d> kCameraOffsets = Map.of(
            CAMERA_BACK_RIGHT, new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-10),   // 10" behind robot center
                    Units.inchesToMeters(-10),   // 10" to the right
                    Units.inchesToMeters(20)),   // 20" above floor
                new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(0))
            ),
            CAMERA_BACK_LEFT, new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-10),   // 10" behind robot center
                    Units.inchesToMeters(10),    // 10" to the left
                    Units.inchesToMeters(20)),   // 20" above floor
                new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180))
            )
        );

        /** Fallback transform used if a camera name is not in kCameraOffsets. */
        public static final Transform3d kDefaultRobotToCam = new Transform3d();

        private VisionHardware() {}
    }

    // =========================================================================
    // VISION — algorithm constants
    // =========================================================================

    public static final class VisionConstants {

        /** 2026 field AprilTag layout — used by PhotonPoseEstimator. */
        public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        /**
         * Maximum pose ambiguity ratio accepted from a single-tag solve.
         * Range [0, 1]; lower = stricter. 0.2 is a good starting point.
         */
        public static final double kAmbiguityThreshold = 0.2;

        // ── Yaw-alignment PID (used for fine rotation-only align) ─────────────
        public static final double kP = 0.06;
        public static final double kI = 0.002;
        public static final double kD = 0.005;
        /** Acceptable yaw error before "aligned" is declared (degrees). */
        public static final double angleTolerance = 0.5;

        // ── Camera geometry (used for 2D distance calculation) ────────────────
        /** Height of the camera lens above the floor (meters). */
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(20);
        /** Height of the generic (non-hub) tag target center above floor (meters). */
        public static final double TARGET_HEIGHT_METERS  = Units.inchesToMeters(12.9);
        /** Camera pitch angle (radians, negative = tilted up). */
        public static final double CAMERA_PITCH_RADIANS  = Units.degreesToRadians(-20);

        private VisionConstants() {}
    }

    // =========================================================================
    // SHOOTER — geometry, ballistics, alignment PID
    // =========================================================================

    /**
     * All shooter-related constants.
     *
     * Physics model (from frc_v0_calculator.html):
     *   v₀ = d · √( g / (2·cos²θ · (d·tanθ − Δh)) )
     *   entry_angle = atan2(−vy_final, vx)
     */
    public static final class ShooterConstants {

        // ── Field / Hub geometry ──────────────────────────────────────────────
        /**
         * Hub center position on the field (meters, field-relative blue-origin).
         * Update once the official 2026 field drawing is released.
         */
        public static final Translation2d HUB_CENTER = new Translation2d(8.270, 4.105);

        /**
         * Height of the hub opening (top of target) above the floor (meters).
         * Measure from the 2026 game manual or physical field.
         */
        public static final double HUB_TARGET_HEIGHT_METERS = 2.64;

        /**
         * AprilTag IDs that are attached to / centered on the hub.
         * Check the 2026 game manual and update before competition.
         */
        public static final int[] HUB_APRIL_TAG_IDS = {4, 19};

        // ── Shooter mechanism geometry ────────────────────────────────────────
        /** Height of the game-piece exit point above the floor (meters). */
        public static final double SHOOTER_EXIT_HEIGHT_METERS = 0.52;

        /**
         * Horizontal distance from robot center to shooter exit (meters).
         * Positive = exit is in front of robot center.
         */
        public static final double SHOOTER_X_OFFSET_METERS = 0.20;

        // ── Flywheel / motor hardware ─────────────────────────────────────────
        /** Shooter wheel diameter (inches). */
        public static final double WHEEL_DIAMETER_INCHES = 4.0;

        /** Gear ratio: motor turns per one wheel turn. 1.0 = direct drive. */
        public static final double SHOOTER_GEAR_RATIO = 1.0;

        /**
         * Compression efficiency: fraction of flywheel surface speed that
         * transfers to the game piece (typical range 0.80–0.95).
         */
        public static final double EFFICIENCY = 0.85;

        /** Motor RPM cap — shots requiring more than this are rejected as invalid. */
        public static final double MAX_MOTOR_RPM = 6000.0;

        /** Idle RPM: flywheel speed held while the command is active but not firing. */
        public static final double IDLE_RPM = 500.0;

        // ── Pivot (hood) hardware — matches ShooterSubsystem ─────────────────
        /** Pivot motor CAN ID (leader). */
        public static final int PIVOT_LEADER_ID = 15;
        /** Pivot motor CAN ID (follower). */
        public static final int PIVOT_FOLLOWER_ID = 16;
        /** CAN bus name for the pivot motors. */
        public static final String PIVOT_CAN_BUS = "ChassisCAN";

        /** Pivot gear ratio: motor turns per one degree of pivot rotation. */
        public static final double PIVOT_GEAR_RATIO = 1.0; // adjust to actual gearing

        /** Motion Magic cruise velocity (rotations/sec). */
        public static final double PIVOT_CRUISE_VELOCITY = 100.0;
        /** Motion Magic acceleration (rotations/sec²). */
        public static final double PIVOT_ACCELERATION = 200.0;

        /** Pivot PID — Slot 0. */
        public static final double PIVOT_kP = 2.0;
        public static final double PIVOT_kI = 0.0;
        public static final double PIVOT_kD = 0.1;

        /** Acceptable pivot angle error to consider "at target" (degrees). */
        public static final double PIVOT_ANGLE_TOLERANCE_DEG = 1.0;

        // ── Shot angle optimizer ──────────────────────────────────────────────
        /** Minimum launch angle evaluated by ShotCalculator (degrees). */
        public static final double MIN_LAUNCH_ANGLE_DEG = 20.0;
        /** Maximum launch angle evaluated by ShotCalculator (degrees). */
        public static final double MAX_LAUNCH_ANGLE_DEG = 75.0;
        /** Sweep resolution (degrees). Smaller = more precise, slightly more CPU. */
        public static final double ANGLE_SWEEP_STEP_DEG = 0.5;

        /**
         * Scoring weights [flightTime, v₀, entryAngle, motorRPM].
         * Must sum to 1.0. Higher entryAngle weight = accuracy-focused.
         */
        public static final double[] SHOT_SCORE_WEIGHTS = {0.15, 0.15, 0.55, 0.15};

        /** Minimum hub entry angle for a shot to be considered valid (degrees). */
        public static final double MIN_ENTRY_ANGLE_DEG = 25.0;

        // ── Alignment setpoints & tolerances ─────────────────────────────────
        /** Desired standoff distance from the hub face to robot center (meters). */
        public static final double DESIRED_DISTANCE_METERS = 2.5;

        /** Acceptable distance error to allow firing (meters). */
        public static final double DISTANCE_TOLERANCE_METERS = 0.05;

        /** Acceptable heading error to allow firing (degrees). */
        public static final double YAW_TOLERANCE_DEG = 1.5;

        // ── Drive PID — HubAlignController ───────────────────────────────────
        public static final double DRIVE_kP            = 3.5;
        public static final double DRIVE_kD            = 0.15;
        public static final double DRIVE_MAX_SPEED_MPS = 3.0;
        public static final double DRIVE_MAX_ACCEL     = 4.5; // m/s²

        public static final double STRAFE_kP            = 3.5;
        public static final double STRAFE_kD            = 0.15;
        public static final double STRAFE_MAX_SPEED_MPS = 2.5;

        public static final double ROTATION_kP        = 5.5;
        public static final double ROTATION_kD        = 0.25;
        public static final double ROTATION_MAX_RAD_S = Math.PI * 2.0;

        /**
         * Low-pass filter alpha for drive output smoothing.
         * Range (0, 1): smaller = more smoothing (more lag), larger = less smoothing.
         */
        public static final double OUTPUT_FILTER_ALPHA = 0.20;

        /** Minimum output speed (m/s or rad/s) — below this, output is zeroed. */
        public static final double VELOCITY_DEADBAND = 0.01;

        /** Gravitational acceleration (m/s²). */
        public static final double g = 9.81;

        private ShooterConstants() {}
    }
}