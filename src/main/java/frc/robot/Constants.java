package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

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

        private AutoConstants() {
        }
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
                    Units.inchesToMeters(10),   // 10" behind robot center
                    Units.inchesToMeters(10),   // 10" to the right
                    Units.inchesToMeters(20)),   // 20" above floor
                new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(90))
            ),
            CAMERA_BACK_LEFT, new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-10),   // 10" behind robot center
                    Units.inchesToMeters(10),    // 10" to the left
                    Units.inchesToMeters(20)),   // 20" above floor
                new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(285))
            )
        );

        /** Fallback transform used if a camera name is not in kCameraOffsets. */
        public static final Transform3d kDefaultRobotToCam = new Transform3d();

        private VisionHardware() {}
    }
// =========================================================================
// AUTONOMOUS STARTING POSITIONS
// =========================================================================
public static final class AutoStartConstants {

    /** Left start — robot centre on the hub line, facing hub */
    public static final double LEFT_START_X   = 2.0;
    public static final double LEFT_START_Y   = 6.5;
    public static final double LEFT_START_HDG = -30.0; // degrees

    /** Center start — directly in front of hub */
    public static final double CENTER_START_X   = 2.0;
    public static final double CENTER_START_Y   = 4.026; // matches HUB_CENTER.y
    public static final double CENTER_START_HDG = 0.0;

    /** Right start — right side of hub line */
    public static final double RIGHT_START_X   = 2.0;
    public static final double RIGHT_START_Y   = 1.7;
    public static final double RIGHT_START_HDG = 30.0;

    /** Balls to shoot from preload (first shot zone) */
    public static final int PRELOAD_BALL_COUNT = 3;
    /** Balls picked up from the field */
    public static final int PICKUP_BALL_COUNT  = 5;
    /** Hard safety timeout for the entire auto sequence */
    public static final double AUTO_TIMEOUT_S  = 15.0;
    // PathPlanner auto constraints
    public static final double AUTO_MAX_VEL       = 2.0;   // m/s
    public static final double AUTO_MAX_ACCEL     = 2.5;   // m/s²
    public static final double AUTO_MAX_ANG_VEL   = 360.0; // deg/s
    public static final double AUTO_MAX_ANG_ACCEL = 720.0; // deg/s²
    private AutoStartConstants() {}
}
    // =========================================================================
    // VISION — algorithm constants
    // =========================================================================

    public static final class VisionConstants {

        /** 2026 field AprilTag layout — used by PhotonPoseEstimator. */
        public static AprilTagFieldLayout kTagLayout;
                static {
                    try {
                        // This looks for the file you just put in src/main/deploy
                        Path path = Filesystem.getDeployDirectory().toPath().resolve("custom_hub.json");
                        kTagLayout = new AprilTagFieldLayout(path);
                    } catch (IOException e) {
                        DriverStation.reportError("Could not load custom AprilTag layout!", e.getStackTrace());
                        // Fallback to empty layout if file is missing
                        kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
            }
        }
        // ── 2026 Rebuilt (AndyMark) custom hub field dimensions ───────────────
        /** Full field length — from blue wall to red wall (metres). */
        public static final double FIELD_LENGTH_M = 8.0;
        /** Full field width (metres). */
        public static final double FIELD_WIDTH_M  = 8.052;

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
        public static final double CAMERA_PITCH_RADIANS  = Units.degreesToRadians(20);
        // In VisionConstants:
        /** Height of the hub AprilTag center above the floor (meters). */
        public static final double HUB_TAG_HEIGHT_METERS = Units.inchesToMeters(24.0); // measure and update
        private VisionConstants() {}
    }

    // =========================================================================
    // DRIVE TO POSE — ProfiledPID gains & tolerances for DriveToPose command
    // =========================================================================

    public static final class DriveToPoseConstants {

        // ── Translation ProfiledPID (X and Y share the same gains) ───────────
        public static final double kP_XY   = 3.5;
        public static final double kI_XY   = 0.0;
        public static final double kD_XY   = 0.15;
        /** Maximum approach speed (m/s). */
        public static final double MAX_VEL_MPS    = 3.0;
        /** Maximum approach acceleration (m/s²). */
        public static final double MAX_ACCEL_MPS2 = 4.0;

        // ── Heading ProfiledPID ───────────────────────────────────────────────
        public static final double kP_THETA  = 5.5;
        public static final double kI_THETA  = 0.0;
        public static final double kD_THETA  = 0.25;
        /** Maximum rotation speed (rad/s). */
        public static final double MAX_OMEGA_RAD_S   = Math.PI * 2.0;
        /** Maximum angular acceleration (rad/s²). */
        public static final double MAX_ALPHA_RAD_S2  = Math.PI * 4.0;

        // ── At-goal tolerances ────────────────────────────────────────────────
        /** XY tolerance before "at goal" (metres). ~1.5 inches. */
        public static final double XY_TOLERANCE_M      = Units.inchesToMeters(1.5);
        /** Heading tolerance before "at goal" (radians). 1 degree. */
        public static final double THETA_TOLERANCE_RAD = Units.degreesToRadians(1.0);

        // ── Debounce & safety ─────────────────────────────────────────────────
        /** Robot must stay within tolerance for this long before isFinished (seconds). */
        public static final double DEBOUNCE_S       = 0.10;
        /** Kill-switch: abort if the command runs longer than this (seconds). */
        public static final double MAX_RUNTIME_S    = 5.0;
        /** Joystick axis magnitude that triggers driver override. */
        public static final double OVERRIDE_DEADBAND = 0.15;

        private DriveToPoseConstants() {}
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
         * Hub center position on the custom 8 m × 8.052 m field (metres).
         * Derived from the four hub AprilTag positions in custom_hub.json:
         *   Tag 19: (4.0,    4.026), Tag 9: (5.1938, 4.026)  ← X midpoint = 4.5969
         *   Tag 8:  (4.5969, 4.6229), Tag 11: (4.5969, 3.4291) ← Y midpoint = 4.026
         */
        // ── Flywheel ──────────────────────────────────────────────────────────────────
        public static final int    FLYWHEEL_LEADER_ID   = 29;   // ← your CAN ID
        public static final int    FLYWHEEL_FOLLOWER_ID = 30;   // ← your CAN ID
        public static final String FLYWHEEL_CAN_BUS     = "rio"; // or "CANivore", etc.
        public static final boolean FLYWHEEL_FOLLOWER_OPPOSE = true; // oppose if motors face each other
        public static final double FIXED_SHOT_RPM_M    = 200.0;
        public static final double FIXED_SHOT_ANGLE_DEG = 45.0; // tune this on the robot

        // ── Distance-based fixed RPM presets (operator D-pad) ─────────────────
        // D-Pad Up    — close range  (1.5 m) ✓ confirmed
        public static final double PRESET_CLOSE_RPM      = 2440.0;
        public static final double PRESET_CLOSE_DIST_M   = 1.5;
        // D-Pad Right — mid range    (2.5 m) Confirmed
        public static final double PRESET_MID_RPM        = 2610.0;
        public static final double PRESET_MID_DIST_M     = 2.5;
        // D-Pad Down  — far range    (4.0 m) Confirmed
        public static final double PRESET_FAR_RPM        = 3000;
        public static final double PRESET_FAR_DIST_M     = 4.0;
        // D-Pad Left  — very far     (5.5 m) Confirmed
        public static final double PRESET_VFAR_RPM       = 3600;
        public static final double PRESET_VFAR_DIST_M    = 5.5;
    // Flywheel PID / feedforward (tune these on the robot)
        public static final double FLYWHEEL_kP = 0.3;   // increased: actively corrects RPM error
        public static final double FLYWHEEL_kI = 0.003; // small integral: eliminates steady-state offset
        public static final double FLYWHEEL_kD = 0.0;
        public static final double FLYWHEEL_kS = 0.0;
        public static final double FLYWHEEL_kV = 0.13; // V per rot/s — above 0.12 saturates output at high RPM so motor runs at full duty cycle
        public static final Translation2d HUB_CENTER = new Translation2d(4.5969, 4.026);

        /**
         * Height of the hub opening (top of target) above the floor (meters).
         * Measure from the 2026 game manual or physical field.
         */
        public static final double HUB_TARGET_HEIGHT_METERS = 2.64;

        /**
         * AprilTag IDs that are attached to / centered on the hub.
         * Check the 2026 game manual and update before competition.
         */
        public static final int[] HUB_APRIL_TAG_IDS = {8,9,11, 19};

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
        public static final int PIVOT_LEADER_ID = 99;
        /** Pivot motor CAN ID (follower). */
        public static final int PIVOT_FOLLOWER_ID = 99;
        /** CAN bus name for the pivot motors. */
        public static final String PIVOT_CAN_BUS = "rio";

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
        /**
         * Desired standoff distance from the hub face to robot center (meters).
         * Used as a fallback only; the physics-based optimal distance computed by
         * {@code ShotCalculator.OPTIMAL_STANDOFF_M} replaces this at runtime.
         */
        public static final double DESIRED_DISTANCE_METERS = 2.5;

        /** Acceptable distance error to allow firing (meters). */
        public static final double DISTANCE_TOLERANCE_METERS = 0.05;

        // ── Optimal-distance sweep (used by ShotCalculator.findOptimalDistance) ──
        /** Closest distance (m) the sweep will consider. */
        public static final double OPTIMAL_DIST_MIN_M  = 1.5;
        /** Farthest distance (m) the sweep will consider. */
        public static final double OPTIMAL_DIST_MAX_M  = 6.0;
        /** Resolution of the distance sweep (m). Smaller = more precise, slightly more startup CPU. */
        public static final double OPTIMAL_DIST_STEP_M = 0.10;

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

        public static final double ROTATION_kP             = 5.5;
        public static final double ROTATION_kD             = 0.25;
        public static final double ROTATION_MAX_RAD_S      = Math.PI * 2.0;
        /** Max angular acceleration for the ProfiledPIDController (rad/s²). */
        public static final double ROTATION_MAX_ACCEL_RAD_S2 = Math.PI * 4.0;

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