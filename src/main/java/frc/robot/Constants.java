package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
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
 *   AutoConstants        — autonomous path-following gains + PathPlanner limits
 *   AutoStartConstants   — 3 starting positions, ball counts, timing
 *   VisionHardware       — camera names, physical Transform3d offsets
 *   VisionConstants      — tag layout, Kalman/PID tuning, distance thresholds
 *   DriveToPoseConstants — ProfiledPID gains for DriveToPose command
 *   ShooterConstants     — hub geometry, ballistics parameters, alignment PID
 */
public final class Constants {

    private Constants() {}

    // =========================================================================
    // AUTO — path-following gains
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

        // ── PathPlanner motion limits ─────────────────────────────────────────
        /** Max linear velocity for PathPlanner paths (m/s). */
        public static final double AUTO_MAX_VEL       = 2.0;
        /** Max linear acceleration for PathPlanner paths (m/s²). */
        public static final double AUTO_MAX_ACCEL     = 2.5;
        /** Max angular velocity for PathPlanner paths (degrees/s). */
        public static final double AUTO_MAX_ANG_VEL   = 360.0;
        /** Max angular acceleration for PathPlanner paths (degrees/s²). */
        public static final double AUTO_MAX_ANG_ACCEL = 720.0;

        private AutoConstants() {}
    }

    // =========================================================================
    // AUTONOMOUS STARTING POSITIONS
    // =========================================================================

    /**
     * Starting positions for the 8-ball autonomous sequence.
     *
     * <p><b>Coordinate system (Blue alliance):</b>
     * <pre>
     *   x = 0.000  → Blue wall
     *   x = 8.000  → Red wall
     *   y = 0.000  → Bottom wall
     *   y = 8.052  → Top wall
     *   Hub center → (4.5969, 4.026)   from custom_hub.json AprilTags
     *
     *   Starting line  x ≈ 0.900 m    (robot bumper ~0.9 m from blue wall)
     *   Shoot zone     x ≈ 2.500 m    (2.097 m from hub — confirmed in PathPlanner)
     *   Ball pickup    x ≈ 5.600 m    (≈1.0 m past hub into centre ball cluster)
     * </pre>
     *
     * <p><b>Alliance mirroring:</b> All paths are drawn for Blue alliance.
     * PathPlanner automatically mirrors them for Red alliance via the flip flag
     * in {@code configurePathPlanner()} — no separate Red paths are needed.
     *
     * <p><b>Fine-tune after first field test:</b>
     * <ul>
     *   <li>{@code *_START_X} — adjust if robot bumper is not 0.9 m from the wall.</li>
     *   <li>{@code LEFT/RIGHT_START_Y} — adjust if starting boxes are not 2 m apart.</li>
     *   <li>{@code *_START_HDG} — adjust if robot overshoots hub heading at shoot zone.</li>
     *   <li>Ball pickup X in paths — move right/left until intake reliably captures balls.</li>
     * </ul>
     */
    public static final class AutoStartConstants {

        // ── LEFT position (upper — high Y) ────────────────────────────────────
        /** Distance from blue wall to robot center at start (metres). */
        public static final double LEFT_START_X   = 0.900;
        /** Y centre of the top starting box — 2 m above hub centre. */
        public static final double LEFT_START_Y   = 6.026;
        /** Heading angled toward hub from top position (degrees). */
        public static final double LEFT_START_HDG = -28.4;

        // ── CENTER position ───────────────────────────────────────────────────
        public static final double CENTER_START_X   = 0.900;
        /** Y == hub centre Y — robot starts directly in front of hub. */
        public static final double CENTER_START_Y   = 4.026;
        /** Straight toward hub. */
        public static final double CENTER_START_HDG = 0.0;

        // ── RIGHT position (lower — low Y) ────────────────────────────────────
        public static final double RIGHT_START_X   = 0.900;
        /** Y centre of the bottom starting box — 2 m below hub centre. */
        public static final double RIGHT_START_Y   = 2.026;
        /** Heading angled toward hub from bottom position (degrees). */
        public static final double RIGHT_START_HDG = 28.4;

        // ── Ball counts ───────────────────────────────────────────────────────
        /** Balls preloaded in robot — fired at shoot zone 1. */
        public static final int    PRELOAD_BALL_COUNT = 3;
        /** Balls picked up from the field — fired at shoot zone 2. */
        public static final int    PICKUP_BALL_COUNT  = 5;

        // ── Timing ────────────────────────────────────────────────────────────
        /** Hard kill-switch: entire auto aborts after this many seconds. */
        public static final double AUTO_TIMEOUT_S     = 15.0;

        private AutoStartConstants() {}
    }

    // =========================================================================
    // VISION — hardware (camera physical positions)
    // =========================================================================

    /**
     * Camera names must match exactly what is configured in PhotonVision.
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
                    Units.inchesToMeters(10),    // 10" behind robot center
                    Units.inchesToMeters(10),    // 10" to the right
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
    // VISION — algorithm constants
    // =========================================================================

    public static final class VisionConstants {

        /** 2026 field AprilTag layout — used by PhotonPoseEstimator. */
        public static AprilTagFieldLayout kTagLayout;
        static {
            try {
                Path path = Filesystem.getDeployDirectory().toPath().resolve("custom_hub.json");
                kTagLayout = new AprilTagFieldLayout(path);
            } catch (IOException e) {
                DriverStation.reportError("Could not load custom AprilTag layout!", e.getStackTrace());
                kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
            }
        }

        // ── 2026 Rebuilt (AndyMark) field dimensions ──────────────────────────
        /** Full field length — blue wall to red wall (metres). */
        public static final double FIELD_LENGTH_M = 8.0;
        /** Full field width (metres). */
        public static final double FIELD_WIDTH_M  = 8.052;

        /**
         * Maximum pose ambiguity ratio accepted from a single-tag solve.
         * Range [0, 1]; lower = stricter. 0.2 is a good starting point.
         */
        public static final double kAmbiguityThreshold = 0.2;

        // ── Yaw-alignment PID ─────────────────────────────────────────────────
        public static final double kP = 0.06;
        public static final double kI = 0.002;
        public static final double kD = 0.005;
        /** Acceptable yaw error before "aligned" is declared (degrees). */
        public static final double angleTolerance = 0.5;

        // ── Camera geometry ───────────────────────────────────────────────────
        /** Height of the camera lens above the floor (metres). */
        public static final double CAMERA_HEIGHT_METERS  = Units.inchesToMeters(20);
        /** Height of a generic tag target centre above floor (metres). */
        public static final double TARGET_HEIGHT_METERS  = Units.inchesToMeters(12.9);
        /** Camera pitch angle (radians, positive = tilted up). */
        public static final double CAMERA_PITCH_RADIANS  = Units.degreesToRadians(20);
        /** Height of the hub AprilTag centre above the floor (metres). Measure and verify. */
        public static final double HUB_TAG_HEIGHT_METERS = Units.inchesToMeters(24.0);

        private VisionConstants() {}
    }

    // =========================================================================
    // DRIVE TO POSE — ProfiledPID gains & tolerances
    // =========================================================================

    public static final class DriveToPoseConstants {

        // ── Translation ProfiledPID ───────────────────────────────────────────
        public static final double kP_XY         = 3.5;
        public static final double kI_XY         = 0.0;
        public static final double kD_XY         = 0.15;
        public static final double MAX_VEL_MPS   = 3.0;
        public static final double MAX_ACCEL_MPS2 = 4.0;

        // ── Heading ProfiledPID ───────────────────────────────────────────────
        public static final double kP_THETA          = 5.5;
        public static final double kI_THETA          = 0.0;
        public static final double kD_THETA          = 0.25;
        public static final double MAX_OMEGA_RAD_S   = Math.PI * 2.0;
        public static final double MAX_ALPHA_RAD_S2  = Math.PI * 4.0;

        // ── At-goal tolerances ────────────────────────────────────────────────
        /** XY tolerance before "at goal" (metres). ~1.5 inches. */
        public static final double XY_TOLERANCE_M      = Units.inchesToMeters(1.5);
        /** Heading tolerance before "at goal" (radians). 1 degree. */
        public static final double THETA_TOLERANCE_RAD = Units.degreesToRadians(1.0);

        // ── Debounce & safety ─────────────────────────────────────────────────
        public static final double DEBOUNCE_S        = 0.10;
        public static final double MAX_RUNTIME_S     = 5.0;
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
         * Hub centre position on the custom 8 m × 8.052 m field (metres).
         * Derived from the four hub AprilTag positions in custom_hub.json:
         *   Tag 19: (4.0000, 4.026), Tag 9: (5.1938, 4.026) → X midpoint = 4.5969
         *   Tag 8:  (4.5969, 4.6229), Tag 11: (4.5969, 3.4291) → Y midpoint = 4.026
         */
        public static final Translation2d HUB_CENTER = new Translation2d(4.5969, 4.026);

        /**
         * Height of the hub opening above the floor (metres).
         * Measure from the 2026 game manual or physical field.
         */
        public static final double HUB_TARGET_HEIGHT_METERS = 2.64;

        /** AprilTag IDs attached to the hub. Verify against 2026 game manual. */
        public static final int[] HUB_APRIL_TAG_IDS = {8, 9, 11, 19};

        // ── Shooter mechanism geometry ────────────────────────────────────────
        /** Height of the game-piece exit point above the floor (metres). */
        public static final double SHOOTER_EXIT_HEIGHT_METERS = 0.52;
        /** Horizontal offset from robot centre to shooter exit (metres). */
        public static final double SHOOTER_X_OFFSET_METERS    = 0.20;

        // ── Flywheel / motor hardware ─────────────────────────────────────────
        /** Shooter wheel diameter (inches). */
        public static final double WHEEL_DIAMETER_INCHES = 4.0;
        /** Gear ratio: motor turns per wheel turn. 1.0 = direct drive. */
        public static final double SHOOTER_GEAR_RATIO    = 1.0;
        /**
         * Compression efficiency: fraction of flywheel surface speed that
         * transfers to the game piece (typical 0.80–0.95).
         */
        public static final double EFFICIENCY            = 0.85;
        /** Motor RPM cap — shots above this are rejected. */
        public static final double MAX_MOTOR_RPM         = 6000.0;
        /** Idle RPM: speed held while command is active but not firing. */
        public static final double IDLE_RPM              = 500.0;

        // ── Pivot (hood) hardware ─────────────────────────────────────────────
        /** Pivot leader CAN ID. ⚠ Update to real ID before competition. */
        public static final int    PIVOT_LEADER_ID          = 99;
        /** Pivot follower CAN ID. ⚠ Update to real ID before competition. */
        public static final int    PIVOT_FOLLOWER_ID         = 99;
        /** CAN bus name for pivot motors. */
        public static final String PIVOT_CAN_BUS             = "ChassisCAN";
        /** Pivot gear ratio: motor turns per mechanism degree. */
        public static final double PIVOT_GEAR_RATIO          = 1.0;
        /** Motion Magic cruise velocity (rot/s). */
        public static final double PIVOT_CRUISE_VELOCITY     = 100.0;
        /** Motion Magic acceleration (rot/s²). */
        public static final double PIVOT_ACCELERATION        = 200.0;
        /** Pivot PID — Slot 0. */
        public static final double PIVOT_kP                  = 2.0;
        public static final double PIVOT_kI                  = 0.0;
        public static final double PIVOT_kD                  = 0.1;
        /** Acceptable pivot angle error for "at target" (degrees). */
        public static final double PIVOT_ANGLE_TOLERANCE_DEG = 1.0;

        // ── Shot angle optimizer ──────────────────────────────────────────────
        public static final double MIN_LAUNCH_ANGLE_DEG = 20.0;
        public static final double MAX_LAUNCH_ANGLE_DEG = 75.0;
        /** Sweep resolution (degrees). Smaller = more precise, slightly more startup CPU. */
        public static final double ANGLE_SWEEP_STEP_DEG = 0.5;
        /**
         * Scoring weights [flightTime, v₀, entryAngle, motorRPM] — must sum to 1.0.
         * Higher entryAngle weight = accuracy-focused.
         */
        public static final double[] SHOT_SCORE_WEIGHTS = {0.15, 0.15, 0.55, 0.15};
        /** Minimum hub entry angle for a valid shot (degrees). */
        public static final double MIN_ENTRY_ANGLE_DEG  = 25.0;

        // ── Alignment setpoints & tolerances ─────────────────────────────────
        /** Fallback standoff (metres) — replaced at runtime by ShotCalculator.OPTIMAL_STANDOFF_M. */
        public static final double DESIRED_DISTANCE_METERS   = 2.5;
        /** Acceptable distance error to allow firing (metres). */
        public static final double DISTANCE_TOLERANCE_METERS = 0.05;

        // ── Optimal-distance sweep ────────────────────────────────────────────
        public static final double OPTIMAL_DIST_MIN_M  = 1.5;
        public static final double OPTIMAL_DIST_MAX_M  = 6.0;
        public static final double OPTIMAL_DIST_STEP_M = 0.10;

        /** Acceptable heading error to allow firing (degrees). */
        public static final double YAW_TOLERANCE_DEG = 1.5;

        // ── Drive PID — HubAlignController ───────────────────────────────────
        public static final double DRIVE_kP              = 3.5;
        public static final double DRIVE_kD              = 0.15;
        public static final double DRIVE_MAX_SPEED_MPS   = 3.0;
        public static final double DRIVE_MAX_ACCEL       = 4.5;

        public static final double STRAFE_kP             = 3.5;
        public static final double STRAFE_kD             = 0.15;
        public static final double STRAFE_MAX_SPEED_MPS  = 2.5;

        public static final double ROTATION_kP                = 5.5;
        public static final double ROTATION_kD                = 0.25;
        public static final double ROTATION_MAX_RAD_S         = Math.PI * 2.0;
        public static final double ROTATION_MAX_ACCEL_RAD_S2  = Math.PI * 4.0;

        /** Low-pass filter alpha for drive output smoothing. Range (0, 1). */
        public static final double OUTPUT_FILTER_ALPHA = 0.20;
        /** Minimum output speed (m/s or rad/s) — below this output is zeroed. */
        public static final double VELOCITY_DEADBAND   = 0.01;

        /** Gravitational acceleration (m/s²). */
        public static final double g = 9.81;

        private ShooterConstants() {}
    }
}