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
 *   AutoConstants       — autonomous path-following gains
 *   AutoStartConstants  — starting positions and auto sequence parameters
 *   VisionHardware      — camera names, physical Transform3d offsets
 *   VisionConstants     — tag layout, Kalman/PID tuning, distance thresholds
 *   DriveToPoseConstants— ProfiledPID gains for DriveToPose command
 *   ShooterConstants    — hub geometry, ballistics parameters, alignment PID
 *   IntakeConstants     — intake actuation and roller constants
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * SHOOTER CALIBRATION NOTE (updated):
 *
 *   The original model used EFFICIENCY = 0.85, predicting ~900–1100 RPM.
 *   Physical field measurement shows 1850–3600 RPM is actually required.
 *   Back-calculation from 4 confirmed data points gives η_real ≈ 0.42.
 *
 *   Root causes of the gap:
 *     1. Foam ball compression absorbs significant energy on contact
 *     2. Ball slip during the acceleration phase through the wheels
 *     3. Two-wheel simultaneous grip dynamics
 *     4. Mechanical drivetrain losses not captured by surface-speed model
 *
 *   Fix: Do NOT use the physics v₀ model to compute RPM.
 *        Use ShooterConstants.interpolateRPM(distanceMeters) instead.
 *        The RPM_DISTANCE_TABLE is the source of truth — all entries
 *        marked "confirmed" were measured on the physical field.
 * ─────────────────────────────────────────────────────────────────────────────
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
            new Translation2d( Units.inchesToMeters(-10), Units.inchesToMeters(10)),
            new Translation2d( Units.inchesToMeters(-10), Units.inchesToMeters(-10))
        );

        private AutoConstants() {}
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

        /** Balls to shoot from preload (first shot zone). Robot can hold up to 8. */
        public static final int PRELOAD_BALL_COUNT = 8;
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
    // CONTROL DEADZONES — unified deadband constants
    // =========================================================================

    public static final class ControlDeadbands {
        // Driver inputs (joystick)
        public static final double DRIVER_ROTATION_DEADBAND = 0.12;  // Slightly tighter for precision
        public static final double DRIVE_REQUEST_DEADBAND   = 0.10;  // 10% of max speed

        // Operator inputs
        public static final double TURRET_STICK_DEADBAND    = 0.10;
        public static final double OPERATOR_ROTATION_DEADBAND = 0.12;

        // Output velocity thresholds — below these, outputs are zeroed
        public static final double VELOCITY_OUTPUT_DEADBAND = 0.01;  // m/s
        public static final double ANGULAR_OUTPUT_DEADBAND  = 0.01;  // rad/s

        // Vision & alignment
        public static final double VISION_ANGLE_DEADBAND    = 0.50;  // degrees
        public static final double ALIGNMENT_OUTPUT_DEADBAND = 0.015; // m/s

        private ControlDeadbands() {}
    }

    // =========================================================================
    // VISION — hardware (camera physical positions)
    // =========================================================================

    public static final class VisionHardware {

        /** Name used in PhotonVision UI for the rear-right camera. */
        public static final String CAMERA_BACK_RIGHT = "Camera-BackRight";
        /** Name used in PhotonVision UI for the rear-left camera. */
        public static final String CAMERA_BACK_LEFT  = "Camera-BackLeft";

        /**
         * All camera offsets keyed by camera name.
         * Transform3d is robot-center → camera (forward=+X, left=+Y, up=+Z).
         */
        public static final Map<String, Transform3d> kCameraOffsets = Map.of(
            CAMERA_BACK_RIGHT, new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(10),
                    Units.inchesToMeters(10),
                    Units.inchesToMeters(20)),
                new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(90))
            ),
            CAMERA_BACK_LEFT, new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-10),
                    Units.inchesToMeters(10),
                    Units.inchesToMeters(20)),
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

        /** Full field length — from blue wall to red wall (metres). */
        public static final double FIELD_LENGTH_M = 8.0;
        /** Full field width (metres). */
        public static final double FIELD_WIDTH_M  = 8.052;

        /** Maximum pose ambiguity ratio accepted from a single-tag solve. */
        public static final double kAmbiguityThreshold = 0.2;

        // ── Yaw-alignment PID ─────────────────────────────────────────────────
        public static final double kP = 0.06;
        public static final double kI = 0.002;
        public static final double kD = 0.005;
        /** Acceptable yaw error before "aligned" is declared (degrees). */
        public static final double angleTolerance = 0.5;

        // ── Turret aiming blend weight ────────────────────────────────────────
        /** Vision-to-odometry blend weight for turret aiming.
         *  0.0 = pure odometry (FireControlSolver)
         *  1.0 = pure vision (AprilTag bearing)
         *  0.6 = balanced (60% vision, 40% odometry) — tuned for SOTM + vision blend
         */
        public static final double TURRET_VISION_WEIGHT = 0.6;

        // ── Camera geometry ───────────────────────────────────────────────────
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(20);
        public static final double TARGET_HEIGHT_METERS  = Units.inchesToMeters(12.9);
        public static final double CAMERA_PITCH_RADIANS  = Units.degreesToRadians(20);
        /** Height of the hub AprilTag center above the floor (meters). */
        public static final double HUB_TAG_HEIGHT_METERS = Units.inchesToMeters(24.0);

        private VisionConstants() {}
    }

    // =========================================================================
    // DRIVE TO POSE — ProfiledPID gains & tolerances
    // =========================================================================

    public static final class DriveToPoseConstants {

        public static final double kP_XY         = 3.5;
        public static final double kI_XY         = 0.05;    // Small: eliminates last 1–2 cm creep
        public static final double kD_XY         = 0.15;
        public static final double MAX_VEL_MPS   = 3.0;
        public static final double MAX_ACCEL_MPS2 = 4.0;

        public static final double kP_THETA      = 5.5;
        public static final double kI_THETA      = 0.0;
        public static final double kD_THETA      = 0.25;
        public static final double MAX_OMEGA_RAD_S  = Math.PI * 2.0;
        public static final double MAX_ALPHA_RAD_S2 = Math.PI * 4.0;

        /** XY tolerance before "at goal" (metres). ~1.5 inches. */
        public static final double XY_TOLERANCE_M      = Units.inchesToMeters(1.5);
        /** Heading tolerance before "at goal" (radians). 1 degree. */
        public static final double THETA_TOLERANCE_RAD = Units.degreesToRadians(1.0);

        /** Robot must stay within tolerance for this long before isFinished (seconds). */
        public static final double DEBOUNCE_S        = 0.10;
        /** Kill-switch: abort if the command runs longer than this (seconds). */
        public static final double MAX_RUNTIME_S     = 5.0;
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
     * ── Physics model (for trajectory/entry-angle calculations only) ──────────
     *   v₀ = d · √( g / (2·cos²θ · (d·tanθ − Δh)) )
     *   entry_angle = atan2(−vy_final, vx)
     *
     * ── RPM source of truth ───────────────────────────────────────────────────
     *   Always call interpolateRPM(distanceMeters).
     *   Never derive RPM from the physics model — η_real ≈ 0.42, not 0.85.
     */
    public static final class ShooterConstants {

        // ── Field / Hub geometry ──────────────────────────────────────────────
        /**
         * Hub center on the custom 8 m × 8.052 m field (metres).
         * Derived from the four hub AprilTag positions in custom_hub.json.
         */
        public static final Translation2d HUB_CENTER =
                new Translation2d(4.5969, 4.026);

        /** Height of the hub opening above the floor (meters). */
        public static final double HUB_TARGET_HEIGHT_METERS = 2.64;

        /** AprilTag IDs attached to the hub. */
        public static final int[] HUB_APRIL_TAG_IDS = {8, 9, 11, 19};

        // ── Shooter mechanism geometry ────────────────────────────────────────
        /** Height of the game-piece exit point above the floor (meters). */
        public static final double SHOOTER_EXIT_HEIGHT_METERS = 0.52;

        /** Horizontal distance from robot center to shooter exit (meters). */
        public static final double SHOOTER_X_OFFSET_METERS = 0.20;

        // ── Fixed hood angle ──────────────────────────────────────────────────
        /**
         * The hood (pivot) is physically fixed — it cannot be adjusted during a match.
         * All shot calculations must use this angle.
         * Previously stored as FIXED_SHOT_ANGLE_DEG = 45.0 (incorrect/placeholder).
         */
        public static final double FIXED_SHOT_ANGLE_DEG = 61.5; // Confirmed physical angle

        // ── Flywheel hardware ─────────────────────────────────────────────────
        public static final int     FLYWHEEL_LEADER_ID       = 29;
        public static final int     FLYWHEEL_FOLLOWER_ID     = 30;
        public static final String  FLYWHEEL_CAN_BUS         = "rio";
        public static final boolean FLYWHEEL_FOLLOWER_OPPOSE = true;

        /** Shooter wheel diameter (inches). */
        public static final double WHEEL_DIAMETER_INCHES = 4.0;

        /** Gear ratio: motor turns per one wheel turn. 1.0 = direct drive. */
        public static final double SHOOTER_GEAR_RATIO = 1.0;

        /**
         * Compression efficiency — CORRECTED from 0.85 to empirical value.
         *
         * Back-calculated from 4 confirmed field measurements:
         *   1.5 m → 1850 RPM  → η = 0.410
         *   2.5 m → 2200 RPM  → η = 0.426
         *   4.0 m → 2680 RPM  → η = 0.436
         *   5.5 m → 3600 RPM  → η = 0.425
         *   Average           → η ≈ 0.424
         *
         * NOTE: This value is retained for trajectory/entry-angle display purposes.
         * DO NOT use it to compute target RPM — use interpolateRPM() instead.
         */
        public static final double EFFICIENCY = 0.424; // corrected (was 0.85)

        /** Motor RPM cap — shots requiring more than this are rejected. */
        public static final double MAX_MOTOR_RPM = 6000.0;

        /** Idle RPM held while the command is active but not firing. */
        public static final double IDLE_RPM = 500.0;

        /** Stator current limit for flywheel motors (Amps). */
        public static final double FLYWHEEL_STATOR_LIMIT_AMPS = 60.0;

        // ── Flywheel PID / feedforward (VelocityTorqueCurrentFOC, Phoenix Pro) ──
        // Gains are in Amps. Derived from SysId voltage values ÷ Kt (~0.0181 for Kraken/Falcon).
        public static final double FLYWHEEL_kP = 25.0;   // Kept at 25.0 for good RPM tracking
        public static final double FLYWHEEL_kI = 0.8;    // Reduced from 1.2 to avoid overshoot on distance changes
        public static final double FLYWHEEL_kD = 0.1;    // Added small D term for transient damping
        public static final double FLYWHEEL_kS = 12.426;
        public static final double FLYWHEEL_kV = 0.1565;
        public static final double FLYWHEEL_kA = 0.0;

        // ── Distance → RPM table (SOURCE OF TRUTH for RPM) ───────────────────
        /**
         * Monotone cubic-spline interpolation anchors.
         * ★ = physically confirmed on the field.
         * All other rows are spline-interpolated from those four anchors.
         *
         * To add more confirmed points: measure on the field, mark with ★,
         * and update the interpolator — the spline will reshape automatically.
         *
         * Format: { distance_meters, target_rpm }
         */
        public static final double[][] RPM_DISTANCE_TABLE = {
            { 1.153,  1930.0 },  // ★ confirmed  (was 1.50 m bumper→face)
            { 1.400,  2000.0 },  // interpolated
            { 1.650,  2055.0 },  // interpolated — reduced to prevent overshooting at 2M
            { 1.900,  2145.0 },  // interpolated — reduced slightly
            { 2.153,  2350.0 },  // ★ confirmed  (was 2.50 m bumper→face) — increased from 2310
            { 2.400,  2480.0 },  // interpolated — increased from 2430
            { 2.650,  2600.0 },  // interpolated — increased from 2550  
            { 2.900,  2710.0 },  // interpolated — increased from 2660  
            { 3.153,  2790.0 },  // interpolated — increased from 2740  
            { 3.400,  2820.0 },  // interpolated — increased from 2770
            { 3.653,  2850.0 },  // ★ confirmed  (was 4.00 m bumper→face) — increased from 2800
            { 4.000,  2950.0 },  // interpolated — increased from 2900
            { 4.400,  3090.0 },  // interpolated — increased from 3040
            { 4.800,  3270.0 },  // interpolated — increased from 3220
            { 5.153,  3470.0 },  // ⚠ unconfirmed (would be 5.50 m bumper→face) — increased from 3420
        };

        // ── D-Pad operator presets ────────────────────────────────────────────
        // Tape distances (bumper→face) shown in comments for field reference.
        // The _DIST_M values are physics coordinates (shooter-exit→hub-center).add
        // When driving to a preset, command: bumper→face = _DIST_M + DIST_MEAS_CORRECTION_M
        // ── Named RPM presets (used by PathPlanner auto named commands) ─────────
        public static final double PRESET_CLOSE_RPM    = 1980.0; // 1.5m
        public static final double PRESET_CLOSE_DIST_M = 1.153;  // tape: 1.5 m
        public static final double PRESET_MID_RPM      = 2350.0; // increased from 2510 for 2.5m
        public static final double PRESET_MID_DIST_M   = 2.153;  // tape: 2.5 m
        public static final double PRESET_FAR_RPM      = 2850.0; // increased from 2820 for 4.0m
        public static final double PRESET_FAR_DIST_M   = 3.653;  // tape: 4.0 m
        public static final double PRESET_VFAR_RPM     = 3470.0; // increased from 3820 for 5.5m
        public static final double PRESET_VFAR_DIST_M  = 5.153;  // tape: 5.5 m  ⚠ unconfirmed

        // ── D-Pad operator distance presets (used by RobotContainer D-pad bindings) ──
        // RPM is interpolated live from RPM_DISTANCE_TABLE via setFlywheelRPMFromDistance().
        // _DIST_M values are shooter-exit → hub-center distances.
        // Tape (bumper→face) distances shown in comments for field reference.
        public static final double PRESET_2M_DIST_M   = 1.653; // tape: 2.0 m
        public static final double PRESET_3M_DIST_M   = 2.653; // tape: 3.0 m
        public static final double PRESET_3_5M_DIST_M = 3.153; // tape: 3.5 m
        public static final double PRESET_4_5M_DIST_M = 4.153; // tape: 4.5 m
        public static final double PRESET_5M_DIST_M   = 4.653; // tape: 5.0 m


        /**
         * Interpolates target RPM from RPM_DISTANCE_TABLE using linear
         * interpolation between the two nearest rows.
         *
         * This is the ONLY correct way to get a target RPM in ShotCalculator.
         * Do not call the physics v₀ model for RPM — it assumes η = 0.85
         * which is wrong for this mechanism (η_real ≈ 0.42).
         *
         * @param distMeters horizontal distance from shooter exit to hub center
         * @return target flywheel RPM, clamped to table bounds
         */
        public static double interpolateRPM(double distMeters) {
            double[][] tbl = RPM_DISTANCE_TABLE;
            if (distMeters <= tbl[0][0])              return tbl[0][1];
            if (distMeters >= tbl[tbl.length - 1][0]) return tbl[tbl.length - 1][1];
            for (int i = 0; i < tbl.length - 1; i++) {
                if (distMeters >= tbl[i][0] && distMeters <= tbl[i + 1][0]) {
                    double t = (distMeters - tbl[i][0]) / (tbl[i + 1][0] - tbl[i][0]);
                    return tbl[i][1] + t * (tbl[i + 1][1] - tbl[i][1]);
                }
            }
            return tbl[tbl.length - 1][1];
        }

        // ── Pivot (hood) hardware — FIXED, no motion needed ──────────────────
        // Hood is physically locked at FIXED_SHOT_ANGLE_DEG.
        // Pivot motor IDs retained in case a future robot re-enables adjustment.
        public static final int    PIVOT_LEADER_ID   = 99; // unused — hood fixed
        public static final int    PIVOT_FOLLOWER_ID = 99; // unused — hood fixed
        public static final String PIVOT_CAN_BUS     = "rio";

        public static final double PIVOT_GEAR_RATIO        = 1.0;
        public static final double PIVOT_CRUISE_VELOCITY   = 100.0;
        public static final double PIVOT_ACCELERATION      = 200.0;
        public static final double PIVOT_kP                = 2.0;
        public static final double PIVOT_kI                = 0.05;   // Added integral for steady-state accuracy
        public static final double PIVOT_kD                = 0.1;
        public static final double PIVOT_ANGLE_TOLERANCE_DEG = 0.5;  // Tightened from 1.0° for better shot consistency

        // ── Shot angle optimizer (fixed angle — sweep is informational only) ──
        public static final double MIN_LAUNCH_ANGLE_DEG  = 20.0;
        public static final double MAX_LAUNCH_ANGLE_DEG  = 75.0;
        public static final double ANGLE_SWEEP_STEP_DEG  = 0.5;

        /**
         * Scoring weights [flightTime, v₀, entryAngle, motorRPM].
         * Sum = 1.0. Higher entryAngle weight = accuracy-focused.
         * Used by ShotCalculator for informational scoring only;
         * RPM is now determined by interpolateRPM(), not the optimizer.
         */
        public static final double[] SHOT_SCORE_WEIGHTS = {0.15, 0.15, 0.55, 0.15};

        /** Minimum hub entry angle for a shot to be considered valid (degrees). */
        public static final double MIN_ENTRY_ANGLE_DEG = 25.0;

        // ── Alignment setpoints & tolerances ─────────────────────────────────
        /** Fallback desired standoff distance (meters). */
        public static final double DESIRED_DISTANCE_METERS   = 2.5;
        /** Acceptable distance error to allow firing (meters). */
        public static final double DISTANCE_TOLERANCE_METERS = 0.05;
        /** Acceptable heading error to allow firing (degrees). */
        public static final double YAW_TOLERANCE_DEG         = 1.5;

        // ── Optimal-distance sweep ────────────────────────────────────────────
        public static final double OPTIMAL_DIST_MIN_M  = 1.5;
        public static final double OPTIMAL_DIST_MAX_M  = 6.0;
        public static final double OPTIMAL_DIST_STEP_M = 0.10;

        // ── Drive PID — HubAlignController ───────────────────────────────────
        public static final double DRIVE_kP            = 3.5;
        public static final double DRIVE_kD            = 0.15;
        public static final double DRIVE_MAX_SPEED_MPS = 3.0;
        public static final double DRIVE_MAX_ACCEL     = 4.5;

        public static final double STRAFE_kP            = 3.5;
        public static final double STRAFE_kD            = 0.15;
        public static final double STRAFE_MAX_SPEED_MPS = 2.5;

        public static final double ROTATION_kP              = 5.5;
        public static final double ROTATION_kD              = 0.25;
        public static final double ROTATION_MAX_RAD_S       = Math.PI * 2.0;
        public static final double ROTATION_MAX_ACCEL_RAD_S2 = Math.PI * 4.0;

        /**
         * Low-pass filter alpha for drive output smoothing.
         * Range (0, 1): smaller = more smoothing, larger = less smoothing.
         */
        public static final double OUTPUT_FILTER_ALPHA = 0.20;

        /** Minimum output speed (m/s or rad/s) — below this, output is zeroed. */
        public static final double VELOCITY_DEADBAND = 0.01;

        /** Gravitational acceleration (m/s²). */
        public static final double g = 9.81;

        public static final double FIXED_SHOT_RPM_M = 2300;

        private ShooterConstants() {}
    }

    // =========================================================================
    // TURRET
    // =========================================================================

    public static final class TurretConstants {

        public static final int    TURRET_MOTOR_ID  = 32;
        public static final String TURRET_CAN_BUS   = "rio";

        /** Motor turns per one full turret rotation. */
        public static final double TURRET_GEAR_RATIO = 20.0;

        // ── Cord-safety soft limits ───────────────────────────────────────────
        // Convention: positive = RIGHT, negative = LEFT (Clockwise_Positive inversion).
        // Physical hard stops measured on robot: right +279.75°, left -266.22°.
        // INCREASED margin from 15° to 25° for safety during robot tilt/deceleration.
        // FORWARD must always be > REVERSE or Phoenix will reject the config.
        public static final double TURRET_FORWARD_LIMIT_DEG =  260.0; 
        public static final double TURRET_REVERSE_LIMIT_DEG = -260.0; 

        // ── MotionMagic profile (used during limit-flip) ──────────────────────
        public static final double TURRET_CRUISE_VEL_RPS = 6.0;   // output-shaft rot/s during flip
        public static final double TURRET_ACCEL_RPS2     = 10.0;  // output-shaft rot/s² — snappy ramp

        // ── Position PID (Slot 0, MotionMagicVoltage) ────────────────────────
        public static final double TURRET_kP = 24.0;
        public static final double TURRET_kI =  0.0;
        public static final double TURRET_kD =  0.5;
        public static final double TURRET_kS =  0.0;
        public static final double TURRET_kV =  0.0;

        public static final double TURRET_ANGLE_TOLERANCE_DEG = 1.0;
        public static final double TURRET_STATOR_LIMIT_AMPS   = 40.0;

        private TurretConstants() {}
    }

    // =========================================================================
    // CLIMBER
    // =========================================================================

    public static final class ClimberConstants {

        // ── Hardware ─────────────────────────────────────────────────────────
        public static final int    LEADER_ID   = 15;
        public static final int    FOLLOWER_ID = 16;
        public static final String CAN_BUS     = "ChassisCAN";

        // ── Mechanism ────────────────────────────────────────────────────────
        /** Motor turns per one output shaft turn. 1.0 = direct drive. */
        public static final double GEAR_RATIO      = 1.0;

        // ── MotionMagic profile ───────────────────────────────────────────────
        public static final double CRUISE_VELOCITY = 100.0; // rot/s
        public static final double ACCELERATION    = 200.0; // rot/s²

        // ── Slot 0 gains ──────────────────────────────────────────────────────
        public static final double kP = 2.0;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
        /** Gravity feedforward (Volts) — opposes gravity when arm is extended. */
        public static final double kG = 0.2;

        // ── Stator current limit ──────────────────────────────────────────────
        public static final double STATOR_LIMIT_AMPS = 40.0;

        /**
         * Motor encoder degrees for the Level 1 rung position.
         *
         * ── How to tune ──────────────────────────────────────────────────────
         * 1. Manually jog the climber up until the hooks contact the Level 1 rung.
         * 2. Read the current position from SmartDashboard → Climber/Position_rot.
         * 3. Multiply by 360 to convert rotations → degrees.
         * 4. Replace the placeholder below with that value.
         *
         * The Level 1 rung is 27 in (0.686 m) above the carpet.
         */
        public static final double LEVEL1_CLIMB_DEGREES = 720.0; // ⚠ placeholder — tune on robot

        /** Position tolerance for "at target" check (degrees). */
        public static final double CLIMB_POSITION_TOLERANCE_DEG = 18.0; // ≈ 0.05 rotations

        private ClimberConstants() {}
    }

    // =========================================================================
    // FEEDER
    // =========================================================================

    public static final class FeederConstants {
        /** Feeder motor CAN ID. */
        public static final int FEEDER_MOTOR_ID = 25;

        /** Normal feeder duty cycle (forward, 0–1 fraction of battery voltage). */
        public static final double FEEDER_DUTY_NORMAL = 0.60;  // Reduced from 0.65 for safety margin

        /** Reverse feeder duty cycle (backward, 0–1). */
        public static final double FEEDER_DUTY_REVERSE = -0.40;

        /** Supply current limit to prevent jamming and brownout (Amps). */
        public static final double FEEDER_CURRENT_LIMIT_AMPS = 20.0;

        /** Current threshold above which a jam is suspected (Amps). */
        public static final double FEEDER_JAM_DETECTION_AMPS = 18.0;

        private FeederConstants() {}
    }

    // =========================================================================
    // INTAKE
    // =========================================================================

    public static final class IntakeConstants {

        // ── Actuation motor ───────────────────────────────────────────────────
        /** Actuation motor CAN ID. */
        public static final int    ACTUATION_MOTOR_ID      = 26;
        public static final String ACTUATION_CAN_BUS       = "rio";
        public static final double ACTUATION_CRUISE_VEL    = 100.0; // rot/s
        public static final double ACTUATION_ACCELERATION  = 200.0; // rot/s²
        public static final double ACTUATION_kP            = 2.0;
        public static final double ACTUATION_kI            = 0.0;
        public static final double ACTUATION_kD            = 0.1;
        /** Forward soft limit (rotations) — fully deployed position. */
        public static final double ACTUATION_FORWARD_LIMIT = 50.0;
        /** Peak voltage (V) clamp for actuation motor. */
        public static final double ACTUATION_PEAK_VOLTAGE  = 8.0;

        // ── Uptake motor ──────────────────────────────────────────────────────
        public static final int    UPTAKE_MOTOR_ID     = 42;
        public static final String UPTAKE_CAN_BUS      = "ChassisCAN";
        public static final double UPTAKE_CRUISE_VEL   = 100.0; // rot/s
        public static final double UPTAKE_ACCELERATION = 200.0; // rot/s²
        public static final double UPTAKE_kP           = 2.0;
        public static final double UPTAKE_kI           = 0.0;
        public static final double UPTAKE_kD           = 0.1;
        public static final double UPTAKE_PEAK_VOLTAGE = 8.0;

        // ── Intake roller motors ──────────────────────────────────────────────
        public static final int    ROLLER_LEADER_ID    = 27;
        public static final int    ROLLER_FOLLOWER_ID  = 28;
        public static final String ROLLER_CAN_BUS      = "rio";
        public static final double ROLLER_STATOR_LIMIT = 40.0; // Amps
        public static final double ROLLER_SUPPLY_LIMIT = 30.0; // Amps

        // ── Arm positions & powers ────────────────────────────────────────────
        /** Target position (rotations) when the arm is fully deployed. */
        public static final double DEPLOYED_ROTATIONS = 11.33;
        /** Target position (rotations) when the arm is fully stowed. */
        public static final double STOWED_ROTATIONS   = 0.0;
        /** Bar stops spinning when arm is within this many rotations of stow. */
        public static final double STOW_TOLERANCE     = 0.5;

        /** Duty-cycle power for the uptake roller (0–1). */
        public static final double ROLLER_POWER = 0.65;
        /** Duty-cycle power for the intake bar/rod motors (0–1). */
        public static final double BAR_POWER    = 0.7;

        private IntakeConstants() {}
    }
}