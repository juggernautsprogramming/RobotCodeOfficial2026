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
        /** Default starting pose (field-relative, metres). Used when no PathPlanner auto resets pose. */
        public static final double DEFAULT_START_X   = 3.476;
        public static final double DEFAULT_START_Y   = 4.003;
        public static final double DEFAULT_START_HDG = 0.0; // degrees — facing hub (~1.2° off, near enough)
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
        public static final double DRIVER_ROTATION_DEADBAND = 0.18;  // Slightly tighter for precision
        public static final double DRIVE_REQUEST_DEADBAND   = 0.13;  // 10% of max speed

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

        // ── PhotonVision camera names (must match names set in PhotonVision UI) ──
        public static final String CAMERA_LEFT  = "Camera_Left";   // facing left,  zθ=270°
        public static final String CAMERA_BACK  = "Camera_Back";       // facing back,  zθ=180°
        /** Turret-mounted camera — transform must be updated dynamically as turret rotates. */
        public static final String CAMERA_TURRET = "camera_turret";

        /**
         * Fixed-camera offsets from robot centre (forward=+X, left=+Y, up=+Z).
         * Rotation3d(roll, pitch, yaw):
         *   pitch < 0  = camera tilts upward (30° upward tilt → −30°)
         *   yaw        = horizontal pointing direction (90°=left, 180°=back, 270°=right)
         *
         * Values measured in CAD (inches → metres).
         * Note: left & right cameras share the same x/z; verify y-sign symmetry on robot.
         */
        public static final Map<String, Transform3d> kCameraOffsets = Map.of(
            // Left camera — faces left (zθ=270° CW → 90° CCW in WPILib), tilts up 30°
            CAMERA_LEFT, new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-13.747),//y
                    Units.inchesToMeters(10.489),//x
                    Units.inchesToMeters(25.880)),
                new Rotation3d(0,
                    Units.degreesToRadians(-30),  // upward tilt: negative pitch in WPILib convention
                    Units.degreesToRadians(90))   // facing left (270° in WPILib = -90° = left)
            ),
            // Back camera — faces backward (zθ=180°), tilts up 30°
            CAMERA_BACK, new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-10.511),
                    Units.inchesToMeters(10.872),
                    Units.inchesToMeters(25.880)),
                new Rotation3d(
                    0,
                    Units.degreesToRadians(-30),  // upward tilt: negative pitch in WPILib convention
                    Units.degreesToRadians(180))   // facing back
            ),
            // Right camera — faces right (zθ=90° CW → 270° CCW in WPILib), tilts up 30°
            /**CAMERA_BACK_RIGHT, new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-13.747),
                    Units.inchesToMeters(-10.489),
                    Units.inchesToMeters(25.880)),
                new Rotation3d(0,
                    Units.degreesToRadians(-30),   // upward tilt (yθ=30°)
                    Units.degreesToRadians(270))   // facing right
            ),*/
            // Turret camera — initial static transform (turret at 0°, facing forward).
            // Live transform is recomputed each loop by VisionSubsystem using kTurretCamTurretFrame.
            CAMERA_TURRET, new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(5.735),   // x: forward from turret pivot
                    Units.inchesToMeters(5.341),   // y: lateral from turret pivot (+y = left in WPILib)
                    Units.inchesToMeters(25.387)), // z: total height above robot centre
                new Rotation3d(0, Units.degreesToRadians(0), 0) 
            )
        );

        /**
         * Turret camera position/orientation in the TURRET'S LOCAL frame (turret pivot = origin).
         *
         * <p>Translation:
         *   x = 5.735 in forward from turret pivot
         *   y = 5.341 in lateral from turret pivot (+y = left in WPILib convention)
         *   z = 25.387 in total height above robot centre (measured directly)
         *
         * <p>Rotation: 67° upward pitch. Yaw is always 0 in turret frame (camera faces forward on turret).
         *   VisionSubsystem rotates x/y by the live turret angle and adds turret yaw each loop.
         */
        public static final Transform3d kTurretCamTurretFrame = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(5.735),   // x: forward from turret pivot
                Units.inchesToMeters(5.341),   // y: lateral from turret pivot (+y = left)
                Units.inchesToMeters(25.387)), // z: total height above robot centre
            new Rotation3d(0, Units.degreesToRadians(22), 0) // 67° upward pitch; yaw added dynamically
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
        public static final double TARGET_HEIGHT_METERS  = Units.inchesToMeters(44.25); // hub AprilTag center, 2026 REBUILT
        public static final double CAMERA_PITCH_RADIANS  = Units.degreesToRadians(20);
        /** Height of the hub AprilTag center above the floor (meters). 2026 REBUILT: 44.25 in. */
        public static final double HUB_TAG_HEIGHT_METERS = Units.inchesToMeters(44.25); // 1.1240 m

        /** AprilTag IDs used for robot-rotation alignment (AlignToTag + SnapAimAndShootCommand). */
        public static final int[] ALIGN_TAG_IDS = {26, 10, 21, 5, 18, 2, 20, 4};

        // ── Turret camera geometry (for PhotonUtils hub distance calculation) ──
        /** Height of the turret camera lens above the floor (meters).
         *  Derived from kTurretCamTurretFrame z-translation. */
        public static final double TURRET_CAM_HEIGHT_METERS = Units.inchesToMeters(25.387);
        /** Turret camera upward pitch used with PhotonUtils (radians, positive = looking up). */
        public static final double TURRET_CAM_PITCH_RADIANS  = Units.degreesToRadians(62.0);

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
         * Hub centers on the 2026 REBUILT field (metres, blue-alliance origin).
         * Red hub is the field mirror of the blue hub (X mirrored across 16.54 / 2 = 8.27 m).
         * TODO: verify X/Y against the official 2026 field-dimension drawings.
         */
        public static final Translation2d HUB_CENTER_BLUE = new Translation2d(4.5969, 4.026);
        public static final Translation2d HUB_CENTER_RED  = new Translation2d(16.54 - 4.5969, 4.026); // ≈ (11.94, 4.026)
        /** Backwards-compat alias — points to the blue hub center. */
        public static final Translation2d HUB_CENTER = HUB_CENTER_BLUE;

        /** Height of the hub opening above the floor (meters). 2026 REBUILT: 41 in. */
        public static final double HUB_TARGET_HEIGHT_METERS = Units.inchesToMeters(41); // 1.0414 m

        /**
         * AprilTag IDs on each alliance's hub.
         * TODO: verify split against the official 2026 AprilTag layout — lower IDs are
         *       typically red-side, higher IDs blue-side but confirm from the field manual.
         */
        public static final int[] HUB_APRIL_TAG_IDS_RED  = {2, 4, 5, 10};
        public static final int[] HUB_APRIL_TAG_IDS_BLUE = {18, 20, 21, 26};
        /** All hub tag IDs combined (both alliances). */
        public static final int[] HUB_APRIL_TAG_IDS = {26,21,18,20,10,5,2,4};

        // ── Shooter mechanism geometry ────────────────────────────────────────
        /** Height of the game-piece exit point above the floor (meters). */
        public static final double SHOOTER_EXIT_HEIGHT_METERS = 0.52;

        /** Horizontal distance from robot center to shooter exit (meters). */
        public static final double SHOOTER_X_OFFSET_METERS = 0.20;

        /**
         * Offset subtracted from odometry-based robot-centre→hub-centre distance
         * before RPM interpolation, converting it to the "shooter-exit→hub-face"
         * coordinate space the RPM table was calibrated in.
         *
         * <p>Breakdown: robot_centre_to_bumper (~0.50 m) + hub_face_to_hub_centre
         * (~0.20 m) + tape_to_physics_correction (0.347 m) ≈ 1.05 m.
         * Tune this value if shots still over/undershoot after distance is correct.
         */
        public static final double ODOMETRY_TO_RPM_TABLE_OFFSET_M = 1.09;

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
        public static final double FLYWHEEL_kP = 6.0;
        public static final double FLYWHEEL_kI = 0.0;
        public static final double FLYWHEEL_kD = 0.0;
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
            { 1.153,  1850.0 },  // ★ confirmed  (was 1.50 m bumper→face)
            { 1.400,  1920.0 },  // interpolated
            { 1.650,  1995.0 },  // interpolated
            { 1.900,  2075.0 },  // interpolated
            { 2.153,  2200.0 },  // ★ confirmed  (was 2.50 m bumper→face)
            { 2.400,  2310.0 },  // interpolated
            { 2.650,  2430.0 },  // interpolated
            { 2.900,  2540.0 },  // interpolated
            { 3.153,  2620.0 },  // interpolated
            { 3.400,  2650.0 },  // interpolated
            { 3.653,  2680.0 },  // ★ confirmed  (was 4.00 m bumper→face)
            { 4.000,  2780.0 },  // interpolated — extrapolated beyond confirmed range, verify on field
            { 4.400,  2920.0 },  // interpolated — extrapolated, verify on field
            { 4.800,  3100.0 },  // interpolated — extrapolated, verify on field
            { 5.153,  3300.0 },  // ⚠ unconfirmed (would be 5.50 m bumper→face) — MEASURE AND UPDATE
        };

        // ── D-Pad operator presets ────────────────────────────────────────────
        // Tape distances (bumper→face) shown in comments for field reference.
        // The _DIST_M values are physics coordinates (shooter-exit→hub-center).
        // When driving to a preset, command: bumper→face = _DIST_M + DIST_MEAS_CORRECTION_M
        public static final double PRESET_CLOSE_RPM    = 1850.0;
        public static final double PRESET_CLOSE_DIST_M = 1.153; // tape: 1.5 m  ★ confirmed
        public static final double PRESET_MID_RPM      = 2081.0;
        public static final double PRESET_MID_DIST_M   = 2.153; // tape: 2.5 m  ★ confirmed
        public static final double PRESET_FAR_RPM      = 2570.0;
        public static final double PRESET_FAR_DIST_M   = 3.653; // tape: 4.0 m  ★ confirmed
        public static final double PRESET_VFAR_RPM     = 3649.0; // ⚠ unconfirmed — measure on field
        public static final double PRESET_VFAR_DIST_M  = 5.153; // tape: 5.5 m  ⚠ unconfirmed

        // ── D-Pad operator distance presets (used by RobotContainer D-pad bindings) ──
        // RPM is interpolated live from RPM_DISTANCE_TABLE via setFlywheelRPMFromDistance().
        // _DIST_M values are shooter-exit → hub-center distances.
        // Tape (bumper→face) distances shown in comments for field reference.
        public static final double PRESET_2M_DIST_M   = 1.653; // tape: 2.0 m
        public static final double PRESET_3M_DIST_M   = 2.653; // tape: 3.0 m
        public static final double PRESET_3_5M_DIST_M = 3.153; // tape: 3.5 m
        public static final double PRESET_4_5M_DIST_M = 4.153; // tape: 4.5 m
        public static final double PRESET_5M_DIST_M   = 4.653; // tape: 5.0 m


        // ── Monotone cubic spline — built from confirmed RPM_DISTANCE_TABLE anchors ──
        // Anchor RPMs match the ★-confirmed entries in RPM_DISTANCE_TABLE above.
        // Built once at class-load; no allocations at runtime.
        private static final double[] SPLINE_X;
        private static final double[] SPLINE_Y;
        private static final double[] SPLINE_M; // tangent slopes at each knot
        static {
            // Physics distances = tape distance − DIST_CORRECTION (0.347 m)
            // RPM anchors match the confirmed entries in RPM_DISTANCE_TABLE (★ confirmed on field).
            double[] x = { 0.6153,0.953,1.153, 2.153, 3.653, 5.153 };
            double[] y = { 2170,2200,2460.0, 2540.0, 2620.0, 3260.0 };
            int n = x.length;
            double[] h     = new double[n - 1];
            double[] delta = new double[n - 1];
            double[] m     = new double[n];
            for (int i = 0; i < n - 1; i++) {
                h[i]     = x[i + 1] - x[i];
                delta[i] = (y[i + 1] - y[i]) / h[i];
            }
            // End-point slopes = first/last segment slope
            m[0]     = delta[0];
            m[n - 1] = delta[n - 2];
            for (int i = 1; i < n - 1; i++) m[i] = (delta[i - 1] + delta[i]) / 2.0;
            // Monotonicity enforcement (Steffen-style slope clamping)
            for (int i = 0; i < n - 1; i++) {
                if (Math.abs(delta[i]) < 1e-10) { m[i] = 0; m[i + 1] = 0; continue; }
                double a = m[i] / delta[i], b = m[i + 1] / delta[i];
                if (a < 0 || b < 0) { m[i] = 0; m[i + 1] = 0; continue; }
                double sq = a * a + b * b;
                if (sq > 9) { double tau = 3.0 / Math.sqrt(sq); m[i] *= tau; m[i + 1] *= tau; }
            }
            SPLINE_X = x;
            SPLINE_Y = y;
            SPLINE_M = m;
        }

        /**
         * Interpolates target RPM using a monotone cubic spline through the 3
         * field-confirmed anchor points. Matches the {@code interpRPM()} function
         * in {@code shooter_calculator.html} exactly.
         *
         * Outside the anchor range the spline extrapolates linearly using the
         * end-point tangent slope (same behaviour as the HTML).
         *
         * @param distMeters horizontal distance from shooter exit to hub center
         * @return target flywheel RPM
         */
        public static double interpolateRPM(double distMeters) {
            double[] x = SPLINE_X, y = SPLINE_Y, m = SPLINE_M;
            int n = x.length;
            if (distMeters <= x[0])      return y[0]     + m[0]     * (distMeters - x[0]);
            if (distMeters >= x[n - 1]) return y[n - 1] + m[n - 1] * (distMeters - x[n - 1]);
            int lo = 0, hi = n - 1;
            while (hi - lo > 1) { int mid = (lo + hi) >> 1; if (x[mid] <= distMeters) lo = mid; else hi = mid; }
            double seg = x[lo + 1] - x[lo];
            double t = (distMeters - x[lo]) / seg;
            double t2 = t * t, t3 = t2 * t;
            return (2*t3 - 3*t2 + 1)*y[lo]     + (t3 - 2*t2 + t)*seg*m[lo]
                 + (-2*t3 + 3*t2)*y[lo + 1] + (t3 - t2)*seg*m[lo + 1];
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

        public static final double FIXED_SHOT_RPM_M = 1995.0;

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
        public static final double TURRET_AIM_TRIM_DEG = 147;
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
        public static final int    MOTOR_ID   = 31;
        public static final String CAN_BUS     = "ChassisCAN";

        // ── Mechanism ────────────────────────────────────────────────────────
        /** Motor turns per one output shaft turn. 1.0 = direct drive. */
        public static final double GEAR_RATIO      = 1.0;

        // ── MotionMagic profile ───────────────────────────────────────────────
        public static final double CRUISE_VELOCITY = 100.0; // rot/s
        public static final double ACCELERATION    = 200.0; // rot/s²

        // ── Slot 0 gains ──────────────────────────────────────────────────────
        public static final double kP = 6.0;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
        /** Gravity feedforward (Volts) — opposes gravity when arm is extended. */
        public static final double kG = 0.8;

        // ── Stator current limit ──────────────────────────────────────────────
        public static final double STATOR_LIMIT_AMPS = 45.0;

        // ── Climber Positions (in motor rotations) ─────────────
        
        public static final double DOWN_POSITION_ROTATIONS = -40.4;  // your DOWN value
        public static final double UP_POSITION_ROTATIONS = 0;     // your UP value

        /** Position tolerance for "at target" check (rotations). */
        public static final double POSITION_TOLERANCE_ROTATIONS = 0.05; // ≈ 18 degrees

        private ClimberConstants() {}
    }

    // =========================================================================
    // FEEDER
    // =========================================================================

    public static final class FeederConstants {
        /** Feeder motor CAN ID. */
        public static final int    FEEDER_MOTOR_ID = 25;
        public static final String FEEDER_CAN_BUS  = "rio";

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

        /** Duty-cycle power for the uptake roller when ejecting (0–1). */
        public static final double ROLLER_POWER    = 0.9;
        /** Duty-cycle power for the intake bar/rod motors when ejecting (0–1). */
        public static final double BAR_POWER       = 0.9;
        /** Duty-cycle power for the uptake roller when intaking (0–1). */
        public static final double ROLLER_POWER_IN = 0.05;
        /** Duty-cycle power for the intake bar/rod motors when intaking (0–1). */
        public static final double BAR_POWER_IN    = 0.9;

        private IntakeConstants() {}
    }
}