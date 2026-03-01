package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ShooterConstants;

/**
 * ShotCalculator — ballistic solver + optimal-angle finder using the exact
 * physics from {@code frc_v0_calculator.html}.
 *
 * <h2>Physics model (directly from the HTML calculator)</h2>
 * <pre>
 *   h   = h2 − h1                          (net height the ball must gain)
 *   v₀  = d × √( g / ( 2·cos²θ · (d·tanθ − h) ) )
 *   t   = d / (v₀ · cosθ)                  (flight time)
 *   vy_final = v₀·sinθ − g·t              (vertical velocity at target)
 *   entry_angle = atan2(−vy_final, v₀·cosθ)   (how steeply the ball falls into hub)
 * </pre>
 *
 * <h2>Optimal angle finder</h2>
 * Sweeps every 0.5° from {@link ShooterConstants#MIN_LAUNCH_ANGLE_DEG} to
 * {@link ShooterConstants#MAX_LAUNCH_ANGLE_DEG} and scores each candidate using
 * the same weighted formula as the HTML Optimizer tab:
 * <pre>
 *   tScore   = max(0, 1 − t / 1.2)           (shorter flight = better)
 *   v0Score  = max(0, 1 − v0 / 20)           (lower exit speed = better)
 *   eaScore  = min(1, entryAngle / 65)        (steeper entry = better accuracy)
 *   rpmScore = max(0, 1 − motorRPM / 6000)   (lower RPM = better)
 *   score    = w[0]·tScore + w[1]·v0Score + w[2]·eaScore + w[3]·rpmScore
 * </pre>
 * Weights come from {@link ShooterConstants#SHOT_SCORE_WEIGHTS}
 * (default: [0.15, 0.15, 0.55, 0.15] — accuracy-biased, matching the HTML
 * "Max accuracy" priority because entry angle is the most important factor for
 * consistently clearing the hub rim).
 *
 * <h2>Calibration / LUT</h2>
 * After the optimal angle is determined for a given distance, the required RPM
 * is computed analytically from the same physics (no separate LUT needed).
 * If you prefer to use tested values, you can still override
 * {@link #overrideRPM(double, double)} entries in the static initialiser.
 */
public final class ShotCalculator {

    // ── Physical constants (from the HTML calculator) ─────────────────────────
    private static final double G              = ShooterConstants.g;
    private static final double H1             = ShooterConstants.SHOOTER_EXIT_HEIGHT_METERS;
    private static final double H2             = ShooterConstants.HUB_TARGET_HEIGHT_METERS;
    private static final double DELTA_H        = H2 - H1;   // net height to gain

    // ── Flywheel geometry (for RPM conversion, matching HTML RPM tab) ─────────
    private static final double WHEEL_DIAM_M   =
        ShooterConstants.WHEEL_DIAMETER_INCHES * 0.0254;   // inches → metres
    private static final double ETA            = ShooterConstants.EFFICIENCY;
    private static final double GEAR_RATIO     = ShooterConstants.SHOOTER_GEAR_RATIO;

    private ShotCalculator() {} // static-only utility class

    // ── Public result record ───────────────────────────────────────────────────

    /**
     * @param rpm          Motor RPM setpoint — use this in {@code setFlywheelRPM()}.
     * @param hoodDeg      Optimal launch angle (degrees above horizontal).
     * @param v0           Ball exit speed (m/s) — for diagnostics.
     * @param flightTime   Estimated flight time (seconds) — for diagnostics / ToF.
     * @param entryAngle   Ball entry angle into the hub (degrees) — >35° is "good".
     */
    public record ShotResult(
            double rpm,
            double hoodDeg,
            double v0,
            double flightTime,
            double entryAngle) {}

    // ── Primary API ───────────────────────────────────────────────────────────

    /**
     * Calculate the optimal shot parameters for a given distance.
     *
     * <p>Sweeps launch angles and scores each one using the weighted formula from
     * the HTML Optimizer tab, then returns the highest-scoring valid angle and
     * its derived RPM.
     *
     * @param distanceMeters Horizontal distance from shooter exit to hub centre (metres).
     * @return {@link ShotResult} for the best angle, or a safe fallback if no
     *         valid angle exists at this distance.
     */
    public static ShotResult calculate(double distanceMeters) {
        double d = MathUtil.clamp(distanceMeters, 0.5, 8.0);

        double bestScore  = -1.0;
        double bestAngle  = ShooterConstants.MIN_LAUNCH_ANGLE_DEG;
        Physics bestPhys  = null;

        double step = ShooterConstants.ANGLE_SWEEP_STEP_DEG;
        for (double theta = ShooterConstants.MIN_LAUNCH_ANGLE_DEG;
                   theta <= ShooterConstants.MAX_LAUNCH_ANGLE_DEG;
                   theta += step) {

            Physics p = solvePhysics(d, theta);
            if (p == null) continue;                           // ball cannot reach target
            if (p.entryAngle < ShooterConstants.MIN_ENTRY_ANGLE_DEG) continue; // too flat
            double motorRPM = toMotorRPM(p.v0);
            if (motorRPM > ShooterConstants.MAX_MOTOR_RPM) continue;  // flywheel too slow

            double score = scoreAngle(p, motorRPM);
            if (score > bestScore) {
                bestScore = score;
                bestAngle = theta;
                bestPhys  = p;
            }
        }

        if (bestPhys == null) {
            // No valid angle found — return a safe default that doesn't fire
            return new ShotResult(ShooterConstants.IDLE_RPM, 45.0, 0, 0, 0);
        }

        return new ShotResult(
            toMotorRPM(bestPhys.v0),
            bestAngle,
            bestPhys.v0,
            bestPhys.flightTime,
            bestPhys.entryAngle);
    }

    // ── Direct physics query (for diagnostics / AimToHub) ────────────────────

    /**
     * Compute physics for an explicit angle and distance.
     * Returns {@code null} if the ball cannot reach the target.
     *
     * @param distanceMeters Horizontal distance to hub (metres).
     * @param thetaDeg       Launch angle (degrees).
     */
    public static ShotResult calculateAtAngle(double distanceMeters, double thetaDeg) {
        Physics p = solvePhysics(distanceMeters, thetaDeg);
        if (p == null) return null;
        return new ShotResult(
            toMotorRPM(p.v0), thetaDeg, p.v0, p.flightTime, p.entryAngle);
    }

    // ── Internal physics solver ───────────────────────────────────────────────

    /**
     * Exact formula from the HTML calculator:
     * <pre>
     *   denom = 2 · cos²θ · (d·tanθ − h)
     *   v₀    = d · √(g / denom)           [only valid when denom > 0]
     *   t     = d / (v₀ · cosθ)
     *   vy_f  = v₀·sinθ − g·t
     *   ea    = atan2(−vy_f, v₀·cosθ)      [entry angle in degrees]
     * </pre>
     */
    private static Physics solvePhysics(double d, double thetaDeg) {
        double th    = Math.toRadians(thetaDeg);
        double cosT  = Math.cos(th);
        double sinT  = Math.sin(th);
        double denom = 2.0 * cosT * cosT * (d * Math.tan(th) - DELTA_H);

        if (denom <= 0.0 || d <= 0.0) return null;   // ball cannot reach hub at this angle

        double v0    = d * Math.sqrt(G / denom);
        double t     = d / (v0 * cosT);
        double vyFin = v0 * sinT - G * t;
        double ea    = Math.toDegrees(Math.atan2(-vyFin, v0 * cosT));

        return new Physics(v0, t, ea);
    }

    /**
     * Scoring function — identical to the HTML {@code scoreAngle()} function:
     * <pre>
     *   tScore   = max(0, 1 − t   / 1.2)
     *   v0Score  = max(0, 1 − v0  / 20)
     *   eaScore  = min(1, ea / 65)
     *   rpmScore = max(0, 1 − rpm / 6000)
     * </pre>
     * Weights from {@link ShooterConstants#SHOT_SCORE_WEIGHTS}.
     */
    private static double scoreAngle(Physics p, double motorRPM) {
        double tScore   = Math.max(0.0, 1.0 - p.flightTime / 1.2);
        double v0Score  = Math.max(0.0, 1.0 - p.v0 / 20.0);
        double eaScore  = Math.min(1.0, p.entryAngle / 65.0);
        double rpmScore = Math.max(0.0, 1.0 - motorRPM / 6000.0);
        double[] w      = ShooterConstants.SHOT_SCORE_WEIGHTS;
        return w[0]*tScore + w[1]*v0Score + w[2]*eaScore + w[3]*rpmScore;
    }

    /**
     * Convert exit speed v₀ to motor RPM — exact formula from the HTML RPM tab:
     * <pre>
     *   surface_speed = v₀ / η
     *   wheel_RPM     = (surface_speed / (π × diameter)) × 60
     *   motor_RPM     = wheel_RPM × gear_ratio
     * </pre>
     */
    private static double toMotorRPM(double v0) {
        double surfaceSpeed = v0 / ETA;
        double wheelRPM     = (surfaceSpeed / (Math.PI * WHEEL_DIAM_M)) * 60.0;
        return wheelRPM * GEAR_RATIO;
    }

    /** Internal physics result — not exposed publicly. */
    private record Physics(double v0, double flightTime, double entryAngle) {}
}